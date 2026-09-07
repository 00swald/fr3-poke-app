"""
stl_geometry.py — STL center-of-mass + spherical-feature detection.

Pure geometry: no robot/serial/hardware imports anywhere in this file. Loads an
STL, computes its volume and center of mass (COM), re-origins the mesh on that
point, then looks for spherical surface patches (bumps or divots with a real
crease boundary) and reports each one's center, radius, and the direction you'd
approach it from (pointing from free space into the material).

ALGORITHM
  1. Load triangles, auto-fix globally-inverted winding (negative volume).
  2. Compute volume + COM via signed-tetrahedron integration; recenter on COM.
  3. Weld coincident vertices (KD-tree, mesh-scale-relative tolerance) to get
     real triangle adjacency — STL files store no shared-vertex topology.
  4. Segment the mesh into smooth patches by cutting at "sharp" edges (dihedral
     angle between adjacent face normals above --crease-angle-deg). A spherical
     bump/divot fused into a bigger part becomes its own patch this way, without
     needing to be a separate mesh shell.
  5. Fit a sphere to each sufficiently-large, non-planar patch: linear algebraic
     initial guess, refined against true Euclidean point-to-sphere residuals.
     Validate with an RMS-vs-radius check and a normal/radial consistency check
     (rejects e.g. cylindrical fillets that could otherwise algebraically fit a
     plausible-looking sphere).

KNOWN LIMITATION (by construction, not a tuning bug): a sphere blended
*tangentially* into surrounding geometry has no crease anywhere, at any
threshold — step 4 cannot see it. Machined/printed bumps and dimples (the
expected case for calibration/probe features) have a real crease; smoothly
filleted blends don't. If that ever matters, the fix is incremental
region-growing with a running sphere refit instead of a hard dihedral cutoff —
not implemented here.

Usage:
    python3 stl_geometry.py part.STL --json out.json
    python3 stl_geometry.py part.STL --min-radius 2 --max-radius 8
"""

import argparse
import json
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares

try:
    from stl import mesh as stl_mesh
except ImportError:
    stl_mesh = None


# --------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------

def load_triangles(path):
    """Returns (N,3,3) float64 vertex array. Ignores the file's own stored
    normals entirely -- those are frequently zero/wrong on real exports; every
    normal used anywhere below is recomputed from vertex winding."""
    if stl_mesh is None:
        raise ImportError("numpy-stl is required: pip install numpy-stl")
    m = stl_mesh.Mesh.from_file(path)
    return np.asarray(m.vectors, dtype=np.float64).copy()


# --------------------------------------------------------------------------
# Volume / center of mass
# --------------------------------------------------------------------------

def _signed_tetra_volume_and_com(triangles):
    """Standard divergence-theorem formula: decompose into tetrahedra with the
    origin, sum signed volumes and volume-weighted tetrahedron centroids.
    Correct for a closed, consistently-wound mesh; garbage-in-garbage-out
    (caller is responsible for checking watertightness) otherwise."""
    p1, p2, p3 = triangles[:, 0, :], triangles[:, 1, :], triangles[:, 2, :]
    signed_vol6 = np.einsum('ij,ij->i', p1, np.cross(p2, p3))  # 6x signed tetra volume
    volume = float(signed_vol6.sum() / 6.0)
    if abs(volume) < 1e-12:
        return volume, triangles.reshape(-1, 3).mean(axis=0)
    tetra_centroid = (p1 + p2 + p3) / 4.0  # 4th vertex is the origin
    weights = signed_vol6 / 6.0
    com = (tetra_centroid * weights[:, None]).sum(axis=0) / volume
    return volume, com


def compute_volume_and_com(triangles):
    """Returns (triangles, volume, com, method, sane, warnings).

    triangles may come back with columns 1/2 swapped if the source winding was
    globally inverted (negative volume) -- everything downstream should use the
    returned triangles, not the input, so normals stay consistent.
    """
    warnings = []
    volume, com = _signed_tetra_volume_and_com(triangles)

    if volume < 0:
        triangles = triangles[:, [0, 2, 1], :].copy()
        volume, com = _signed_tetra_volume_and_com(triangles)
        warnings.append(
            "source mesh winding was globally inverted (negative volume) -- "
            "auto-flipped and recomputed")

    all_verts = triangles.reshape(-1, 3)
    bbox_min, bbox_max = all_verts.min(axis=0), all_verts.max(axis=0)
    bbox_volume = float(np.prod(np.maximum(bbox_max - bbox_min, 1e-9)))

    method, sane = "mass_properties", True
    if not np.isfinite(volume) or volume <= 1e-9 * bbox_volume:
        warnings.append(
            f"volume is zero/NaN/implausible ({volume!r}) -- mesh is likely not "
            f"watertight. Falling back to vertex centroid for COM -- this is NOT "
            f"a true center-of-mass, just a geometric approximation.")
        com = all_verts.mean(axis=0)
        method, sane = "vertex_centroid_fallback", False
    elif volume > 1.001 * bbox_volume:
        warnings.append(
            f"computed volume ({volume:.3f}) exceeds the bounding-box volume "
            f"({bbox_volume:.3f}) -- mass-properties result looks wrong; "
            f"treat COM/volume with suspicion")
        sane = False
    elif not np.all((com >= bbox_min - 1e-6) & (com <= bbox_max + 1e-6)):
        warnings.append("computed COM lies outside the mesh bounding box -- "
                         "treat with suspicion")
        sane = False

    return triangles, volume, com, method, sane, warnings


def recenter(triangles, com):
    return triangles - np.asarray(com).reshape(1, 1, 3)


# --------------------------------------------------------------------------
# Face normals / areas
# --------------------------------------------------------------------------

def compute_face_normals_and_areas(triangles):
    """Recomputed from winding (p2-p1) x (p3-p1), NOT the file's stored
    normals. Returns (normals (N,3), areas (N,), degenerate_mask (N,) bool)."""
    p1, p2, p3 = triangles[:, 0, :], triangles[:, 1, :], triangles[:, 2, :]
    raw = np.cross(p2 - p1, p3 - p1)
    twice_area = np.linalg.norm(raw, axis=1)
    positive = twice_area[twice_area > 0]
    mean_twice_area = float(positive.mean()) if len(positive) else 0.0
    degenerate = twice_area < max(mean_twice_area * 1e-6, 1e-15)

    normals = np.zeros_like(raw)
    ok = ~degenerate
    normals[ok] = raw[ok] / twice_area[ok, None]
    areas = twice_area / 2.0
    return normals, areas, degenerate


# --------------------------------------------------------------------------
# Vertex welding + face adjacency
# --------------------------------------------------------------------------

def weld_vertices(triangles, tol_fraction=1e-4):
    """KD-tree + union-find welding with a mesh-scale-relative tolerance
    (fraction of bounding-box diagonal), not fixed-decimal rounding -- rounding
    is scale-blind and has bucket-boundary artifacts where two very-close
    points straddle a rounding boundary and fail to weld.

    Returns (unique_verts (M,3), face_vertex_indices (N,3) int, tol_used)."""
    all_verts = triangles.reshape(-1, 3)
    n = all_verts.shape[0]
    bbox_diag = float(np.linalg.norm(all_verts.max(axis=0) - all_verts.min(axis=0)))
    tol = max(bbox_diag * tol_fraction, 1e-9)

    tree = cKDTree(all_verts)
    pairs = tree.query_pairs(r=tol)

    parent = list(range(n))

    def find(x):
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i, j in pairs:
        union(i, j)

    roots = np.array([find(i) for i in range(n)])
    unique_roots, inverse = np.unique(roots, return_inverse=True)

    unique_verts = np.zeros((len(unique_roots), 3))
    counts = np.zeros(len(unique_roots))
    np.add.at(unique_verts, inverse, all_verts)
    np.add.at(counts, inverse, 1)
    unique_verts /= counts[:, None]

    face_vertex_indices = inverse.reshape(-1, 3).astype(np.int64)
    return unique_verts, face_vertex_indices, tol


def build_edge_face_map(face_vertex_indices):
    """undirected edge (sorted vertex-index pair) -> list of incident face indices."""
    edge_faces = {}
    for f_idx, (a, b, c) in enumerate(face_vertex_indices):
        for u, v in ((a, b), (b, c), (c, a)):
            key = (int(u), int(v)) if u < v else (int(v), int(u))
            edge_faces.setdefault(key, []).append(f_idx)
    return edge_faces


def weld_quality(edge_faces):
    """Fraction of edges with exactly 2 incident triangles -- the expected
    case for a clean closed/manifold mesh. Low values mean either the weld
    tolerance is wrong or the mesh genuinely has holes/non-manifold geometry;
    either way it's worth a warning (segmentation degrades safely -- towards
    over-segmentation -- rather than silently misbehaving)."""
    counts = np.array([len(v) for v in edge_faces.values()])
    if len(counts) == 0:
        return 0.0
    return float(np.mean(counts == 2))


# --------------------------------------------------------------------------
# Crease segmentation
# --------------------------------------------------------------------------

def segment_smooth_patches(face_vertex_indices, face_normals, edge_faces, crease_angle_deg):
    """Connected components of the face-adjacency graph, cutting at edges
    whose dihedral angle exceeds crease_angle_deg, AND at every edge that
    doesn't have exactly 2 incident faces (boundary or non-manifold -- always
    a segmentation break, never an arbitrary pick of which 2 faces to compare).

    Compares cosines directly rather than computing arccos + comparing angles
    -- equivalent (cos is monotonically decreasing on [0,180]) and sidesteps
    the arccos-domain-error class of bug entirely for this hot inner loop."""
    n_faces = len(face_vertex_indices)
    cos_thresh = np.cos(np.radians(crease_angle_deg))
    adj = [[] for _ in range(n_faces)]
    for faces in edge_faces.values():
        if len(faces) != 2:
            continue
        i, j = faces
        d = np.clip(np.dot(face_normals[i], face_normals[j]), -1.0, 1.0)
        if d >= cos_thresh:
            adj[i].append(j)
            adj[j].append(i)

    visited = np.zeros(n_faces, dtype=bool)
    patches = []
    for start in range(n_faces):
        if visited[start]:
            continue
        stack, comp = [start], []
        visited[start] = True
        while stack:
            f = stack.pop()
            comp.append(f)
            for nb in adj[f]:
                if not visited[nb]:
                    visited[nb] = True
                    stack.append(nb)
        patches.append(np.array(comp, dtype=np.int64))
    return patches


# --------------------------------------------------------------------------
# Sphere fitting
# --------------------------------------------------------------------------

@dataclass
class PatchResult:
    accepted: bool
    reason: str
    n_faces: int = 0
    center: Optional[np.ndarray] = None
    radius: Optional[float] = None
    outward_normal: Optional[np.ndarray] = None       # points from material into free space
    contact_point: Optional[np.ndarray] = None         # center + radius * (geometric cap direction)
    rms_residual_frac: Optional[float] = None


def fit_sphere_to_patch(patch_face_idx, face_vertex_indices, unique_verts,
                         face_normals, face_areas, *, min_patch_faces,
                         max_rms_frac, min_radius, max_radius,
                         normal_consistency_deg=35.0, max_radius_footprint_ratio=50.0,
                         min_normal_spread_ratio=1e-6):
    n_faces = len(patch_face_idx)
    if n_faces < min_patch_faces:
        return PatchResult(False, "too_few_faces", n_faces=n_faces)

    patch_normals = face_normals[patch_face_idx]

    # Reject patches whose face normals are confined to a 2D subspace (i.e.
    # the normal-covariance matrix is rank-deficient: its smallest eigenvalue
    # is ~0 relative to its largest). A true 3D spherical patch -- bump,
    # divot, even a very shallow cap, even a full closed sphere -- always
    # uses all 3 dimensions: normals fan out around 2 independent axes, not
    # just 1. The one shape whose normals are EXACTLY confined to a plane is
    # a surface of revolution swept along a straight axis -- a cylinder --
    # e.g. the two rim loops of a low-poly cylindrical through-hole, which
    # can otherwise fit a sphere with near-zero RMS residual purely by
    # symmetry (both rims equidistant from the midpoint between them) despite
    # not being spherical at all. Deliberately NOT a "normals too parallel"
    # filter (an earlier version of that heuristic wrongly rejected real
    # shallow caps, whose normals genuinely are nearly parallel but still
    # span all 3 dimensions) -- this only fires on true rank-2 degeneracy,
    # which a shallow-but-real cap does not exhibit even at extreme
    # shallowness (see test_geometry_selftest.py's 12-degree cap case).
    normal_cov = patch_normals.T @ patch_normals
    normal_eigvals = np.sort(np.linalg.eigvalsh(normal_cov))[::-1]
    if normal_eigvals[0] > 1e-12 and normal_eigvals[-1] / normal_eigvals[0] < min_normal_spread_ratio:
        return PatchResult(False, "normals_coplanar", n_faces=n_faces)

    vert_idx = np.unique(face_vertex_indices[patch_face_idx].reshape(-1))
    pts = unique_verts[vert_idx]
    if len(pts) < 4:
        return PatchResult(False, "too_few_vertices", n_faces=n_faces)

    # Footprint (patch's own spatial extent) -- used below to sanity-bound the
    # fitted radius. NOTE: there is deliberately no normal-spread/"are these
    # normals nearly parallel" pre-filter here. A genuinely shallow spherical
    # cap has, by definition, nearly-parallel normals -- that heuristic can't
    # tell "shallow but real" apart from "actually flat" and was found (via
    # test_geometry_selftest.py) to reject real shallow caps outright, which
    # defeats the whole point of the geometric refinement below. The correct
    # guard against flat/degenerate patches is applied AFTER fitting, against
    # the fitted radius itself (see max_radius_footprint_ratio below).
    # bbox diagonal (cheap upper bound on max pairwise distance -- exact value
    # doesn't matter, this only feeds a generous order-of-magnitude sanity check)
    footprint_diameter = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))

    # --- algebraic (Kasa) initial guess, locally recentered for conditioning ---
    local_origin = pts.mean(axis=0)
    local = pts - local_origin
    A = np.hstack([2 * local, np.ones((len(local), 1))])
    b = np.sum(local ** 2, axis=1)
    sol, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 4:
        return PatchResult(False, "rank_deficient_fit", n_faces=n_faces)
    c0 = sol[:3]
    r_sq0 = float(sol[3] + np.dot(c0, c0))
    if not np.isfinite(r_sq0) or r_sq0 <= 0:
        return PatchResult(False, "non_positive_radius_squared", n_faces=n_faces)
    r0 = float(np.sqrt(r_sq0))

    # --- refine against true Euclidean point-to-sphere residuals. The linear
    # fit above is known to be biased on shallow caps -- exactly what a small
    # probe-contact bump/divot looks like -- so this refinement is load-bearing,
    # not optional polish. ---
    def residuals(params):
        c, r = params[:3], params[3]
        return np.linalg.norm(local - c, axis=1) - r

    fit = least_squares(residuals, np.concatenate([c0, [r0]]))
    c_local, radius = fit.x[:3], float(fit.x[3])
    if not np.isfinite(radius) or radius <= 0:
        return PatchResult(False, "refine_failed", n_faces=n_faces)
    center = local_origin + c_local

    # Reject fits where the radius is wildly large relative to how big the
    # patch itself is -- the real signature of a near-planar/degenerate fit
    # (a flat patch is "explained" almost as well by a huge-radius sphere as
    # by a plane, since a huge sphere is locally nearly flat). A genuinely
    # shallow-but-real cap still has a bounded, sane radius-to-footprint
    # ratio; see the note above fit_sphere_to_patch for why this replaced an
    # earlier normal-spread pre-filter that incorrectly rejected real shallow
    # caps outright.
    if footprint_diameter > 1e-9 and radius > max_radius_footprint_ratio * footprint_diameter:
        return PatchResult(False, "radius_footprint_ratio_too_large", n_faces=n_faces, radius=radius)

    if min_radius is not None and radius < min_radius:
        return PatchResult(False, "radius_below_min", n_faces=n_faces, radius=radius)
    if max_radius is not None and radius > max_radius:
        return PatchResult(False, "radius_above_max", n_faces=n_faces, radius=radius)

    resid = np.linalg.norm(pts - center, axis=1) - radius
    rms_frac = float(np.sqrt(np.mean(resid ** 2)) / radius)
    if rms_frac > max_rms_frac:
        return PatchResult(False, "rms_too_high", n_faces=n_faces, radius=radius,
                            rms_residual_frac=rms_frac)

    # --- normal/radial consistency: rejects patches that algebraically fit a
    # sphere but aren't really one (e.g. a cylindrical fillet) ---
    face_verts = unique_verts[face_vertex_indices[patch_face_idx]]
    face_centroids = face_verts.mean(axis=1)
    radial = face_centroids - center
    radial_len = np.linalg.norm(radial, axis=1, keepdims=True)
    radial_norm = radial / np.clip(radial_len, 1e-12, None)
    cos_align = np.clip(np.einsum('ij,ij->i', radial_norm, patch_normals), -1.0, 1.0)
    frac_aligned = float(np.mean(np.abs(cos_align) >= np.cos(np.radians(normal_consistency_deg))))
    if frac_aligned < 0.8:
        return PatchResult(False, "normal_radial_inconsistent", n_faces=n_faces,
                            radius=radius, rms_residual_frac=rms_frac)

    # --- cap_direction: purely geometric "where on the sphere is this patch",
    # area-weighted mean of the (always well-defined) per-face radial direction.
    # Used for contact_point -- independent of whether STL normals are trustworthy. ---
    weights = face_areas[patch_face_idx]
    cap_vec = np.sum(radial_norm * weights[:, None], axis=0)
    cap_len = float(np.linalg.norm(cap_vec))
    if cap_len < 1e-9:
        return PatchResult(False, "cap_direction_degenerate", n_faces=n_faces,
                            radius=radius, rms_residual_frac=rms_frac)
    cap_direction = cap_vec / cap_len

    # --- majority_sign: does material sit inside (bump) or outside (divot) the
    # sphere here? The one place we trust the STL face normals' *sign* -- as a
    # majority vote, so a few individually-inverted-winding faces can't skew it. ---
    signs = np.sign(cos_align)
    signs[signs == 0] = 1.0
    majority_sign = 1.0 if np.sum(signs > 0) >= np.sum(signs < 0) else -1.0

    outward_normal = majority_sign * cap_direction          # material -> free space
    contact_point = center + radius * cap_direction          # always ON the sphere

    return PatchResult(True, "accepted", n_faces=n_faces, center=center, radius=radius,
                        outward_normal=outward_normal, contact_point=contact_point,
                        rms_residual_frac=rms_frac)


def flag_close_pairs(results, center_factor=0.5, radius_rel_tol=0.25):
    """Indices of accepted results that look like the SAME physical sphere
    detected twice (e.g. one sphere accidentally split into two patches by
    segmentation) -- not just spatially nearby. Requires both:
      - centers much closer together than the smaller radius (near-coincident
        centers, as you'd get fitting the same underlying sphere twice), and
      - radii within radius_rel_tol of each other.
    Using distance-vs-max(radius) alone (an earlier version of this function)
    over-triggers: a small 4mm corner fillet and an unrelated 30mm body dome
    can be well within "1.5x the big one's radius" of each other while being
    completely different, non-duplicate features. NOT merged either way --
    connected components already partition faces disjointly, so a genuine
    duplicate means either two real close-together physical spheres or a
    segmentation artifact; flagged for a human to look at, not silently
    resolved."""
    flags = {i: [] for i in range(len(results))}
    for i in range(len(results)):
        for j in range(i + 1, len(results)):
            a, b = results[i], results[j]
            dist = float(np.linalg.norm(a.center - b.center))
            radius_rel_diff = abs(a.radius - b.radius) / max(a.radius, b.radius)
            if dist < center_factor * min(a.radius, b.radius) and radius_rel_diff < radius_rel_tol:
                flags[i].append(j)
                flags[j].append(i)
    return flags


# --------------------------------------------------------------------------
# Orchestration
# --------------------------------------------------------------------------

def find_spheres(path, *, crease_angle_deg=20.0, min_patch_faces=6,
                  max_rms_frac=0.02, min_radius=None, max_radius=None,
                  weld_tol_fraction=1e-4, min_normal_spread_ratio=1e-6):
    """Full pipeline. Returns a dict ready to serialize:
        {volume, com, com_method, com_sane, warnings, weld_quality,
         degenerate_faces_dropped, n_patches, spheres: [...], rejections: {reason: count},
         close_pairs: {index: [other indices]}}
    All sphere coordinates are in the COM-centered frame (COM already
    subtracted) -- `com` is reported in the STL file's original coordinates so
    callers can go back and forth unambiguously.
    """
    warnings = []
    triangles = load_triangles(path)
    triangles, volume, com, com_method, com_sane, com_warnings = compute_volume_and_com(triangles)
    warnings.extend(com_warnings)
    triangles = recenter(triangles, com)

    normals, areas, degenerate = compute_face_normals_and_areas(triangles)
    n_dropped = int(degenerate.sum())
    if n_dropped:
        keep = ~degenerate
        triangles, normals, areas = triangles[keep], normals[keep], areas[keep]
        warnings.append(f"dropped {n_dropped} zero-area (degenerate) triangle(s) before "
                         f"adjacency/normal work")

    unique_verts, face_vertex_indices, tol = weld_vertices(triangles, weld_tol_fraction)
    edge_faces = build_edge_face_map(face_vertex_indices)
    wq = weld_quality(edge_faces)
    if wq < 0.9:
        warnings.append(f"only {wq*100:.1f}% of edges have exactly 2 incident triangles after "
                         f"welding (tol={tol:.4g}) -- mesh may be non-watertight, or the weld "
                         f"tolerance may need adjusting via weld_tol_fraction; segmentation will "
                         f"be more fragmented as a result, not silently wrong")

    patches = segment_smooth_patches(face_vertex_indices, normals, edge_faces, crease_angle_deg)

    results = []
    rejections = {}
    for patch in patches:
        r = fit_sphere_to_patch(patch, face_vertex_indices, unique_verts, normals, areas,
                                 min_patch_faces=min_patch_faces, max_rms_frac=max_rms_frac,
                                 min_radius=min_radius, max_radius=max_radius,
                                 min_normal_spread_ratio=min_normal_spread_ratio)
        if r.accepted:
            results.append(r)
        else:
            rejections[r.reason] = rejections.get(r.reason, 0) + 1

    close_pairs = flag_close_pairs(results)

    return {
        "source_file": path,
        "volume": volume,
        "com": com.tolist(),
        "com_method": com_method,
        "com_sane": com_sane,
        "weld_quality": wq,
        "degenerate_faces_dropped": n_dropped,
        "n_patches": len(patches),
        "warnings": warnings,
        "rejections": rejections,
        "close_pairs": {k: v for k, v in close_pairs.items() if v},
        "spheres": [
            {
                "index": i,
                "center": r.center.tolist(),
                "radius": r.radius,
                "outward_normal": r.outward_normal.tolist(),
                "contact_point": r.contact_point.tolist(),
                "n_faces": r.n_faces,
                "rms_residual_frac": r.rms_residual_frac,
            }
            for i, r in enumerate(results)
        ],
    }


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl_path")
    ap.add_argument("--json", help="write full results to this JSON path")
    ap.add_argument("--crease-angle-deg", type=float, default=20.0,
                     help="dihedral angle above which an edge is a segmentation boundary (default 20)")
    ap.add_argument("--min-patch-faces", type=int, default=6,
                     help="minimum triangle count for a patch to be considered (default 6)")
    ap.add_argument("--max-rms-frac", type=float, default=0.02,
                     help="max allowed fit RMS residual / radius (default 0.02 = 2%%)")
    ap.add_argument("--min-radius", type=float, default=None, help="reject spheres smaller than this (mesh units)")
    ap.add_argument("--max-radius", type=float, default=None, help="reject spheres larger than this (mesh units)")
    ap.add_argument("--min-normal-spread-ratio", type=float, default=1e-6,
                     help="reject patches whose face normals are confined to a 2D plane (default "
                          "1e-6) -- catches e.g. the two rim loops of a cylindrical through-hole, "
                          "which can fit a sphere with near-zero RMS by symmetry alone despite not "
                          "being spherical; real caps (however shallow) always use all 3 dimensions")
    args = ap.parse_args(argv)

    result = find_spheres(args.stl_path, crease_angle_deg=args.crease_angle_deg,
                           min_patch_faces=args.min_patch_faces, max_rms_frac=args.max_rms_frac,
                           min_radius=args.min_radius, max_radius=args.max_radius,
                           min_normal_spread_ratio=args.min_normal_spread_ratio)

    print(f"{args.stl_path}")
    print(f"  volume: {result['volume']:.4f}  (com method: {result['com_method']}, "
          f"sane: {result['com_sane']})")
    print(f"  com (original frame): {result['com']}")
    print(f"  weld quality: {result['weld_quality']*100:.1f}%  "
          f"degenerate faces dropped: {result['degenerate_faces_dropped']}")
    for w in result["warnings"]:
        print(f"  WARNING: {w}")
    print(f"  {result['n_patches']} candidate patch(es) -> {len(result['spheres'])} sphere(s) accepted")
    if result["rejections"]:
        print(f"  rejected patches by reason: {result['rejections']}")
    for s in result["spheres"]:
        # NOTE: close_pairs keys are plain ints here (pre-JSON-serialization);
        # json.dump() below stringifies them in the file, same as any JSON object key.
        flag = " (CLOSE TO ANOTHER DETECTION -- see close_pairs)" if s["index"] in result["close_pairs"] else ""
        print(f"  sphere[{s['index']}]: center={_fmt(s['center'])} radius={s['radius']:.4f} "
              f"contact_point={_fmt(s['contact_point'])} outward_normal={_fmt(s['outward_normal'])} "
              f"n_faces={s['n_faces']} rms_frac={s['rms_residual_frac']:.4f}{flag}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=2)
        print(f"  wrote {args.json}")

    return 0


def _fmt(v):
    return "(" + ", ".join(f"{x:.4f}" for x in v) + ")"


if __name__ == "__main__":
    sys.exit(main())
