// grid.js -- a 36x72 optical-table hole-grid widget, shared by Mode 1
// (mount hole selection) and Mode 3 (height-map center selection).
//
// Rendered on a <canvas>, not a per-cell DOM table: with n_cols*n_rows up
// to ~2592 cells, a DOM table is unnecessary weight for something that's
// really just click-to-select-a-dot. A single canvas redraw per selection
// change is trivially fast at this scale.

class HoleGrid {
  constructor(canvas, opts) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.nCols = (opts && opts.nCols) || 72;
    this.nRows = (opts && opts.nRows) || 36;
    this.onSelect = (opts && opts.onSelect) || function () {};
    this.selected = null; // {col, row}
    this.overlayPoints = []; // extra markers, e.g. mode3's preview probe grid

    this._layout();
    canvas.addEventListener("click", (e) => this._onClick(e));
    this.draw();
  }

  _layout() {
    const w = this.canvas.width;
    const h = this.canvas.height;
    this.marginX = 20;
    this.marginY = 20;
    this.cellW = (w - 2 * this.marginX) / (this.nCols - 1);
    this.cellH = (h - 2 * this.marginY) / (this.nRows - 1);
  }

  setGridSize(nCols, nRows) {
    this.nCols = nCols;
    this.nRows = nRows;
    this._layout();
    this.draw();
  }

  _onClick(e) {
    const rect = this.canvas.getBoundingClientRect();
    const scaleX = this.canvas.width / rect.width;
    const scaleY = this.canvas.height / rect.height;
    const px = (e.clientX - rect.left) * scaleX;
    const py = (e.clientY - rect.top) * scaleY;
    let col = Math.round((px - this.marginX) / this.cellW);
    let row = Math.round((py - this.marginY) / this.cellH);
    col = Math.max(0, Math.min(this.nCols - 1, col));
    row = Math.max(0, Math.min(this.nRows - 1, row));
    this.select(col, row);
  }

  select(col, row, silent) {
    this.selected = { col, row };
    this.draw();
    if (!silent) this.onSelect(col, row);
  }

  setOverlayPoints(points) {
    // points: [{col, row}, ...] in fractional col/row space, purely a
    // client-side preview -- the server recomputes authoritatively.
    this.overlayPoints = points || [];
    this.draw();
  }

  _xy(col, row) {
    return [this.marginX + col * this.cellW, this.marginY + row * this.cellH];
  }

  draw() {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    ctx.fillStyle = "#c7cdd6";
    for (let r = 0; r < this.nRows; r++) {
      for (let c = 0; c < this.nCols; c++) {
        const [x, y] = this._xy(c, r);
        ctx.beginPath();
        ctx.arc(x, y, 1.4, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    if (this.overlayPoints.length) {
      ctx.strokeStyle = "#2563eb88";
      ctx.lineWidth = 1;
      for (const p of this.overlayPoints) {
        const [x, y] = this._xy(p.col, p.row);
        ctx.beginPath();
        ctx.moveTo(x - 3, y);
        ctx.lineTo(x + 3, y);
        ctx.moveTo(x, y - 3);
        ctx.lineTo(x, y + 3);
        ctx.stroke();
      }
    }

    if (this.selected) {
      const [x, y] = this._xy(this.selected.col, this.selected.row);
      ctx.fillStyle = "#dc2626";
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}
