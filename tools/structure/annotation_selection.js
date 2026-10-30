/* Pure, conservative point picking for the annotation canvas. Coordinates are
 * backing-store pixels (not CSS pixels), with Y downward. Rebuild visibility
 * after changing the camera, viewport, point size, membership, or hidden layers.
 *
 * The CPU buffer follows the point shader's circular disks and WebGL LESS depth
 * test at pixel centers. MSAA edge coverage can differ by a fraction of a pixel;
 * no expanded pick radius is used to make hidden points selectable.
 */
(function (root, factory) {
  'use strict';
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.AtlasPointSelection = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const maximumPixels = 16_777_216;

  function buildVisibility({points, memberships, visibleGroups, vp, width, height, pointSize}) {
    if (!Number.isInteger(width) || !Number.isInteger(height) || width < 1 || height < 1 ||
        width * height > maximumPixels) throw new RangeError('Invalid point-selection viewport');
    if (!Number.isFinite(pointSize) || pointSize <= 0 || pointSize > 1024)
      throw new RangeError('Invalid rendered point size');
    if (!vp || vp.length !== 16 || !Array.from(vp).every(Number.isFinite))
      throw new TypeError('Point selection requires a finite column-major view-projection matrix');
    if (!points || !memberships || !visibleGroups || points.length !== memberships.length ||
        points.length > 0x7fffffff) throw new TypeError('Point membership must preserve canonical indices');

    const pointIndices = new Int32Array(width * height);
    pointIndices.fill(-1);
    const depth = new Float32Array(width * height);
    depth.fill(1); // gl.clearDepth(1), with the renderer's default gl.LESS.
    const radius = pointSize / 2, radiusSquared = radius * radius;
    for (let index = 0; index < points.length; index++) {
      const group = memberships[index];
      if (!Number.isInteger(group) || group < 0 || !visibleGroups[group]) continue;
      const point = points[index];
      if (!point || point.length !== 3) continue;
      // The position attribute uploaded to WebGL is Float32 even if the JSON
      // retains more decimals. Keeping that conversion avoids outlier mismatch.
      const x = Math.fround(point[0]), y = Math.fround(point[1]), z = Math.fround(point[2]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      const w = vp[3] * x + vp[7] * y + vp[11] * z + vp[15];
      if (!Number.isFinite(w) || w <= 0) continue;
      const cx = vp[0] * x + vp[4] * y + vp[8] * z + vp[12];
      const cy = vp[1] * x + vp[5] * y + vp[9] * z + vp[13];
      const cz = vp[2] * x + vp[6] * y + vp[10] * z + vp[14];
      // OpenGL clips a point by its center before rasterizing its disk.
      if (cx < -w || cx > w || cy < -w || cy > w || cz < -w || cz > w) continue;
      const sx = (cx / w + 1) * width / 2, sy = (1 - cy / w) * height / 2;
      const pointDepth = Math.fround(cz / w * 0.5 + 0.5);
      const left = Math.max(0, Math.ceil(sx - radius - 0.5));
      const right = Math.min(width - 1, Math.floor(sx + radius - 0.5));
      const top = Math.max(0, Math.ceil(sy - radius - 0.5));
      const bottom = Math.min(height - 1, Math.floor(sy + radius - 0.5));
      for (let py = top; py <= bottom; py++) {
        const dy = py + 0.5 - sy, offset = py * width;
        for (let px = left; px <= right; px++) {
          const dx = px + 0.5 - sx;
          if (dx * dx + dy * dy > radiusSquared) continue;
          const pixel = offset + px;
          if (pointDepth < depth[pixel]) {
            pointIndices[pixel] = index;
            depth[pixel] = pointDepth;
          }
        }
      }
    }
    return {width, height, pointIndices, depth};
  }

  // A capsule swept between successive pointer samples prevents holes when
  // Pencil/pointer events arrive far apart. Only winning visible pixels count.
  // A stationary segment is a circular brush. The caller owns add/erase/undo.
  function collectStroke(visibility, from, to, radiusPixels) {
    if (!visibility || !Number.isInteger(visibility.width) || !Number.isInteger(visibility.height) ||
        visibility.width < 1 || visibility.height < 1 ||
        !visibility.pointIndices || visibility.pointIndices.length !== visibility.width * visibility.height)
      throw new TypeError('Invalid visibility buffer');
    if (!from || !to || ![from.x, from.y, to.x, to.y, radiusPixels].every(Number.isFinite) || radiusPixels < 0)
      throw new RangeError('Invalid brush segment');
    const {width, height, pointIndices} = visibility, result = new Set();
    const left = Math.max(0, Math.ceil(Math.min(from.x, to.x) - radiusPixels - 0.5));
    const right = Math.min(width - 1, Math.floor(Math.max(from.x, to.x) + radiusPixels - 0.5));
    const top = Math.max(0, Math.ceil(Math.min(from.y, to.y) - radiusPixels - 0.5));
    const bottom = Math.min(height - 1, Math.floor(Math.max(from.y, to.y) + radiusPixels - 0.5));
    const dx = to.x - from.x, dy = to.y - from.y, lengthSquared = dx * dx + dy * dy;
    const radiusSquared = radiusPixels * radiusPixels;
    for (let py = top; py <= bottom; py++) {
      const offset = py * width;
      for (let px = left; px <= right; px++) {
        const index = pointIndices[offset + px];
        if (index < 0 || result.has(index)) continue;
        const vx = px + 0.5 - from.x, vy = py + 0.5 - from.y;
        const t = lengthSquared ? Math.max(0, Math.min(1, (vx * dx + vy * dy) / lengthSquared)) : 0;
        const ex = vx - dx * t, ey = vy - dy * t;
        if (ex * ex + ey * ey <= radiusSquared) result.add(index);
      }
    }
    return result;
  }

  return Object.freeze({buildVisibility, collectStroke});
});
