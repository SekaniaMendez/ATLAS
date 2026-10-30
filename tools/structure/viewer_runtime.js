// WebGL drawing, camera controls and selection; shares the viewer closure.
function multiply(a, b) {
  const r = new Float32Array(16);
  for (let col = 0; col < 4; col++) for (let row = 0; row < 4; row++) for (let k = 0; k < 4; k++) r[col * 4 + row] += a[k * 4 + row] * b[col * 4 + k];
  return r;
}
function viewProjection() {
  eye = add(center, [distance * Math.cos(el) * Math.cos(az), distance * Math.cos(el) * Math.sin(az), distance * Math.sin(el)]);
  const z = unit(sub(eye, center)),
    x = unit(cross([0, 0, 1], z)),
    y = cross(z, x);
  lastRight = x;
  lastUp = y;
  const view = new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, eye), -dot(y, eye), -dot(z, eye), 1]);
  const aspect = c.width / c.height,
    f = 1 / Math.tan(Math.PI / 8),
    near = Math.max(radius * 0.001, 0.0001),
    far = Math.max(distance + radius * 4, near + 1);
  const projection = new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) / (near - far), -1, 0, 0, 2 * far * near / (near - far), 0]);
  return multiply(projection, view);
}
function attribute(b, l) {
  gl.bindBuffer(gl.ARRAY_BUFFER, b);
  gl.enableVertexAttribArray(l);
  gl.vertexAttribPointer(l, 3, gl.FLOAT, false, 0, 0);
}
function visible(o) {
  return o.s.kind === 'freeform' ? $('freeform').checked : $('planes').checked;
}
function drawAxes(origin, axes, length) {
  gl.disableVertexAttribArray(loc.n);
  gl.vertexAttrib3f(loc.n, 0, 0, 1);
  gl.uniform1f(loc.unlit, 1);
  for (let k = 0; k < 3; k++) {
    const end = add(origin, mul(axes.map(r => r[k]), length)),
      b = buffer([...origin, ...end]);
    attribute(b, loc.p);
    gl.uniform3fv(loc.color, [[1, .28, .28], [.29, .92, .46], [.3, .57, 1]][k]);
    gl.drawArrays(gl.LINES, 0, 2);
    gl.deleteBuffer(b);
  }
}
function structuralAxes() {
  if (!sf) return null;
  let axes = sf.axes,
    origin = sf.origin ?? sf.origin_m;
  if ((!axes || !origin) && sf.T_map_structural) {
    const t = sf.T_map_structural;
    axes = t.slice(0, 3).map(row => row.slice(0, 3));
    origin = t.slice(0, 3).map(row => row[3]);
  }
  return axes && origin ? {
    axes,
    origin
  } : null;
}
function render() {
  dirty = false;
  const w = Math.round(c.clientWidth * Math.min(devicePixelRatio || 1, 2)),
    h = Math.round(c.clientHeight * Math.min(devicePixelRatio || 1, 2));
  if (c.width !== w || c.height !== h) {
    c.width = w;
    c.height = h;
  }
  gl.viewport(0, 0, w, h);
  gl.clearColor(0, 0, 0, 0);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  gl.disable(gl.CULL_FACE);
  gl.useProgram(prog);
  lastVP = viewProjection();
  gl.uniformMatrix4fv(loc.vp, false, lastVP);
  gl.uniform1f(loc.unlit, 0);
  gl.enable(gl.POLYGON_OFFSET_FILL);
  gl.polygonOffset(1, 1);
  for (const o of objects) {
    if (!visible(o)) continue;
    attribute(o.p, loc.p);
    attribute(o.n, loc.n);
    const color = o.s.color.map(v => o.index === selected ? Math.min(1, v * 1.13 + .04) : v * .82);
    gl.uniform3fv(loc.color, color);
    gl.drawArrays(gl.TRIANGLES, 0, o.count);
  }
  gl.disable(gl.POLYGON_OFFSET_FILL);
  gl.uniform1f(loc.unlit, 1);
  gl.disableVertexAttribArray(loc.n);
  gl.vertexAttrib3f(loc.n, 0, 0, 1);
  if ($('wire').checked) {
    for (const o of objects) {
      if (!visible(o)) continue;
      attribute(o.edge, loc.p);
      gl.uniform3fv(loc.color, [.12, .18, .22]);
      gl.drawArrays(gl.LINES, 0, o.edges);
    }
  }
  if ($('axes').checked) {
    gl.disable(gl.DEPTH_TEST);
    const o = objects[selected];
    if (o && visible(o)) drawAxes(o.s.origin, o.s.axes, Math.max(.18, Math.min(.65, radius * .13)));
    const f = structuralAxes();
    if (f) drawAxes(f.origin, f.axes, Math.max(.3, Math.min(1.0, radius * .2)));
  }
  gl.flush();
}
function request() {
  if (!dirty) {
    dirty = true;
    requestAnimationFrame(render);
  }
}
function pick(px, py) {
  const rect = c.getBoundingClientRect(),
    x = (2 * (px - rect.left) / rect.width - 1) * (rect.width / rect.height) * Math.tan(Math.PI / 8),
    y = (1 - 2 * (py - rect.top) / rect.height) * Math.tan(Math.PI / 8),
    direction = unit(add(add(unit(sub(center, eye)), mul(lastRight, x)), mul(lastUp, y)));
  let nearest = Infinity,
    hit = -1;
  for (const o of objects) {
    if (!visible(o)) continue;
    for (const t of o.s.triangles) {
      const a = o.vertices[t[0]],
        b = o.vertices[t[1]],
        d = o.vertices[t[2]],
        e1 = sub(b, a),
        e2 = sub(d, a),
        p = cross(direction, e2),
        det = dot(e1, p);
      if (Math.abs(det) < 1e-10) continue;
      const inv = 1 / det,
        s = sub(eye, a),
        u = dot(s, p) * inv;
      if (u < 0 || u > 1) continue;
      const q = cross(s, e1),
        v = dot(direction, q) * inv;
      if (v < 0 || u + v > 1) continue;
      const dist = dot(e2, q) * inv;
      if (dist > 0 && dist < nearest) {
        nearest = dist;
        hit = o.index;
      }
    }
  }
  if (hit >= 0) select(hit);
}
const pointers = new Map();
let dragDistance = 0,
  lastPinch = 0;
c.onpointerdown = e => {
  c.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, [e.clientX, e.clientY]);
  dragDistance = 0;
  lastPinch = 0;
};
c.onpointermove = e => {
  const old = pointers.get(e.pointerId);
  if (!old) return;
  const dx = e.clientX - old[0],
    dy = e.clientY - old[1];
  pointers.set(e.pointerId, [e.clientX, e.clientY]);
  dragDistance += Math.abs(dx) + Math.abs(dy);
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()],
      p = Math.hypot(a[0] - b[0], a[1] - b[1]);
    if (lastPinch > 0) distance = Math.max(radius * .15, Math.min(radius * 20, distance * lastPinch / p));
    lastPinch = p;
  } else {
    az -= dx * .007;
    el = Math.max(-1.56, Math.min(1.56, el + dy * .007));
  }
  request();
};
c.onpointerup = e => {
  if (pointers.size === 1 && dragDistance < 5) pick(e.clientX, e.clientY);
  pointers.delete(e.pointerId);
  lastPinch = 0;
};
c.onpointercancel = e => pointers.delete(e.pointerId);
c.onwheel = e => {
  e.preventDefault();
  distance = Math.max(radius * .15, Math.min(radius * 20, distance * Math.exp(e.deltaY * .001)));
  request();
};
$('reset').onclick = () => {
  az = -1.1;
  el = .55;
  distance = radius * 2.8;
  request();
};
$('top').onclick = () => {
  az = -Math.PI / 2;
  el = 1.56;
  distance = radius * 2.8;
  request();
};
$('front').onclick = () => {
  az = -Math.PI / 2;
  el = 0;
  distance = radius * 2.8;
  request();
};
$('zoomIn').onclick = () => {
  distance = Math.max(radius * .15, distance / 1.25);
  request();
};
$('zoomOut').onclick = () => {
  distance = Math.min(radius * 20, distance * 1.25);
  request();
};
for (const id of ['planes', 'freeform', 'axes', 'wire']) $(id).onchange = request;
c.onkeydown = e => {
  if (e.key === 'ArrowLeft') az -= .12;else if (e.key === 'ArrowRight') az += .12;else if (e.key === 'ArrowUp') el = Math.min(1.56, el + .12);else if (e.key === 'ArrowDown') el = Math.max(-1.56, el - .12);else if (e.key === '+' || e.key === '=') distance /= 1.15;else if (e.key === '-') distance *= 1.15;else return;
  e.preventDefault();
  request();
};
new ResizeObserver(request).observe(c);
select(0);
request();
window.atlasViewer = {
  surfaceCount: surfaces.length,
  triangleCount: surfaces.reduce((n, s) => n + s.triangles.length, 0),
  select,
  render,
  metadata: meta
};
