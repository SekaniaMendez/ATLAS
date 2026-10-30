// Scene preparation and inspection UI; inlined by export_geometry.py.
const data = JSON.parse(document.getElementById('atlas-data').textContent),
  meta = data.metadata || {},
  surfaces = data.surfaces || [],
  $ = id => document.getElementById(id);
const types = {
  horizontal: 'Plano horizontal',
  vertical: 'Plano vertical',
  inclined: 'Plano inclinado',
  freeform: 'Superficie irregular'
};
const fmt = (n, d = 2) => n != null && Number.isFinite(Number(n)) ? Number(n).toLocaleString('es', {
  maximumFractionDigits: d
}) : '—';
const size = n => !Number.isFinite(n) ? '—' : n < 1024 ? `${n} B` : n < 1048576 ? `${fmt(n / 1024, 1)} KB` : `${fmt(n / 1048576, 2)} MB`;
const coverage = meta.coverage_ratio ?? meta.point_coverage,
  inputBytes = Number(meta.input_bytes),
  modelBytes = Number(meta.model_bytes ?? meta.geometry_bytes);
$('source').textContent = meta.source_name || meta.source_file || 'Captura local · unidades en metros';
$('surfaceCount').textContent = surfaces.length;
$('coverage').textContent = typeof coverage === 'number' ? `${fmt(100 * coverage, 1)} %` : '—';
$('size').textContent = size(modelBytes);
$('size-label').textContent = meta.model_bytes != null ? 'GLB + referencias' : 'Tamaño del GLB';
if (meta.status === 'partial_model') $('model-status').textContent = 'Modelo parcial de superficies observadas.';
$('reduction').textContent = inputBytes > 0 && modelBytes >= 0 ? `${fmt(100 * (1 - modelBytes / inputBytes), 1)} %` : '—';
const tolerance = meta.coverage_tolerance_m ?? meta.tolerance_m;
if (tolerance != null) $('tolerance').textContent = `Cobertura: mediciones a ≤ ${fmt(1000 * tolerance, 1)} mm de una superficie.`;
for (const warning of [...new Set([...(meta.warnings || []), ...(meta.retention_reasons || [])])]) {
  const e = document.createElement('div');
  e.className = 'note warning';
  e.textContent = String(warning);
  $('warnings').append(e);
}
if (typeof meta.model_filename === 'string' && /^[^/\\:]+\.glb$/i.test(meta.model_filename)) {
  const a = $('download');
  a.href = meta.model_filename;
  a.classList.remove('hidden');
}
const sf = data.structural_frame;
$('frameInfo').textContent = sf ? sf.description || sf.note ||
  (sf.yaw_defined_by_plane === false ? 'Vertical de gravedad; orientación horizontal sin una pared de referencia.' :
    'Referencia local derivada de las superficies. No representa norte geográfico.') :
  'No hay soporte suficiente para una referencia estructural completa.';
const c = $('view'),
  gl = c.getContext('webgl', {
    antialias: true,
    alpha: true,
    preserveDrawingBuffer: true
  });
if (!gl) {
  $('error').textContent = 'Este navegador no pudo iniciar WebGL. Puedes abrir el modelo GLB en Blender u otro visor 3D.';
  $('error').classList.remove('hidden');
  return;
}
const add = (a, b) => a.map((x, i) => x + b[i]),
  sub = (a, b) => a.map((x, i) => x - b[i]),
  mul = (a, k) => a.map(x => x * k),
  dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0),
  cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm = a => Math.hypot(...a),
  unit = a => mul(a, 1 / (norm(a) || 1));
const world = (p, s) => add(s.origin, s.axes.map(row => dot(row, p)));
function shader(type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
  return s;
}
function program(v, f) {
  const p = gl.createProgram();
  gl.attachShader(p, shader(gl.VERTEX_SHADER, v));
  gl.attachShader(p, shader(gl.FRAGMENT_SHADER, f));
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(p));
  return p;
}
const vsrc = 'attribute vec3 p;attribute vec3 n;uniform mat4 vp;varying vec3 normal;void main(){gl_Position=vp*vec4(p,1.0);normal=n;}',
  fsrc = 'precision mediump float;uniform vec3 color;uniform float unlit;varying vec3 normal;void main(){float light=mix(0.50+0.50*abs(dot(normalize(normal+vec3(0.00001)),normalize(vec3(0.4,-0.5,0.8)))),1.0,unlit);gl_FragColor=vec4(color*light,1.0);}';
let prog;
try {
  prog = program(vsrc, fsrc);
} catch (e) {
  $('error').textContent = e.message;
  $('error').classList.remove('hidden');
  return;
}
const loc = {
  p: gl.getAttribLocation(prog, 'p'),
  n: gl.getAttribLocation(prog, 'n'),
  vp: gl.getUniformLocation(prog, 'vp'),
  color: gl.getUniformLocation(prog, 'color'),
  unlit: gl.getUniformLocation(prog, 'unlit')
};
function buffer(a) {
  const b = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, b);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(a), gl.STATIC_DRAW);
  return b;
}
let minimum = [Infinity, Infinity, Infinity],
  maximum = [-Infinity, -Infinity, -Infinity];
const objects = surfaces.map((s, i) => {
  const verts = s.vertices.map(p => world(p, s)),
    positions = [],
    normals = [],
    edges = [];
  for (const p of verts) for (let k = 0; k < 3; k++) {
    minimum[k] = Math.min(minimum[k], p[k]);
    maximum[k] = Math.max(maximum[k], p[k]);
  }
  for (const t of s.triangles) {
    const a = verts[t[0]],
      b = verts[t[1]],
      d = verts[t[2]],
      n = unit(cross(sub(b, a), sub(d, a)));
    positions.push(...a, ...b, ...d);
    normals.push(...n, ...n, ...n);
    edges.push(...a, ...b, ...b, ...d, ...d, ...a);
  }
  return {
    s,
    index: i,
    vertices: verts,
    p: buffer(positions),
    n: buffer(normals),
    edge: buffer(edges),
    count: positions.length / 3,
    edges: edges.length / 3
  };
});
if (!objects.length) {
  $('error').textContent = 'Esta reconstrucción no contiene superficies trianguladas.';
  $('error').classList.remove('hidden');
  return;
}
const center = mul(add(minimum, maximum), 0.5),
  radius = Math.max(norm(sub(maximum, minimum)) / 2, 0.25);
let az = -1.1,
  el = 0.55,
  distance = radius * 2.8,
  selected = 0,
  dirty = false,
  lastVP,
  eye,
  lastRight,
  lastUp;
const selectedButtons = [];
function select(i) {
  selected = i;
  selectedButtons.forEach((b, j) => b.classList.toggle('active', i === j));
  const s = surfaces[i],
    box = $('details');
  box.replaceChildren();
  const title = document.createElement('strong');
  title.textContent = `${s.id} · ${types[s.kind] || s.kind}`;
  box.append(title);
  const rms = s.rmse_m == null ? 'no evaluado' : `${fmt(1000 * s.rmse_m, 2)} mm`;
  for (const text of [`Área triangulada: ${fmt(s.area_m2)} m²`, `Error RMS de ajuste: ${rms}`, `Soporte: ${fmt(s.support_count, 0)} mediciones`, `Origen: [${s.origin.map(n => fmt(n, 3)).join('; ')}] m`]) {
    const e = document.createElement('div');
    e.textContent = text;
    box.append(e);
  }
  if (s.normal && s.kind !== 'freeform') {
    const e = document.createElement('div');
    e.textContent = `Normal: [${s.normal.map(n => fmt(n, 3)).join('; ')}]`;
    box.append(e);
  }
  request();
}
for (const [i, s] of surfaces.entries()) {
  const b = document.createElement('button');
  b.className = 'surface';
  const sw = document.createElement('span');
  sw.className = 'swatch';
  sw.style.background = `rgb(${s.color.map(x => Math.round(x * 255)).join(',')})`;
  const body = document.createElement('span'),
    title = document.createElement('strong'),
    detail = document.createElement('small');
  title.textContent = s.id;
  detail.textContent = `${types[s.kind] || s.kind} · ${fmt(s.area_m2)} m²`;
  body.append(title, detail);
  b.append(sw, body);
  b.onclick = () => select(i);
  $('list').append(b);
  selectedButtons.push(b);
}
