const {test} = require('node:test');
const assert = require('node:assert/strict');
const {buildVisibility, collectStroke} = require('../../tools/structure/annotation_selection.js');

const identity = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
const pointAt = (x, y, z = 0, size = 100) => [x * 2 / size - 1, 1 - y * 2 / size, z];
function build(points, extra = {}) {
  return buildVisibility({points, memberships: points.map(() => 0), visibleGroups: [true], vp: identity,
    width: 100, height: 100, pointSize: 5, ...extra});
}
const stroke = (buffer, from, to = from, radius = 1) => [...collectStroke(buffer, from, to, radius)].sort((a, b) => a - b);
const owners = buffer => [...new Set(buffer.pointIndices)].filter(index => index >= 0).sort((a, b) => a - b);

test('overlapping disks select the nearest measured point, not all points along the ray', () => {
  const points = [[0, 0, 0.7], [0, 0, -0.7], [0, 0, 0]];
  const buffer = build(points);
  assert.deepEqual(stroke(buffer, {x: 50, y: 50}, undefined, 20), [1]);
  assert.ok(Math.abs(buffer.depth[50 * 100 + 50] - 0.15) < 1e-6);
  assert.deepEqual(owners(build([points[1], points[2], points[0]])), [0]);
});

test('hidden layers neither select nor occlude visible points', () => {
  const points = [[0, 0, -0.8], [0, 0, 0.5]];
  const memberships = [0, 1];
  assert.deepEqual(owners(build(points, {memberships, visibleGroups: [false, true]})), [1]);
  assert.deepEqual(owners(build(points, {memberships, visibleGroups: [true, false]})), [0]);
  assert.deepEqual(owners(build(points, {memberships, visibleGroups: [false, false]})), []);
});

test('perspective near/far/behind-eye/side clipping excludes invisible centers', () => {
  // 90-degree perspective, aspect 1, near 1, far 10; camera looks toward -Z.
  const vp = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -11 / 9, -1, 0, 0, -20 / 9, 0]);
  const points = [[0, 0, -2], [0, 0, -0.5], [0, 0, 1], [0, 0, -12], [2.01, 0, -2], [0, -2.01, -2]];
  assert.deepEqual(owners(build(points, {vp, pointSize: 30})), [0]);
  // Even a very large point is clipped if its center lies outside the viewport.
  assert.deepEqual(owners(build([[1.01, 0, 0]], {pointSize: 30})), []);
});

test('circular shader disks leave square corners unselectable', () => {
  const buffer = build([[0, 0, 0]], {pointSize: 10});
  assert.equal(buffer.pointIndices[50 * 100 + 50], 0);
  assert.equal(buffer.pointIndices[45 * 100 + 45], -1);
  assert.deepEqual(stroke(buffer, {x: 45.5, y: 45.5}, undefined, 0), []);
  assert.deepEqual(stroke(buffer, {x: 50.5, y: 50.5}, undefined, 0), [0]);
});

test('equal depth keeps the first drawArrays point and the clear-depth far plane stays empty', () => {
  const buffer = build([[0, 0, 0], [0, 0, 0]]);
  assert.deepEqual(owners(buffer), [0]);
  assert.deepEqual(owners(build([[0, 0, 1]])), []);
});

test('a long fast Pencil stroke picks intermediate points continuously with a bounded radius', () => {
  const points = [10, 30, 50, 70, 90].map(x => pointAt(x, 50));
  points.push(pointAt(50, 58));
  const buffer = build(points, {pointSize: 2});
  const from = {x: 10, y: 50}, to = {x: 90, y: 50};
  assert.deepEqual(stroke(buffer, from, to, 2), [0, 1, 2, 3, 4]);
  assert.deepEqual(stroke(buffer, to, from, 2), [0, 1, 2, 3, 4]);
  assert.deepEqual(stroke(buffer, from, undefined, 2), [0]);
  assert.deepEqual(stroke(buffer, {x: -50, y: -50}, {x: -10, y: -10}, 2), []);
});

test('a partly occluded point can be picked only on its actually visible disk edge', () => {
  const buffer = build([pointAt(49, 50, -0.5), pointAt(52, 50, 0.5)], {pointSize: 8});
  assert.deepEqual(stroke(buffer, {x: 50.5, y: 50.5}, undefined, 0), [0]);
  assert.deepEqual(stroke(buffer, {x: 54.5, y: 50.5}, undefined, 0), [1]);
});

test('canonical point indices survive hidden memberships, invalid coordinates and projection', () => {
  const points = [[0, 0, -0.5], [NaN, 0, 0], pointAt(20, 50), pointAt(80, 50)];
  const buffer = build(points, {memberships: [0, 1, 1, 1], visibleGroups: [false, true]});
  assert.deepEqual(stroke(buffer, {x: 0, y: 50}, {x: 100, y: 50}, 3), [2, 3]);
});

test('backing-store scaling preserves selection and radius across display density', () => {
  const points = [pointAt(20, 50), pointAt(50, 50), pointAt(80, 70)];
  const normal = build(points, {pointSize: 4});
  const retina = build(points, {width: 200, height: 200, pointSize: 8});
  assert.deepEqual(stroke(normal, {x: 10, y: 50}, {x: 70, y: 50}, 2),
    stroke(retina, {x: 20, y: 100}, {x: 140, y: 100}, 4));
});

test('visibility and brush operations are deterministic and do not mutate their inputs', () => {
  const points = [pointAt(40, 40), pointAt(60, 60, -0.2)];
  const memberships = new Int32Array([0, 1]), visibleGroups = [true, true], vp = identity.slice();
  const before = JSON.stringify({points, memberships: [...memberships], visibleGroups, vp: [...vp]});
  const first = build(points, {memberships, visibleGroups, vp});
  const second = build(points, {memberships, visibleGroups, vp});
  assert.deepEqual(first.pointIndices, second.pointIndices);
  assert.deepEqual(first.depth, second.depth);
  const savedIndices = first.pointIndices.slice(), savedDepth = first.depth.slice();
  stroke(first, {x: 0, y: 0}, {x: 100, y: 100}, 8);
  assert.deepEqual(first.pointIndices, savedIndices);
  assert.deepEqual(first.depth, savedDepth);
  assert.equal(JSON.stringify({points, memberships: [...memberships], visibleGroups, vp: [...vp]}), before);
});

test('invalid allocation and coordinate inputs are rejected before rasterization', () => {
  assert.throws(() => build([], {width: 100_000, height: 100_000}), RangeError);
  assert.throws(() => build([], {width: 0}), RangeError);
  assert.throws(() => build([], {pointSize: Infinity}), RangeError);
  assert.throws(() => build([[0, 0, 0]], {memberships: []}), TypeError);
  assert.throws(() => build([], {vp: [1]}), TypeError);
  assert.throws(() => collectStroke(build([]), {x: 1, y: 1}, {x: Infinity, y: 0}, 1), RangeError);
  assert.throws(() => collectStroke(build([]), {x: 1, y: 1}, {x: 1, y: 1}, -1), RangeError);
});
