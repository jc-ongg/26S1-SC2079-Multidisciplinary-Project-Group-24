import test from "node:test";
import assert from "node:assert/strict";
import { rotateDirection, routeSegments, snapshotVisits } from "./simulation.mjs";

test("obstacle direction rotates clockwise through every cardinal direction", () => {
  assert.equal(rotateDirection(0), 2);
  assert.equal(rotateDirection(2), 4);
  assert.equal(rotateDirection(4), 6);
  assert.equal(rotateDirection(6), 0);
});

test("visit order uses SNAP IDs and compressed movement endpoints", () => {
  assert.deepEqual(snapshotVisits(["FW12", "SNAP12_C", "FR01", "BW03", "SNAP2", "FIN"]), [
    { id: 12, step: 1 }, { id: 2, step: 3 },
  ]);
});
test("snapshots at the start and consecutive snapshots retain their endpoint", () => {
  assert.deepEqual(snapshotVisits(["SNAP3", "SNAP4_R", "FW01", "FIN"]), [
    { id: 3, step: 0 }, { id: 4, step: 0 },
  ]);
  assert.deepEqual(snapshotVisits([]), []);
});
test("straight traces connect cell centres and invert the grid Y axis", () => {
  assert.deepEqual(routeSegments([{ x: 1, y: 1, d: 0 }, { x: 1, y: 5, d: 0 }]), ["M 1.5 18.5 L 1.5 14.5"]);
  assert.deepEqual(routeSegments([]), []);
  assert.deepEqual(routeSegments([{ x: 1, y: 1, d: 0 }]), []);
});
test("turn traces join the given endpoints with curves for every cardinal heading", () => {
  for (const d of [0, 2, 4, 6]) {
    const [segment] = routeSegments([{ x: 5, y: 5, d }, { x: 9, y: 7, d: (d + 2) % 8 }]);
    assert.match(segment, /^M 5\.5 14\.5 C /);
    assert.ok(segment.endsWith(", 9.5 12.5"));
  }
});
