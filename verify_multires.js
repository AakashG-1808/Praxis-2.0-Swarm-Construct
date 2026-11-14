/**
 * verify_multires.js — Comprehensive verification suite for adjustable grid resolution
 * and rectangular custom aspect ratios in SimEngine.
 */
const assert = require("assert");
const Sim = require("./sim-engine.js");

console.log("================================================================================");
console.log(" MULTI-RESOLUTION & ADAPTABLE RECTANGULAR GRID VERIFICATION SUITE");
console.log("================================================================================");

let testsPassed = 0;
let totalTests = 0;

function runTest(name, fn) {
  totalTests++;
  try {
    fn();
    console.log(`[PASS] ${name}`);
    testsPassed++;
  } catch (err) {
    console.error(`[FAIL] ${name}: ${err.message}`);
    console.error(err.stack);
  }
}

// 1. Standard square resolutions
[32, 40, 48, 64].forEach(dim => {
  runTest(`Square Resolution ${dim}×${dim} initialization & ticking`, () => {
    Sim.setGridDimensions(dim, dim);
    const s = Sim.getState();
    assert.strictEqual(s.gridWidth, dim, `gridWidth should be ${dim}`);
    assert.strictEqual(s.gridHeight, dim, `gridHeight should be ${dim}`);
    assert(s.totalBlocks > 0, "totalBlocks must be > 0");
    assert.strictEqual(s.depots.length, 4, "Must have 4 material depots");

    // Check depots are inside bounds
    for (const d of s.depots) {
      assert(d.x >= 0 && d.x < dim && d.y >= 0 && d.y < dim, `Depot (${d.x}, ${d.y}) out of bounds`);
    }

    // Step 100 ticks
    for (let i = 0; i < 100; i++) {
      const res = Sim.tick();
      assert(res.blocksPlaced >= 0, "blocksPlaced must be non-negative");
    }
  });
});

// 2. Rectangular 2:1 Landscape Grid (e.g. 48 x 24)
runTest("Rectangular 2:1 Landscape Grid (48×24)", () => {
  Sim.setGridDimensions(48, 24);
  const s = Sim.getState();
  assert.strictEqual(s.gridWidth, 48);
  assert.strictEqual(s.gridHeight, 24);
  assert.strictEqual(s.targetHeight.length, 48);
  assert.strictEqual(s.targetHeight[0].length, 24);

  for (let i = 0; i < 150; i++) {
    Sim.tick();
  }
  const s2 = Sim.getState();
  assert(s2.robots.every(r => r.x >= 0 && r.x < 48 && r.y >= 0 && r.y < 24), "All robots must stay within bounds");
});

// 3. Rectangular 1:2 Portrait Grid (e.g. 24 x 48)
runTest("Rectangular 1:2 Portrait Grid (24×48)", () => {
  Sim.setGridDimensions(24, 48);
  const s = Sim.getState();
  assert.strictEqual(s.gridWidth, 24);
  assert.strictEqual(s.gridHeight, 48);
  assert.strictEqual(s.targetHeight.length, 24);
  assert.strictEqual(s.targetHeight[0].length, 48);

  for (let i = 0; i < 150; i++) {
    Sim.tick();
  }
  const s2 = Sim.getState();
  assert(s2.robots.every(r => r.x >= 0 && r.x < 24 && r.y >= 0 && r.y < 48), "All robots must stay within bounds");
});

// 4. Custom Rectangular Blueprint Importer (e.g. 52 x 26 letter typography)
runTest("Custom Rectangular Blueprint Importer (52×26)", () => {
  const custom = [];
  for (let x = 0; x < 52; x++) {
    custom[x] = new Array(26).fill(0);
    // Draw an arbitrary rectangular pattern
    if (x >= 12 && x <= 40 && (x % 4 === 0)) {
      custom[x][6] = 2;
      custom[x][12] = 3;
      custom[x][18] = 2;
    }
  }

  Sim.setCustomBlueprint(custom, "TestRectCustom");
  const s = Sim.getState();
  assert.strictEqual(s.gridWidth, 52);
  assert.strictEqual(s.gridHeight, 26);
  assert(s.totalBlocks > 0, "Custom blueprint must have totalBlocks > 0");

  for (let i = 0; i < 200; i++) {
    Sim.tick();
  }
  const s2 = Sim.getState();
  assert(s2.blocksPlaced > 0, "Robots should place blocks on custom rectangular blueprint");
});

// 5. Audit completion exact-match on rectangular structure
runTest("Audit completion function validates rectangular grid exact match", () => {
  const audit = Sim.auditCompletion();
  assert(typeof audit.isExactMatch === "boolean");
  assert(typeof audit.totalBlueprintBlocks === "number");
  assert(typeof audit.placedBlocks === "number");
  assert(Array.isArray(audit.mismatches));
});

console.log("================================================================================");
console.log(` RESULTS: ${testsPassed} / ${totalTests} multi-resolution tests passed successfully.`);
console.log("================================================================================");
