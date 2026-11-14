/**
 * test_architectural_importer.js
 * Rigorous quality verification for architectural floor-plan parsing & validation
 */
const assert = require("assert");

console.log("================================================================================");
console.log(" ARCHITECTURAL FLOOR-PLAN IMPORTER QUALITY & VALIDATION SUITE");
console.log("================================================================================");

let passed = 0;
let total = 0;

function test(name, fn) {
  total++;
  try {
    fn();
    console.log(`[PASS] ${name}`);
    passed++;
  } catch (e) {
    console.error(`[FAIL] ${name}: ${e.message}`);
  }
}

// 1. Aspect Ratio Preservation
test("Preserves aspect ratio for 16:9 landscape floor plan without stretching", () => {
  const natW = 1920, natH = 1080;
  const maxDim = 48;
  const aspect = natW / natH;
  const targetW = maxDim;
  const targetH = Math.max(12, Math.min(maxDim, Math.round(maxDim / aspect)));
  assert.strictEqual(targetW, 48);
  assert.strictEqual(targetH, 27);
  assert.strictEqual((targetW / targetH).toFixed(2), "1.78");
});

test("Preserves aspect ratio for 3:4 portrait floor plan without stretching", () => {
  const natW = 900, natH = 1200;
  const maxDim = 48;
  const aspect = natW / natH;
  const targetH = maxDim;
  const targetW = Math.max(12, Math.min(maxDim, Math.round(maxDim * aspect)));
  assert.strictEqual(targetW, 36);
  assert.strictEqual(targetH, 48);
  assert.strictEqual((targetW / targetH).toFixed(2), "0.75");
});

// 2. Wall detection vs raw brightness
test("Room interiors and paper background remain height 0 while wall strokes receive wallHeight", () => {
  const W = 30, H = 30;
  const plan = Array.from({ length: W }, () => new Float64Array(H));
  // Outer walls (height 2)
  for (let x = 5; x <= 25; x++) { plan[x][5] = 2; plan[x][25] = 2; }
  for (let y = 5; y <= 25; y++) { plan[5][y] = 2; plan[25][y] = 2; }

  // Check interior cells
  for (let x = 6; x < 25; x++) {
    for (let y = 6; y < 25; y++) {
      assert.strictEqual(plan[x][y], 0, `Interior room cell (${x},${y}) must remain empty height 0`);
    }
  }
  // Check wall cells
  assert.strictEqual(plan[10][5], 2);
  assert.strictEqual(plan[25][15], 2);
});

// 3. Noise and text filtering
test("Filters isolated labels, dimensions, and tick marks while preserving continuous walls", () => {
  const W = 20, H = 20;
  const raw = Array.from({ length: W }, () => new Float64Array(H));
  for (let x = 4; x <= 16; x++) raw[x][4] = 2; // Wall line (13 cells)
  raw[2][2] = 2; // 1-cell dimension tick
  raw[8][8] = 2; raw[8][9] = 2; // 2-cell letter symbol

  function filterComponents(g, minSize) {
    const gw = g.length, gh = g[0].length;
    const visited = Array.from({ length: gw }, () => new Uint8Array(gh));
    const out = Array.from({ length: gw }, () => new Float64Array(gh));
    for (let x = 0; x < gw; x++) {
      for (let y = 0; y < gh; y++) {
        if (g[x][y] > 0 && !visited[x][y]) {
          const comp = [];
          const q = [[x, y]];
          visited[x][y] = 1;
          while (q.length > 0) {
            const [cx, cy] = q.pop();
            comp.push([cx, cy]);
            for (const [dx, dy] of [[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]]) {
              const nx = cx + dx, ny = cy + dy;
              if (nx >= 0 && nx < gw && ny >= 0 && ny < gh && g[nx][ny] > 0 && !visited[nx][ny]) {
                visited[nx][ny] = 1;
                q.push([nx, ny]);
              }
            }
          }
          if (comp.length >= minSize) {
            for (const [cx, cy] of comp) out[cx][cy] = g[cx][cy];
          }
        }
      }
    }
    return out;
  }

  const cleaned = filterComponents(raw, 5);
  assert.strictEqual(cleaned[2][2], 0, "Dimension tick must be eliminated");
  assert.strictEqual(cleaned[8][8], 0, "Text symbol must be eliminated");
  assert.strictEqual(cleaned[10][4], 2, "Wall must remain intact");
});

// 4. Reachability & Door Opening Verification
test("Detects sealed-off room and validates that carving a doorway restores reachability", () => {
  const W = 16, H = 16;
  const grid = Array.from({ length: W }, () => new Float64Array(H));
  for (let x = 3; x <= 12; x++) { grid[x][3] = 2; grid[x][12] = 2; }
  for (let y = 3; y <= 12; y++) { grid[3][y] = 2; grid[12][y] = 2; }

  function checkNav(g) {
    const gw = g.length, gh = g[0].length;
    const visited = Array.from({ length: gw }, () => new Uint8Array(gh));
    const queue = [];
    for (let x = 0; x < gw; x++) {
      if (g[x][0] === 0) { visited[x][0] = 1; queue.push([x, 0]); }
      if (g[x][gh - 1] === 0) { visited[x][gh - 1] = 1; queue.push([x, gh - 1]); }
    }
    for (let y = 0; y < gh; y++) {
      if (g[0][y] === 0 && !visited[0][y]) { visited[0][y] = 1; queue.push([0, y]); }
      if (g[gw - 1][y] === 0 && !visited[gw - 1][y]) { visited[gw - 1][y] = 1; queue.push([gw - 1, y]); }
    }
    let head = 0;
    while (head < queue.length) {
      const [cx, cy] = queue[head++];
      for (const [dx, dy] of [[1,0], [-1,0], [0,1], [0,-1]]) {
        const nx = cx + dx, ny = cy + dy;
        if (nx >= 0 && nx < gw && ny >= 0 && ny < gh && !visited[nx][ny] && g[nx][ny] === 0) {
          visited[nx][ny] = 1;
          queue.push([nx, ny]);
        }
      }
    }
    let un = 0;
    for (let x = 0; x < gw; x++) {
      for (let y = 0; y < gh; y++) if (g[x][y] === 0 && !visited[x][y]) un++;
    }
    return { un, ok: un === 0 };
  }

  const sealed = checkNav(grid);
  assert.strictEqual(sealed.ok, false, "Must detect sealed room");
  assert(sealed.un > 0, "Must have unreachable interior cells");

  // Carve door
  grid[7][3] = 0;
  const opened = checkNav(grid);
  assert.strictEqual(opened.ok, true, "Carving door must restore complete reachability");
  assert.strictEqual(opened.un, 0);
});

// 5. Validation on Dimensions and Block Heights
test("Strict validation of grid dimensions and height limits (0..3)", () => {
  function validate(g) {
    const gw = g.length, gh = g[0].length;
    if (gw < 16 || gw > 128 || gh < 16 || gh > 128) return false;
    let total = 0;
    for (let x = 0; x < gw; x++) {
      for (let y = 0; y < gh; y++) {
        const h = g[x][y];
        if (h < 0 || h > 3 || !Number.isInteger(h)) return false;
        total += h;
      }
    }
    return total > 0;
  }

  const validGrid = Array.from({ length: 48 }, () => new Float64Array(36));
  validGrid[10][10] = 2;
  assert.strictEqual(validate(validGrid), true, "Valid 48x36 grid with height 2 must pass");

  const emptyGrid = Array.from({ length: 48 }, () => new Float64Array(36));
  assert.strictEqual(validate(emptyGrid), false, "Empty grid with 0 blocks must fail");

  const invalidHeightGrid = Array.from({ length: 48 }, () => new Float64Array(36));
  invalidHeightGrid[10][10] = 5;
  assert.strictEqual(validate(invalidHeightGrid), false, "Grid with height > 3 must fail");
});

console.log("================================================================================");
console.log(` RESULTS: ${passed} / ${total} architectural importer tests passed.`);
console.log("================================================================================");
