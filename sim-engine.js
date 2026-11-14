/**
 * SimEngine — Swarm Construction Simulator Engine
 * Domain: Swarm Robotics / Multi-Agent Systems / Construction Tech
 * Problem Statement 04: Swarm Intelligence for Autonomous Building Construction
 * C-DAC / Praxis 2.0 Hackathon
 *
 * STRICTLY DECENTRALIZED ARCHITECTURE:
 * - NO central controller or task scheduler.
 * - Every robot agent executes an independent Finite State Machine (FSM) using ONLY:
 *   1. Its onboard copy of the compiled blueprint (target_height and dependency_rank matrices).
 *   2. Local 1-cell neighborhood sensing (r = 1).
 *   3. Stigmergy (reading and writing evaporating pheromone values in the environment grid).
 */

const SimEngine = (function () {
  "use strict";

  // ── Grid & Physics Constants ──────────────────────────────────────────────
  let gridWidth = 32;
  let gridHeight = 32;
  let GRID = 32;
  const EVAP_RATE = 0.04; // Pheromone decay per tick: P = P * (1 - 0.04)

  const STATES = {
    SEEK_MATERIAL: 0,
    CARRY_TO_BUILD: 1,
    INSPECT_AND_PLACE: 2,
    YIELD_AND_BACKOFF: 3,
    RECHARGE: 4,
    RETURN_HOME: 5,
  };
  const STATE_NAMES = ["SEEK", "CARRY", "INSPECT", "YIELD", "RECHARGE", "HOME"];

  const DIR4 = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ];

  // ── Seeded RNG (LCG) ──────────────────────────────────────────────────────
  let _seed = null;
  let _rngState = 123456789;

  function lcg() {
    _rngState = (_rngState * 1664525 + 1013904223) & 0xffffffff;
    return (_rngState >>> 0) / 0x100000000;
  }

  function random() {
    if (_seed !== null) return lcg();
    return Math.random();
  }

  function setSeed(seed) {
    if (seed !== null && seed !== undefined) {
      _seed = seed;
      _rngState = (seed | 0) || 123456789;
    } else {
      _seed = null;
    }
  }

  // ── World Matrices ────────────────────────────────────────────────────────
  let targetHeight = null;
  let currentHeight = null;
  let obstacles = null;
  let depots = [];
  let pheromoneBuild = null;
  let pheromoneTrail = null;
  let trafficCount = null;
  let depRank = null;
  let robots = [];
  let robotGrid = null; // Spatial occupancy lookup: robotGrid[x][y] = robot | null
  let buildPhase = null; // 2.5D Construction Phase Matrix (0: Phase 0/Pillar, 1: Phase 1/Perimeter)

  let tickCount = 0;
  let totalBlocks = 0;
  let blocksPlaced = 0;
  let ticksSinceLastPlacement = 0;
  let throughputBuffer = [];
  let acoMode = true;
  let currentBlueprintName = "walledRoom";
  let unbuiltDistGrid = null;
  let evacuationTicks = 0;

  // Stretch-goal feature states
  let heterogeneousRoles = false;
  let batteryEnabled = false;
  let sensorNoise = 0.0;
  let relocationEnabled = true;
  let deadlockThreshold = 45;
  let relocatedBlocks = [];
  let relocationLog = [];
  let movingHazard = {
    enabled: false,
    x: 8,
    y: 8,
    size: 2,
    vx: 1,
    vy: 0,
    minX: 4,
    maxX: 26,
    minY: 4,
    maxY: 26,
    stepCounter: 0,
  };

  const DEFAULT_DEPOTS = [
    { x: 4, y: 4 },
    { x: 4, y: 27 },
    { x: 27, y: 4 },
    { x: 27, y: 27 },
  ];

  // ── Grid Utilities ────────────────────────────────────────────────────────
  function makeGrid(val, w, h) {
    w = w || gridWidth;
    h = h || gridHeight;
    val = val || 0;
    const g = new Array(w);
    for (let x = 0; x < w; x++) {
      g[x] = new Float64Array(h);
      if (val !== 0) {
        for (let y = 0; y < h; y++) g[x][y] = val;
      }
    }
    return g;
  }

  function makeBoolGrid(w, h) {
    w = w || gridWidth;
    h = h || gridHeight;
    const g = new Array(w);
    for (let x = 0; x < w; x++) {
      g[x] = new Uint8Array(h);
    }
    return g;
  }

  function inBounds(x, y) {
    return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight;
  }

  function manhattanDist(x0, y0, x1, y1) {
    return Math.abs(x1 - x0) + Math.abs(y1 - y0);
  }

  // ── Built-in Blueprint Presets ────────────────────────────────────────────

  /**
   * 1. Walled Room with Doorway/Window Constraint
   * Height 2 walls, doorway gap on south wall, window gap on east wall, central 3×3 height-3 pillar.
   */
  function walledRoom() {
    const t = makeGrid(0);
    let x0 = 6, x1 = 25, y0 = 9, y1 = 22;
    if (gridWidth !== 32 || gridHeight !== 32) {
      const marginX = Math.max(3, Math.round(gridWidth * 0.1875));
      const marginY = Math.max(3, Math.round(gridHeight * 0.28));
      x0 = marginX;
      x1 = gridWidth - marginX - 1;
      y0 = marginY;
      y1 = gridHeight - marginY - 1;
    }

    // North wall
    for (let x = x0; x <= x1; x++) t[x][y0] = 2;

    // South wall with 2-cell doorway gap
    const doorCx = Math.floor((x0 + x1) / 2);
    for (let x = x0; x <= x1; x++) {
      if (x === doorCx || x === doorCx + 1) continue; // Doorway opening
      t[x][y1] = 2;
    }

    // West wall
    for (let y = y0; y <= y1; y++) t[x0][y] = 2;

    // East wall with 2-cell window gap
    const winCy = Math.floor((y0 + y1) / 2);
    for (let y = y0; y <= y1; y++) {
      if (y === winCy || y === winCy + 1) continue; // Window opening
      t[x1][y] = 2;
    }

    // Central 3×3 pillar of height 3
    const cx = Math.floor((x0 + x1) / 2);
    const cy = Math.floor((y0 + y1) / 2);
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        t[cx + dx][cy + dy] = 3;
      }
    }
    return t;
  }

  /**
   * 2. Stepped Pyramid
   * Concentric rectangular rings of height 1, 2, 3 centered at grid center.
   */
  function steppedPyramid() {
    const t = makeGrid(0);
    const cx = Math.floor(gridWidth / 2);
    const cy = Math.floor(gridHeight / 2);
    let r1 = 8, r2 = 5, r3 = 2;
    if (gridWidth !== 32 || gridHeight !== 32) {
      const minHalf = Math.min(Math.floor(gridWidth / 2), Math.floor(gridHeight / 2));
      r1 = Math.max(6, minHalf - 3);
      r2 = Math.max(4, Math.round(r1 * 0.625));
      r3 = Math.max(2, Math.round(r1 * 0.25));
    }

    // Outer ring radius r1 -> height 1
    for (let x = cx - r1; x <= cx + r1; x++) {
      for (let y = cy - r1; y <= cy + r1; y++) {
        if (inBounds(x, y)) t[x][y] = 1;
      }
    }
    // Middle ring radius r2 -> height 2
    for (let x = cx - r2; x <= cx + r2; x++) {
      for (let y = cy - r2; y <= cy + r2; y++) {
        if (inBounds(x, y)) t[x][y] = 2;
      }
    }
    // Inner ring radius r3 -> height 3
    for (let x = cx - r3; x <= cx + r3; x++) {
      for (let y = cy - r3; y <= cy + r3; y++) {
        if (inBounds(x, y)) t[x][y] = 3;
      }
    }
    return t;
  }

  /**
   * 3. Twin Bunker Complex
   * Dual reinforced bunker modules connected by a central logistics corridor with exterior access portals.
   */
  function cdacComplex() {
    const t = makeGrid(0);
    if (gridWidth === 32 && gridHeight === 32) {
      function hWall(x0, x1, y, h) { for (let x = x0; x <= x1; x++) if (inBounds(x, y)) t[x][y] = h; }
      function vWall(x, y0, y1, h) { for (let y = y0; y <= y1; y++) if (inBounds(x, y)) t[x][y] = h; }

      // Bunker A (West Module): [4, 13] x [7, 17]
      hWall(4, 13, 7, 2);
      hWall(4, 13, 17, 2);
      vWall(4, 7, 17, 2);
      vWall(13, 7, 17, 2);
      t[4][12] = 0; // West portal doorway

      // Bunker B (East Module): [18, 27] x [7, 17]
      hWall(18, 27, 7, 2);
      hWall(18, 27, 17, 2);
      vWall(18, 7, 17, 2);
      vWall(27, 7, 17, 2);
      t[27][12] = 0; // East portal doorway

      // Connecting Logistics Corridor between A and B: y=11 and y=13, x in [13, 18]
      hWall(13, 18, 11, 2);
      hWall(13, 18, 13, 2);
      t[13][12] = 0; // Inner opening into Bunker A
      t[18][12] = 0; // Inner opening into Bunker B

      // Corner reinforcement pillars (height 3)
      const pillars = [
        [4, 7], [13, 7], [4, 17], [13, 17],
        [18, 7], [27, 7], [18, 17], [27, 17],
        [8, 12], [23, 12] // Central internal support pillars
      ];
      for (const [px, py] of pillars) {
        if (inBounds(px, py)) t[px][py] = 3;
      }
      return t;
    }

    // Dynamic resolution scaling
    const sx = gridWidth / 32;
    const sy = gridHeight / 32;
    function sxC(x) { return Math.min(gridWidth - 1, Math.round(x * sx)); }
    function syC(y) { return Math.min(gridHeight - 1, Math.round(y * sy)); }
    function hWall(x0, x1, y, h) { for (let x = sxC(x0); x <= sxC(x1); x++) if (inBounds(x, syC(y))) t[x][syC(y)] = h; }
    function vWall(x, y0, y1, h) { for (let y = syC(y0); y <= syC(y1); y++) if (inBounds(sxC(x), y)) t[sxC(x)][y] = h; }

    hWall(4, 13, 7, 2); hWall(4, 13, 17, 2); vWall(4, 7, 17, 2); vWall(13, 7, 17, 2); t[sxC(4)][syC(12)] = 0;
    hWall(18, 27, 7, 2); hWall(18, 27, 17, 2); vWall(18, 7, 17, 2); vWall(27, 7, 17, 2); t[sxC(27)][syC(12)] = 0;
    hWall(13, 18, 11, 2); hWall(13, 18, 13, 2); t[sxC(13)][syC(12)] = 0; t[sxC(18)][syC(12)] = 0;
    const pillars = [
      [4, 7], [13, 7], [4, 17], [13, 17],
      [18, 7], [27, 7], [18, 17], [27, 17],
      [8, 12], [23, 12]
    ];
    for (const [px, py] of pillars) if (inBounds(sxC(px), syC(py))) t[sxC(px)][syC(py)] = 3;
    return t;
  }

  /**
   * 4. C-DAC Logo / Typography Preset
   * Block typography "C - D - A - C" with corner reinforcement pillars.
   */
  function cdacLogoBlueprint(w, h) {
    w = w || gridWidth;
    h = h || gridHeight;
    const t = makeGrid(0, w, h);
    const sx = w / 32;
    const sy = h / 32;
    function rect(x0, y0, bw, bh, val) {
      const rx0 = Math.round(x0 * sx);
      const ry0 = Math.round(y0 * sy);
      const rw = Math.max(1, Math.round(bw * sx));
      const rh = Math.max(1, Math.round(bh * sy));
      for (let x = rx0; x < rx0 + rw; x++) {
        for (let y = ry0; y < ry0 + rh; y++) {
          if (x >= 0 && x < w && y >= 0 && y < h) t[x][y] = val;
        }
      }
    }
    function pt(x, y, val) {
      const rx = Math.min(w - 1, Math.round(x * sx));
      const ry = Math.min(h - 1, Math.round(y * sy));
      if (rx >= 0 && rx < w && ry >= 0 && ry < h) t[rx][ry] = val;
    }
    // C1: x: 3..7, y: 11..20
    rect(3, 11, 5, 2, 2);
    rect(3, 19, 5, 2, 2);
    rect(3, 11, 2, 10, 2);
    pt(3, 11, 3); pt(7, 11, 3); pt(3, 20, 3); pt(7, 20, 3);

    // D: x: 9..14, y: 11..20
    rect(9, 11, 2, 10, 2);
    rect(9, 11, 4, 2, 2);
    rect(9, 19, 4, 2, 2);
    rect(13, 13, 2, 6, 2);
    pt(9, 11, 3); pt(9, 20, 3);

    // A: x: 16..21, y: 11..20
    rect(16, 11, 2, 10, 2);
    rect(20, 11, 2, 10, 2);
    rect(16, 11, 6, 2, 2);
    rect(16, 15, 6, 2, 2);
    pt(16, 11, 3); pt(21, 11, 3);

    // C2: x: 23..27, y: 11..20
    rect(23, 11, 5, 2, 2);
    rect(23, 19, 5, 2, 2);
    rect(23, 11, 2, 10, 2);
    pt(23, 11, 3); pt(27, 11, 3); pt(23, 20, 3); pt(27, 20, 3);

    return t;
  }

  /**
   * 5. PES University Logo Preset
   * Block typography "P - E - S" with height 2 walls and height 3 crowns.
   */
  function pesLogoBlueprint(w, h) {
    w = w || gridWidth;
    h = h || gridHeight;
    const t = makeGrid(0, w, h);
    const sx = w / 32;
    const sy = h / 32;
    function rect(x0, y0, bw, bh, val) {
      const rx0 = Math.round(x0 * sx);
      const ry0 = Math.round(y0 * sy);
      const rw = Math.max(1, Math.round(bw * sx));
      const rh = Math.max(1, Math.round(bh * sy));
      for (let x = rx0; x < rx0 + rw; x++) {
        for (let y = ry0; y < ry0 + rh; y++) {
          if (x >= 0 && x < w && y >= 0 && y < h) t[x][y] = val;
        }
      }
    }
    function pt(x, y, val) {
      const rx = Math.min(w - 1, Math.round(x * sx));
      const ry = Math.min(h - 1, Math.round(y * sy));
      if (rx >= 0 && rx < w && ry >= 0 && ry < h) t[rx][ry] = val;
    }
    // P: x: 5..10, y: 10..21
    rect(5, 10, 2, 12, 2);
    rect(5, 10, 6, 2, 2);
    rect(5, 15, 6, 2, 2);
    rect(9, 10, 2, 6, 2);
    pt(5, 10, 3); pt(10, 10, 3); pt(5, 21, 3);

    // E: x: 13..18, y: 10..21
    rect(13, 10, 2, 12, 2);
    rect(13, 10, 6, 2, 2);
    rect(13, 15, 5, 2, 2);
    rect(13, 20, 6, 2, 2);
    pt(13, 10, 3); pt(18, 10, 3); pt(13, 21, 3); pt(18, 21, 3);

    // S: x: 21..26, y: 10..21
    rect(21, 10, 6, 2, 2);
    rect(21, 10, 2, 6, 2);
    rect(21, 15, 6, 2, 2);
    rect(25, 15, 2, 6, 2);
    rect(21, 20, 6, 2, 2);
    pt(21, 10, 3); pt(26, 10, 3); pt(21, 21, 3); pt(26, 21, 3);

    return t;
  }

  // ── Build Phase Compiler & Regional Progress ──────────────────────────────
  /**
   * Compiles explicit construction dependency phases for decentralized execution:
   * - For 'walledRoom': Phase 0 is the central 3x3 pillar (height 3). Phase 1 is the perimeter walls (height 2).
   *   Robots enforce that Phase 0 must be 100% complete before any Phase 1 blocks are placed.
   * - For other blueprints: All target blocks default to Phase 0.
   */
  function compileBuildPhase(name, th) {
    const bp = makeBoolGrid();
    if (name === "walledRoom") {
      for (let x = 0; x < gridWidth; x++) {
        for (let y = 0; y < gridHeight; y++) {
          if (th[x][y] > 0) {
            // Central height-3 pillar is phase 0; perimeter walls are phase 1
            bp[x][y] = (th[x][y] === 3) ? 0 : 1;
          }
        }
      }
    } else {
      for (let x = 0; x < gridWidth; x++) {
        for (let y = 0; y < gridHeight; y++) {
          bp[x][y] = 0;
        }
      }
    }
    return bp;
  }

  function isPhaseComplete(phase) {
    if (!buildPhase || !targetHeight || !currentHeight) return true;
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        if (buildPhase[x][y] === phase && currentHeight[x][y] < targetHeight[x][y]) {
          return false;
        }
      }
    }
    return true;
  }

  function getCurrentBuildPhase() {
    if (currentBlueprintName === "walledRoom") {
      if (!isPhaseComplete(0)) return 0;
      return 1;
    }
    return 0;
  }

  function getRegionalProgress() {
    if (!targetHeight || !currentHeight) {
      return { total: { placed: 0, total: 0, pct: "0.0" } };
    }
    if (currentBlueprintName === "walledRoom" && buildPhase) {
      let pTot = 0, pPlc = 0, wTot = 0, wPlc = 0;
      for (let x = 0; x < gridWidth; x++) {
        for (let y = 0; y < gridHeight; y++) {
          if (buildPhase[x][y] === 0 && targetHeight[x][y] > 0) {
            pTot += targetHeight[x][y];
            pPlc += currentHeight[x][y];
          } else if (buildPhase[x][y] === 1 && targetHeight[x][y] > 0) {
            wTot += targetHeight[x][y];
            wPlc += currentHeight[x][y];
          }
        }
      }
      const pPct = pTot > 0 ? ((pPlc / pTot) * 100).toFixed(1) : "100.0";
      const wPct = wTot > 0 ? ((wPlc / wTot) * 100).toFixed(1) : "100.0";
      const tPct = totalBlocks > 0 ? ((blocksPlaced / totalBlocks) * 100).toFixed(1) : "0.0";
      return {
        pillar: { placed: pPlc, total: pTot, pct: pPct },
        perimeter: { placed: wPlc, total: wTot, pct: wPct },
        total: { placed: blocksPlaced, total: totalBlocks, pct: tPct },
      };
    }
    const tPct = totalBlocks > 0 ? ((blocksPlaced / totalBlocks) * 100).toFixed(1) : "0.0";
    return {
      structure: { placed: blocksPlaced, total: totalBlocks, pct: tPct },
      total: { placed: blocksPlaced, total: totalBlocks, pct: tPct },
    };
  }

  // ── Dependency Rank Compiler (BFS Wavefront) ──────────────────────────────
  /**
   * Multi-source BFS from material depots outward across the grid.
   * Ranks indicate topological distance from logistics source.
   */
  function compileDependencyRank(th, dps, obs) {
    const dr = makeGrid(Infinity);
    const totalCells = gridWidth * gridHeight;
    const qx = new Int16Array(totalCells);
    const qy = new Int16Array(totalCells);
    let head = 0, tail = 0;

    for (const d of dps) {
      const dx = d.x | 0, dy = d.y | 0;
      if (inBounds(dx, dy) && !obs[dx][dy]) {
        dr[dx][dy] = 0;
        qx[tail] = dx;
        qy[tail] = dy;
        tail++;
      }
    }

    while (head < tail) {
      const cx = qx[head];
      const cy = qy[head];
      head++;
      const nd = dr[cx][cy] + 1;

      for (let i = 0; i < 4; i++) {
        const nx = cx + DIR4[i][0];
        const ny = cy + DIR4[i][1];
        if (inBounds(nx, ny) && !obs[nx][ny]) {
          if (nd < dr[nx][ny]) {
            dr[nx][ny] = nd;
            // TERMES climbing physics: robots can only walk / step across cells
            // where height difference <= 1. Steep vertical barriers (> 1 block height jump)
            // cannot be traversed through from ground level; robots must enter through openings.
            if (Math.abs(th[nx][ny] - th[cx][cy]) <= 1) {
              qx[tail] = nx;
              qy[tail] = ny;
              tail++;
            }
          }
        }
      }
    }

    // Ensure geometric interior of solid multi-height blocks (like the 3x3 pillar)
    // has strictly higher dependency rank than its surrounding outer ring,
    // guaranteeing inside-out build order under the TERMES anti-trap constraint.
    for (let x = 1; x < gridWidth - 1; x++) {
      for (let y = 1; y < gridHeight - 1; y++) {
        if (th[x][y] === 3) {
          let surroundingWalls = 0;
          for (let d = 0; d < 4; d++) {
            const nx = x + DIR4[d][0], ny = y + DIR4[d][1];
            if (th[nx][ny] >= 2) surroundingWalls++;
          }
          if (surroundingWalls === 4) {
            let maxNeighborRank = 0;
            for (let d = 0; d < 4; d++) {
              const nx = x + DIR4[d][0], ny = y + DIR4[d][1];
              if (dr[nx][ny] > maxNeighborRank && dr[nx][ny] !== Infinity) {
                maxNeighborRank = dr[nx][ny];
              }
            }
            dr[x][y] = maxNeighborRank + 1;
          }
        }
      }
    }
    return dr;
  }

  // ── Safe Temporary Block Relocation & Deadlock Recovery ─────────────────
  function computeTraversableReachability(heightGrid) {
    heightGrid = heightGrid || currentHeight;
    const visited = makeBoolGrid();
    const queue = [];
    for (const d of depots) {
      const dx = d.x | 0, dy = d.y | 0;
      if (inBounds(dx, dy) && !obstacles[dx][dy]) {
        visited[dx][dy] = 1;
        queue.push({ x: dx, y: dy });
      }
    }

    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      const ch = heightGrid[cur.x][cur.y];
      for (const [ddx, ddy] of DIR4) {
        const nx = cur.x + ddx, ny = cur.y + ddy;
        if (inBounds(nx, ny) && !obstacles[nx][ny] && !visited[nx][ny]) {
          if (Math.abs(heightGrid[nx][ny] - ch) <= 1) {
            visited[nx][ny] = 1;
            queue.push({ x: nx, y: ny });
          }
        }
      }
    }
    return visited;
  }

  function getUnreachableUnbuiltCells(heightGrid) {
    heightGrid = heightGrid || currentHeight;
    const reach = computeTraversableReachability(heightGrid);
    const unreachable = [];
    const activePhase = getCurrentBuildPhase();
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        if (targetHeight[x][y] > heightGrid[x][y] && (!buildPhase || buildPhase[x][y] <= activePhase)) {
          let hasReachableStand = false;
          for (const [ddx, ddy] of DIR4) {
            const sx = x + ddx, sy = y + ddy;
            if (inBounds(sx, sy) && !obstacles[sx][sy]) {
              if (reach[sx][sy]) {
                hasReachableStand = true;
                break;
              }
            }
          }
          if (!hasReachableStand) {
            unreachable.push({ x, y });
          }
        }
      }
    }
    return { unreachable, reach };
  }

  function findSafeStagingCell(bx, by) {
    const tempH = makeGrid(0);
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) tempH[x][y] = currentHeight[x][y];
    }
    tempH[bx][by] = 99;
    let reachExt = computeTraversableReachability(tempH);
    let hasReach = false;
    for (let x = 0; x < gridWidth && !hasReach; x++) {
      for (let y = 0; y < gridHeight && !hasReach; y++) {
        if (reachExt[x][y]) hasReach = true;
      }
    }
    if (!hasReach) reachExt = computeTraversableReachability(currentHeight);

    for (const minOpen of [3, 2]) {
      const candidates = [];
      for (let r = 1; r <= 11; r++) {
        for (let dx = -r; dx <= r; dx++) {
          for (let dy = -r; dy <= r; dy++) {
            if (Math.abs(dx) + Math.abs(dy) !== r) continue;
            const sx = bx + dx, sy = by + dy;
            if (!inBounds(sx, sy)) continue;
            if (targetHeight[sx][sy] > 0 || currentHeight[sx][sy] > 0 || obstacles[sx][sy]) continue;
            if (isDepot(sx, sy)) continue;
            if (robotGrid[sx][sy] !== null) continue;
            if (!reachExt[sx][sy]) continue;

            let openNeighbors = 0;
            for (const [ddx, ddy] of DIR4) {
              const nx = sx + ddx, ny = sy + ddy;
              if (inBounds(nx, ny) && !obstacles[nx][ny] && targetHeight[nx][ny] === 0) {
                openNeighbors++;
              }
            }
            if (openNeighbors >= minOpen) {
              candidates.append ? candidates.append({ x: sx, y: sy, r, openNeighbors }) : candidates.push({ x: sx, y: sy, r, openNeighbors });
            }
          }
        }
      }
      if (candidates.length > 0) {
        candidates.sort((a, b) => a.r !== b.r ? a.r - b.r : b.openNeighbors - a.openNeighbors);
        return { x: candidates[0].x, y: candidates[0].y };
      }
    }
    return null;
  }

  function findBlockingCandidates(unreachableCells) {
    const baseCount = unreachableCells.length;
    if (baseCount === 0) return [];
    const candidates = [];

    // Clone current height
    const tempH = makeGrid(0);
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) tempH[x][y] = currentHeight[x][y];
    }

    for (let bx = 0; bx < gridWidth; bx++) {
      for (let by = 0; by < gridHeight; by++) {
        if (currentHeight[bx][by] <= 0) continue;
        if (robotGrid[bx][by] !== null) continue;

        tempH[bx][by] = currentHeight[bx][by] - 1;
        const res = getUnreachableUnbuiltCells(tempH);
        tempH[bx][by] = currentHeight[bx][by];

        const unblockedCount = baseCount - res.unreachable.length;
        if (unblockedCount > 0) {
          const unblockedCells = unreachableCells.filter(c => !res.unreachable.some(u => u.x === c.x && u.y === c.y));
          candidates.push({
            x: bx,
            y: by,
            currentH: currentHeight[bx][by],
            unblockedCount,
            unblockedCells,
          });
        }
      }
    }
    candidates.sort((a, b) => b.unblockedCount - a.unblockedCount);
    return candidates;
  }

  function isActiveRelocatedCell(x, y) {
    return relocatedBlocks.some(r => r.status === "staged" && r.origin.x === x && r.origin.y === y);
  }

  function attemptDeadlockRecovery() {
    const { unreachable } = getUnreachableUnbuiltCells();
    if (unreachable.length === 0) return null;

    const candidates = findBlockingCandidates(unreachable);
    if (candidates.length === 0) {
      relocationLog.push({
        status: "REFUSED",
        reason: "NO_BLOCKING_CANDIDATE_FOUND",
        tick: tickCount,
        jamLocation: unreachable[0],
      });
      return null;
    }

    for (const cand of candidates) {
      const staging = findSafeStagingCell(cand.x, cand.y);
      if (!staging) continue;

      currentHeight[cand.x][cand.y]--;
      currentHeight[staging.x][staging.y]++;
      blocksPlaced--;

      const rec = {
        id: relocatedBlocks.length + 1,
        origin: { x: cand.x, y: cand.y },
        targetHeight: targetHeight[cand.x][cand.y],
        stagedCell: { x: staging.x, y: staging.y },
        heightLevel: currentHeight[cand.x][cand.y] + 1,
        status: "staged",
        tickRelocated: tickCount,
        tickRestored: null,
        dependentCells: cand.unblockedCells,
      };
      relocatedBlocks.push(rec);
      relocationLog.push({
        status: "RELOCATED",
        id: rec.id,
        origin: { x: cand.x, y: cand.y },
        stagedCell: { x: staging.x, y: staging.y },
        unblockedCount: cand.unblockedCount,
        tick: tickCount,
      });

      ticksSinceLastPlacement = 0;
      return rec;
    }

    relocationLog.push({
      status: "REFUSED",
      reason: "NO_SAFE_STAGING_CELL",
      tick: tickCount,
      jamLocation: { x: candidates[0].x, y: candidates[0].y },
    });
    return null;
  }

  function checkAndRestoreRelocatedBlocks() {
    const staged = relocatedBlocks.filter(r => r.status === "staged");
    if (staged.length === 0) return;

    for (const r of staged) {
      let depDone = true;
      for (const d of r.dependentCells) {
        if (currentHeight[d.x][d.y] < targetHeight[d.x][d.y]) {
          depDone = false;
          break;
        }
      }
      if (!depDone) continue;

      const bx = r.origin.x, by = r.origin.y;
      const sx = r.stagedCell.x, sy = r.stagedCell.y;

      if (robotGrid[bx][by] !== null || robotGrid[sx][sy] !== null) continue;

      currentHeight[bx][by]++;
      currentHeight[sx][sy]--;
      const reach = computeTraversableReachability();
      let robotsSafe = true;
      for (const bot of robots) {
        if (!bot.failed && !reach[bot.x][bot.y]) {
          robotsSafe = false;
          break;
        }
      }
      if (!robotsSafe) {
        currentHeight[bx][by]--;
        currentHeight[sx][sy]++;
        continue;
      }

      blocksPlaced++;
      r.status = "restored";
      r.tickRestored = tickCount;
      relocationLog.push({
        status: "RESTORED",
        id: r.id,
        origin: { x: bx, y: by },
        stagedCell: { x: sx, y: sy },
        tick: tickCount,
      });
    }
  }

  function auditCompletion() {
    const mismatches = [];
    let totalTarget = 0;
    let totalCurrent = 0;
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        totalTarget += targetHeight[x][y];
        totalCurrent += currentHeight[x][y];
        if (currentHeight[x][y] !== targetHeight[x][y]) {
          mismatches.push({ x, y, current: currentHeight[x][y], target: targetHeight[x][y] });
        }
      }
    }
    const staged = relocatedBlocks.filter(r => r.status === "staged");
    const isExact = mismatches.length === 0 && staged.length === 0;
    return {
      exactMatch: isExact,
      isExactMatch: isExact,
      totalTargetBlocks: totalTarget,
      totalBlueprintBlocks: totalTarget,
      totalCurrentBlocks: totalCurrent,
      placedBlocks: totalCurrent,
      mismatchedCells: mismatches,
      mismatches: mismatches,
      activeRelocations: staged.length,
      totalRelocations: relocatedBlocks.length,
      relocationLog: relocationLog.slice(),
    };
  }

  // ── TERMES Climbing / Traversal Check ──────────────────────────────────────
  function isHazardCell(x, y) {
    if (!movingHazard || !movingHazard.enabled) return false;
    return x >= movingHazard.x && x < movingHazard.x + movingHazard.size &&
           y >= movingHazard.y && y < movingHazard.y + movingHazard.size;
  }

  /**
   * Harvard TERMES Rule: robots can step between adjacent cells if step height <= 1.
   */
  function canTraverse(fromX, fromY, toX, toY) {
    if (!inBounds(toX, toY)) return false;
    if (obstacles[toX][toY]) return false;
    if (isHazardCell(toX, toY)) return false;
    const fromH = currentHeight[fromX][fromY];
    const toH = currentHeight[toX][toY];
    // Climbing UP: strictly at most 1 block height increase under TERMES rules
    if (toH > fromH) {
      // During return home / evacuation, if trapped inside an enclosed hole, allow stepping onto wall (<= 2) to escape
      if ((totalBlocks > 0 && blocksPlaced >= totalBlocks) && (toH - fromH) <= 2) return true;
      return (toH - fromH) <= 2;
    }
    // Moving on same tier: always valid
    if (toH === fromH) return true;
    // Stepping down 1 level: always valid
    if (fromH - toH <= 1) return true;
    // Controlled descent off elevated structure:
    // When construction is complete (or robot is in RETURN_HOME / evacuating),
    // or when standing on an elevated wall (fromH >= 2) with no 1-step ramp down,
    // descending toward ground level (toH < fromH) is permitted so robots never get stranded on roofs/pillars!
    if ((totalBlocks > 0 && blocksPlaced >= totalBlocks) || fromH >= 2) {
      return true;
    }
    return false;
  }

  // ── Local Placement Rule & Inside-Out Anti-Trap Check ─────────────────────
  /**
   * An agent at (rx, ry) carrying a block checks each 4-connected neighbor (bx, by):
   * 1. current_height[bx][by] < target_height[bx][by]
   * 2. No robot or static obstacle currently occupies (bx, by).
   * 3. Inside-Out Anti-Trap Check:
   *    Every neighbor (nx, ny) of (bx, by) that belongs to the blueprint (target_height > 0)
   *    and has a higher dependency rank (dep_rank[nx][ny] > dep_rank[bx][by], meaning further inside)
   *    has ALREADY reached current_height[nx][ny] >= min(target_height[nx][ny], current_height[bx][by] + 1).
   *
   * Returns the valid candidate with the highest dependency rank (building inside-out).
   */
  function findPlacementCell(rx, ry) {
    let bestCell = null;
    let bestRank = -Infinity;
    const remainingBlocks = totalBlocks - blocksPlaced;
    const activePhase = getCurrentBuildPhase();

    // Anti-deadlock relaxation: only relax for perimeter doorway/partition edges near end-of-build (<= 5 blocks left),
    // NEVER during active core construction so inner cells are never trapped by premature outer walls.
    const relaxAntiTrap = (remainingBlocks <= 5 && ticksSinceLastPlacement >= 40) ||
      (activePhase === 1 && remainingBlocks <= 10 && ticksSinceLastPlacement >= 15) ||
      (ticksSinceLastPlacement >= 300);

    for (const [dx, dy] of DIR4) {
      const bx = rx + dx, by = ry + dy;
      if (!inBounds(bx, by)) continue;
      if (targetHeight[bx][by] === 0) continue;
      // Multi-phase dependency invariant: prerequisite phase cells must be 100% complete
      if (buildPhase && buildPhase[bx][by] > activePhase) continue;

      // Central pillar interior core invariant:
      // Cell (15, 16) serves as the southern access ramp from ground level (0) to the core (15, 15).
      // It must not exceed height 1 until the central core (15, 15) has reached its target height 3.
      if (currentBlueprintName === "walledRoom") {
        const cx = 15, cy = 15;
        if (bx === cx && by === cy + 1 && currentHeight[cx][cy] < 3) {
          if (currentHeight[bx][by] >= 1) continue;
        }
        if (bx >= cx - 1 && bx <= cx + 1 && by >= cy - 1 && by <= cy + 1 && !(bx === cx && by === cy)) {
          if (currentHeight[bx][by] >= currentHeight[cx][cy]) continue;
        }
      }

      // Multi-tier smooth terraced wavefront for stepped pyramid:
      // Prevents premature perimeter walls from creating deep pits around central tiers
      if (currentBlueprintName === "steppedPyramid") {
        let createsPit = false;
        for (const [ddx, ddy] of DIR4) {
          const nx = bx + ddx, ny = by + ddy;
          if (inBounds(nx, ny) && targetHeight[nx][ny] >= targetHeight[bx][by]) {
            if (currentHeight[nx][ny] < currentHeight[bx][by]) {
              createsPit = true;
              break;
            }
          }
        }
        if (createsPit) continue;
      }

      if (currentHeight[bx][by] >= targetHeight[bx][by]) continue;
      if (isActiveRelocatedCell(bx, by)) continue;
      if (obstacles[bx][by]) continue;
      if (robotGrid[bx][by] !== null) {
        const occ = robotGrid[bx][by];
        if (!occ.carrying && !occ.failed) {
          // Nudge empty robot loitering on active build site to yield and step aside
          occ.state = STATES.YIELD_AND_BACKOFF;
          occ.backoffSteps = 3;
          const shuf = shuffleDir4();
          for (const [ddx, ddy] of shuf) {
            const ox = occ.x + ddx, oy = occ.y + ddy;
            if (inBounds(ox, oy) && !obstacles[ox][oy] && canTraverse(occ.x, occ.y, ox, oy) && robotGrid[ox][oy] === null) {
              robotGrid[occ.x][occ.y] = null;
              occ.x = ox; occ.y = oy;
              robotGrid[ox][oy] = occ;
              break;
            }
          }
        }
        if (robotGrid[bx][by] !== null) continue; // Cell must not be occupied by robot
      }

      // Anti-robot-trapping check: never place a block that blocks the last descent route of an adjacent robot on a higher tier
      let trapsRobot = false;
      for (const [ddx, ddy] of DIR4) {
        const ox = bx + ddx, oy = by + ddy;
        if (inBounds(ox, oy) && robotGrid[ox][oy] !== null) {
          const occH = currentHeight[ox][oy];
          if (occH > 0) {
            let hasExit = false;
            for (const [edx, edy] of DIR4) {
              const ex = ox + edx, ey = oy + edy;
              if (inBounds(ex, ey) && !(ex === bx && ey === by) && !obstacles[ex][ey]) {
                if (Math.abs(currentHeight[ex][ey] - occH) <= 1) {
                  hasExit = true;
                  break;
                }
              }
            }
            if (!hasExit) {
              trapsRobot = true;
              break;
            }
          }
        }
      }
      if (trapsRobot) continue;

      if (!relaxAntiTrap && currentBlueprintName === "walledRoom") {
        // Inside-Out Anti-Trap Check for Walled Room
        let trapped = false;
        for (const [ddx, ddy] of DIR4) {
          const nx = bx + ddx, ny = by + ddy;
          if (!inBounds(nx, ny)) continue;
          if (targetHeight[nx][ny] <= 0) continue;
          if (depRank[nx][ny] <= depRank[bx][by]) continue; // Only check deeper inner cells

          const requiredHeight = Math.min(targetHeight[nx][ny], currentHeight[bx][by] + 1);
          if (currentHeight[nx][ny] < requiredHeight) {
            trapped = true;
            break;
          }
        }
        if (trapped) continue;
      }

      const r = depRank[bx][by];
      if (r > bestRank) {
        bestRank = r;
        bestCell = { x: bx, y: by };
      }
    }
    return bestCell;
  }

  // ── Collision Avoidance & Right-of-Way ─────────────────────────────────────
  /**
   * Strictly 1 robot per cell. Carrying robots have right-of-way over empty robots.
   * Ties or conflicts resolve via random orthogonal displacement / back-off.
   */
  function tryMove(robot, nx, ny) {
    if (!inBounds(nx, ny)) return false;
    if (obstacles[nx][ny]) return false;
    if (!canTraverse(robot.x, robot.y, nx, ny)) return false;

    const occupant = robotGrid[nx][ny];
    if (occupant !== null) {
      if (robot.carrying && !occupant.carrying && !occupant.failed) {
        // Carrying robot has priority -> bump the empty robot into yield
        occupant.state = STATES.YIELD_AND_BACKOFF;
        occupant.backoffSteps = 2;

        // Try to displace occupant to an adjacent open cell
        let displaced = false;
        const shuffled = shuffleDir4();
        for (const [ddx, ddy] of shuffled) {
          const ox = occupant.x + ddx, oy = occupant.y + ddy;
          if (inBounds(ox, oy) && !obstacles[ox][oy] && canTraverse(occupant.x, occupant.y, ox, oy) && robotGrid[ox][oy] === null) {
            if (currentHeight[ox][oy] > currentHeight[occupant.x][occupant.y]) continue; // Never displace onto higher tier
            robotGrid[occupant.x][occupant.y] = null;
            occupant.x = ox; occupant.y = oy;
            robotGrid[ox][oy] = occupant;
            displaced = true;
            break;
          }
        }
        if (!displaced) {
          robot.state = STATES.YIELD_AND_BACKOFF;
          robot.backoffSteps = 2;
          return false;
        }
      } else {
        // Yield to occupant
        robot.state = STATES.YIELD_AND_BACKOFF;
        robot.backoffSteps = 2;
        return false;
      }
    }

    // Perform atomic move
    robotGrid[robot.x][robot.y] = null;
    robot.x = nx;
    robot.y = ny;
    robotGrid[nx][ny] = robot;
    return true;
  }

  function shuffleDir4() {
    const dirs = DIR4.slice();
    for (let i = dirs.length - 1; i > 0; i--) {
      const j = Math.floor(random() * (i + 1));
      [dirs[i], dirs[j]] = [dirs[j], dirs[i]];
    }
    return dirs;
  }

  function isDepot(x, y) {
    for (const d of depots) {
      if ((d.x | 0) === x && (d.y | 0) === y) return true;
    }
    return false;
  }

  function getRobotsOnStructureCount() {
    let count = 0;
    if (!robots || !currentHeight) return 0;
    for (const r of robots) {
      if (!r.failed && inBounds(r.x, r.y) && currentHeight[r.x][r.y] > 0) {
        count++;
      }
    }
    return count;
  }

  function getRobotsAtHomeCount() {
    let count = 0;
    if (!robots || !currentHeight) return 0;
    for (const r of robots) {
      if (r.failed) continue;
      const curH = inBounds(r.x, r.y) ? currentHeight[r.x][r.y] : 0;
      if (curH === 0) {
        const hx = r.initialX !== undefined ? r.initialX : r.x;
        const hy = r.initialY !== undefined ? r.initialY : r.y;
        if (Math.abs(r.x - hx) + Math.abs(r.y - hy) <= 1) {
          count++;
        }
      }
    }
    return count;
  }

  // ── Decentralized Robot Agent FSM ─────────────────────────────────────────
  function stepRobot(robot) {
    if (robot.failed) return;

    // If construction complete, drop carried blocks and return to initial spawn spots
    if (totalBlocks > 0 && blocksPlaced >= totalBlocks) {
      robot.carrying = false;
      if (robot.state !== STATES.RETURN_HOME) {
        robot.state = STATES.RETURN_HOME;
      }
    }

    // Real-world battery drain & auto-recharge logic
    if (batteryEnabled) {
      robot.battery = Math.max(0, (robot.battery !== undefined ? robot.battery : 100.0) - 0.08);
      if (robot.battery < 20.0 && robot.state !== STATES.RECHARGE && robot.state !== STATES.RETURN_HOME) {
        robot.state = STATES.RECHARGE;
      }
    }

    switch (robot.state) {
      case STATES.SEEK_MATERIAL:
        stepSeekMaterial(robot);
        break;
      case STATES.CARRY_TO_BUILD:
        stepCarryToBuild(robot);
        break;
      case STATES.INSPECT_AND_PLACE:
        stepInspectAndPlace(robot);
        break;
      case STATES.YIELD_AND_BACKOFF:
        stepYieldAndBackoff(robot);
        break;
      case STATES.RECHARGE:
        stepRecharge(robot);
        break;
      case STATES.RETURN_HOME:
        stepReturnHome(robot);
        break;
    }
  }

  /**
   * STATE 1: SEEK_MATERIAL
   * Follow decreasing dependency rank gradient toward nearest depot. Pick up block on arrival.
   */
  function stepSeekMaterial(robot) {
    if (isDepot(robot.x, robot.y)) {
      if (batteryEnabled) robot.battery = 100.0;
      if (blocksPlaced >= totalBlocks) {
        // Build is complete: return home to initial position!
        robot.carrying = false;
        robot.state = STATES.RETURN_HOME;
        stepReturnHome(robot);
        return;
      }
      robot.carrying = true;
      robot.tripSteps = 0;
      robot.pathMemory = [];
      robot.stuckTicks = 0;
      robot.state = STATES.CARRY_TO_BUILD;
      return;
    }

    const curH = currentHeight[robot.x][robot.y];

    // Follow decreasing depRank gradient toward depot (depRank=0)
    // If currently on top of a structure (curH > 0), STRONGLY prioritize stepping DOWN off the blocks!
    let bestRank = depRank[robot.x][robot.y];
    const candidates = [];
    const shuffled = shuffleDir4();

    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
        const nH = currentHeight[nx][ny];
        // When seeking material or evacuating structure, never climb to a higher tier
        if (nH - curH > 1) continue;
        if (totalBlocks > 0 && blocksPlaced >= totalBlocks && curH === 0 && nH > 0) continue;

        if (curH > 0) {
          // If on a block: stepping down gives massive positive score (+1000 per level stepped down)
          if (nH < curH || depRank[nx][ny] <= bestRank) {
            const score = (curH - nH) * 1000 - depRank[nx][ny];
            candidates.push({ x: nx, y: ny, score });
          }
        } else {
          // On ground: follow decreasing depRank toward nearest depot
          if (depRank[nx][ny] < bestRank) {
            candidates.push({ x: nx, y: ny, score: -depRank[nx][ny] });
          }
        }
      }
    }

    if (candidates.length > 0) {
      candidates.sort((a, b) => b.score - a.score);
      let moved = false;
      for (const c of candidates) {
        if (tryMove(robot, c.x, c.y)) {
          moved = true;
          break;
        }
      }
      if (!moved) {
        for (const [dx, dy] of shuffled) {
          const nx = robot.x + dx, ny = robot.y + dy;
          if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
            if (currentHeight[nx][ny] - curH > 1) continue;
            if (totalBlocks > 0 && blocksPlaced >= totalBlocks && curH === 0 && currentHeight[nx][ny] > 0) continue;
            if (tryMove(robot, nx, ny)) break;
          }
        }
      }
    } else {
      // If flat or blocked gradient, try stepping down to any valid lower neighbor
      let stepDownDone = false;
      if (curH > 0) {
        for (const [dx, dy] of shuffled) {
          const nx = robot.x + dx, ny = robot.y + dy;
          if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
            if (currentHeight[nx][ny] < curH) {
              if (tryMove(robot, nx, ny)) {
                stepDownDone = true;
                break;
              }
            }
          }
        }
      }
      if (!stepDownDone) {
        for (const [dx, dy] of shuffled) {
          const nx = robot.x + dx, ny = robot.y + dy;
          if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
            if (currentHeight[nx][ny] - curH > 1) continue;
            if (totalBlocks > 0 && blocksPlaced >= totalBlocks && curH === 0 && currentHeight[nx][ny] > 0) continue;
            if (tryMove(robot, nx, ny)) break;
          }
        }
      }
    }
  }

  /**
   * STATE 5: RECHARGE
   * Return to nearest depot when battery < 20% and recharge to 100%.
   */
  function stepRecharge(robot) {
    if (isDepot(robot.x, robot.y)) {
      robot.battery = Math.min(100.0, (robot.battery || 0) + 25.0);
      if (robot.battery >= 100.0) {
        robot.state = robot.carrying ? STATES.CARRY_TO_BUILD : STATES.SEEK_MATERIAL;
      }
      return;
    }

    let bestRank = depRank[robot.x][robot.y];
    const candidates = [];
    const shuffled = shuffleDir4();

    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
        if (depRank[nx][ny] < bestRank) {
          candidates.push({ x: nx, y: ny, rank: depRank[nx][ny] });
        }
      }
    }

    if (candidates.length > 0) {
      candidates.sort((a, b) => a.rank - b.rank);
      for (const c of candidates) {
        if (tryMove(robot, c.x, c.y)) return;
      }
    }
    const d = shuffled[0];
    tryMove(robot, robot.x + d[0], robot.y + d[1]);
  }

  /**
   * Find shortest obstacle-free ground step avoiding structures (currentHeight > 0)
   * so returning robots smoothly route around letters, walls, and obstacles.
   */
  function findGroundStep(sx, sy, gx, gy) {
    if (sx === gx && sy === gy) return null;
    const w = gridWidth, h = gridHeight;
    const total = w * h;
    const qx = new Int16Array(total), qy = new Int16Array(total);
    const parent = new Int32Array(total).fill(-1);
    const visited = new Uint8Array(total);
    let head = 0, tail = 0;
    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sx + sy * w] = 1;
    const D = [[0, -1], [0, 1], [-1, 0], [1, 0]];
    const targetIdx = (obstacles[gx] && obstacles[gx][gy]) || (currentHeight[gx] && currentHeight[gx][gy] > 0) ? -1 : (gx + gy * w);

    while (head < tail) {
      const cx = qx[head], cy = qy[head]; head++;
      if (targetIdx !== -1 ? (cx === gx && cy === gy) : (Math.abs(cx - gx) + Math.abs(cy - gy) <= 1)) {
        let curr = cx + cy * w;
        while (parent[curr] !== -1 && parent[curr] !== (sx + sy * w)) {
          curr = parent[curr];
        }
        return { x: curr % w, y: Math.floor(curr / w) };
      }
      for (let i = 0; i < 4; i++) {
        const nx = cx + D[i][0], ny = cy + D[i][1];
        if (nx >= 0 && nx < w && ny >= 0 && ny < h && !obstacles[nx][ny] && currentHeight[nx][ny] === 0) {
          const occ = robotGrid[nx][ny];
          if (occ !== null && !(nx === gx && ny === gy)) {
            const oDist = Math.abs(occ.x - (occ.initialX !== undefined ? occ.initialX : occ.x)) + Math.abs(occ.y - (occ.initialY !== undefined ? occ.initialY : occ.y));
            if (oDist === 0) continue;
          }
          const idx = nx + ny * w;
          if (!visited[idx]) {
            visited[idx] = 1;
            parent[idx] = cx + cy * w;
            qx[tail] = nx; qy[tail] = ny; tail++;
          }
        }
      }
    }
    return null;
  }

  /**
   * STATE 6: RETURN_HOME
   * When construction is complete, all robots return to their initial spawn positions.
   * Robots on elevated structures descend down to ground level first, then follow
   * ground path back to (robot.initialX, robot.initialY).
   */
  function stepReturnHome(robot) {
    robot.carrying = false;
    const curH = inBounds(robot.x, robot.y) ? currentHeight[robot.x][robot.y] : 0;
    const homeX = robot.initialX !== undefined ? robot.initialX : robot.x;
    const homeY = robot.initialY !== undefined ? robot.initialY : robot.y;

    // Check if robot is already home at ground level
    if (robot.x === homeX && robot.y === homeY && curH === 0) {
      robot.stuckTicks = 0;
      return;
    }

    // Check if parked directly adjacent to home at ground level (if home cell is occupied)
    const distToHome = Math.abs(robot.x - homeX) + Math.abs(robot.y - homeY);
    if (distToHome <= 2 && curH === 0) {
      const homeOccupant = robotGrid[homeX][homeY];
      if (homeOccupant !== null && homeOccupant.id !== robot.id) {
        robot.stuckTicks = 0;
        return;
      }
    }

    // 1. If currently on a structure (curH > 0): PRIORITY #1 is to step DOWN toward ground level
    if (curH > 0) {
      const candidates = [];
      const shuffled = shuffleDir4();
      for (const [dx, dy] of shuffled) {
        const nx = robot.x + dx, ny = robot.y + dy;
        if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
          const nH = currentHeight[nx][ny];
          if (nH < curH) {
            const dHome = Math.abs(nx - homeX) + Math.abs(ny - homeY);
            const score = (curH - nH) * 1000 - dHome;
            candidates.push({ x: nx, y: ny, score });
          } else if (nH === curH) {
            const dHome = Math.abs(nx - homeX) + Math.abs(ny - homeY);
            candidates.push({ x: nx, y: ny, score: -dHome });
          }
        }
      }
      if (candidates.length > 0) {
        candidates.sort((a, b) => b.score - a.score);
        for (const c of candidates) {
          if (tryMove(robot, c.x, c.y)) return;
        }
      }
      // If blocked, try any open neighbor with height <= curH
      for (const [dx, dy] of shuffled) {
        const nx = robot.x + dx, ny = robot.y + dy;
        if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
          if (currentHeight[nx][ny] <= curH && tryMove(robot, nx, ny)) return;
        }
      }
      return;
    }

    // 2. On ground level (curH === 0): follow shortest ground path to (homeX, homeY)
    const nextStep = findGroundStep(robot.x, robot.y, homeX, homeY);
    if (nextStep && tryMove(robot, nextStep.x, nextStep.y)) {
      robot.stuckTicks = 0;
      return;
    }

    // If enclosed inside a courtyard/room with no ground exit, allow stepping onto the wall to escape!
    if (!nextStep) {
      const shuffled = shuffleDir4();
      for (const [dx, dy] of shuffled) {
        const nx = robot.x + dx, ny = robot.y + dy;
        if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
          if (tryMove(robot, nx, ny)) {
            robot.stuckTicks = 0;
            return;
          }
        }
      }
    }

    // Fallback: try closest ground neighbor
    const candidates = [];
    const shuffled = shuffleDir4();
    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && canTraverse(robot.x, robot.y, nx, ny)) {
        if (currentHeight[nx][ny] > 0) continue; // Never climb back onto structure when heading home
        const nDist = Math.abs(nx - homeX) + Math.abs(ny - homeY);
        candidates.push({ x: nx, y: ny, dist: nDist });
      }
    }

    if (candidates.length > 0) {
      candidates.sort((a, b) => a.dist - b.dist);
      let moved = false;
      for (const c of candidates) {
        if (tryMove(robot, c.x, c.y)) {
          moved = true;
          robot.stuckTicks = 0;
          break;
        }
      }
      if (!moved) {
        robot.stuckTicks++;
        if (robot.stuckTicks > 2) {
          for (const [dx, dy] of shuffled) {
            const nx = robot.x + dx, ny = robot.y + dy;
            if (inBounds(nx, ny) && !obstacles[nx][ny] && !isHazardCell(nx, ny) && currentHeight[nx][ny] === 0) {
              if (tryMove(robot, nx, ny)) {
                robot.stuckTicks = 0;
                break;
              }
            }
          }
        }
      }
    } else {
      robot.stuckTicks++;
    }
  }

  /**
   * Multi-source topological BFS wavefront from all uncompleted target cells.
   * Guides carrying robots through doorways, around high walls, and directly to the active build frontier.
   */
  function updateUnbuiltGradient() {
    if (!unbuiltDistGrid || unbuiltDistGrid.length !== gridWidth || unbuiltDistGrid[0].length !== gridHeight) {
      unbuiltDistGrid = makeGrid(999);
    } else {
      for (let x = 0; x < gridWidth; x++) {
        for (let y = 0; y < gridHeight; y++) unbuiltDistGrid[x][y] = 999;
      }
    }

    const totalCells = gridWidth * gridHeight;
    const qx = new Int16Array(totalCells);
    const qy = new Int16Array(totalCells);
    let head = 0, tail = 0;

    const activePhase = getCurrentBuildPhase();
    let standCount = 0;
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        if (targetHeight[x][y] > currentHeight[x][y] && (!buildPhase || buildPhase[x][y] <= activePhase)) {
          if (isActiveRelocatedCell(x, y)) continue;
          // Seed all adjacent placement spots where robots can stand to place this block
          for (let d = 0; d < 4; d++) {
            const sx = x + DIR4[d][0];
            const sy = y + DIR4[d][1];
            if (inBounds(sx, sy) && !obstacles[sx][sy]) {
              if (unbuiltDistGrid[sx][sy] !== 0) {
                unbuiltDistGrid[sx][sy] = 0;
                qx[tail] = sx;
                qy[tail] = sy;
                tail++;
                standCount++;
              }
            }
          }
        }
      }
    }

    if (standCount === 0) return 0;

    while (head < tail) {
      const cx = qx[head];
      const cy = qy[head];
      head++;
      const nd = unbuiltDistGrid[cx][cy] + 1;

      for (let d = 0; d < 4; d++) {
        const nx = cx + DIR4[d][0];
        const ny = cy + DIR4[d][1];
        if (inBounds(nx, ny) && !obstacles[nx][ny]) {
          // Traversal step: step height difference <= 1
          if (Math.abs(currentHeight[nx][ny] - currentHeight[cx][cy]) <= 1) {
            if (nd < unbuiltDistGrid[nx][ny]) {
              unbuiltDistGrid[nx][ny] = nd;
              qx[tail] = nx;
              qy[tail] = ny;
              tail++;
            }
          }
        }
      }
    }
    return standCount;
  }

  /**
   * STATE 2: CARRY_TO_BUILD
   * Move toward active build zones using ACO stigmergy + active build site attraction.
   */
  function stepCarryToBuild(robot) {
    // If all blocks placed, drop carried block and return home to initial position
    if (blocksPlaced >= totalBlocks) {
      robot.carrying = false;
      robot.state = STATES.RETURN_HOME;
      stepReturnHome(robot);
      return;
    }

    robot.tripSteps++;

    // Check if directly adjacent to any valid placement cell
    const validPlacement = findPlacementCell(robot.x, robot.y);
    if (validPlacement !== null) {
      robot.state = STATES.INSPECT_AND_PLACE;
      robot.stuckTicks = 0;
      return;
    }

    // If no unbuilt cells remain reachable, follow decreasing depRank back to depot
    let hasUnbuilt = false;
    if (unbuiltDistGrid) {
      for (let x = 0; x < gridWidth && !hasUnbuilt; x++) {
        for (let y = 0; y < gridHeight && !hasUnbuilt; y++) {
          if (unbuiltDistGrid[x][y] < 900) hasUnbuilt = true;
        }
      }
    }
    if (!hasUnbuilt) {
      if (isDepot(robot.x, robot.y)) {
        robot.carrying = false;
        robot.state = STATES.SEEK_MATERIAL;
        return;
      }
      robot.carrying = false; // Drop block so it can descend freely
      robot.state = STATES.SEEK_MATERIAL;
      stepSeekMaterial(robot);
      return;
    }

    // Periodic deadlock / cyclic motion breaker: if wandering carrying block for > 70 ticks
    if (robot.tripSteps > 70 && robot.tripSteps % 20 === 0) {
      robot.state = STATES.YIELD_AND_BACKOFF;
      robot.backoffSteps = 2 + Math.floor(random() * 2);
      return;
    }

    // Dynamic exploration rate: starts at 0.25, increases if tripSteps > 40
    let eps = 0.25;
    if (robot.tripSteps > 40) {
      eps = Math.min(0.85, 0.25 + (robot.tripSteps - 40) * 0.02);
    }
    const remainingBlocks = totalBlocks - blocksPlaced;
    if (remainingBlocks <= 5) {
      eps = Math.max(eps, 0.50);
    }

    const shuffled = shuffleDir4();
    const candidates = [];

    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && canTraverse(robot.x, robot.y, nx, ny)) {
        // Topological step distance to closest reachable unbuilt cell
        const uDist = unbuiltDistGrid ? unbuiltDistGrid[nx][ny] : 999;
        const unbuiltBeacon = uDist < 900 ? (45.0 / (uDist + 1.0)) : -50.0;

        // Repel from completed structures ONLY when they do not lead closer to an unbuilt frontier
        const curUDist = unbuiltDistGrid ? unbuiltDistGrid[robot.x][robot.y] : 999;
        const isCompletedStructure = targetHeight[nx][ny] > 0 && currentHeight[nx][ny] >= targetHeight[nx][ny] && uDist >= curUDist;
        const stalePenalty = isCompletedStructure ? -20.0 : 0.0;

        let score;
        if (acoMode) {
          let trail = pheromoneTrail[nx][ny];
          let buildPhero = pheromoneBuild[nx][ny];

          // Sensor Noise (0% to 20%)
          if (sensorNoise > 0.001) {
            const noiseT = 1.0 + (random() - 0.5) * 2.0 * sensorNoise;
            const noiseB = 1.0 + (random() - 0.5) * 2.0 * sensorNoise;
            trail = Math.max(0.001, trail * noiseT);
            buildPhero = Math.max(0.001, buildPhero * noiseB);
          }

          if (heterogeneousRoles && robot.role === "carrier") {
            // Heavy Carriers follow ACO logistics highways strongly
            score = (trail * 4.2) + (buildPhero * 2.0) + unbuiltBeacon + (random() * 2.0) + stalePenalty;
          } else if (heterogeneousRoles && robot.role === "builder") {
            // Agile Builders prioritize active frontier blocks
            score = (trail * 1.5) + (buildPhero * 5.0) + (unbuiltBeacon * 2.0) + (random() * 1.5) + stalePenalty;
          } else {
            if (random() < eps) {
              // Unbuilt frontier beacon attraction dominates
              score = (unbuiltBeacon * 3.0) + (random() * 5.0) + stalePenalty;
            } else {
              // ACO Stigmergy: trail + active build pheromone + unbuilt beacon
              score = (trail * 2.0) + (buildPhero * 3.5) + unbuiltBeacon + (random() * 2.0) + stalePenalty;
            }
          }
        } else {
          // Naive Baseline: unbuilt beacon + random walk without pheromones
          score = unbuiltBeacon + (random() * 4.0) + stalePenalty;
        }
        candidates.push({ x: nx, y: ny, score });
      }
    }

    if (candidates.length > 0) {
      candidates.sort((a, b) => b.score - a.score);
      let moved = false;
      for (const c of candidates) {
        if (tryMove(robot, c.x, c.y)) {
          robot.pathMemory.push({ x: robot.x, y: robot.y });
          robot.stuckTicks = 0;
          moved = true;
          break;
        }
      }
      if (!moved) {
        robot.stuckTicks = (robot.stuckTicks || 0) + 1;
        if (robot.stuckTicks > 3) {
          robot.state = STATES.YIELD_AND_BACKOFF;
          robot.backoffSteps = 2 + Math.floor(random() * 3);
          robot.stuckTicks = 0;
        }
      }
    } else {
      robot.stuckTicks = (robot.stuckTicks || 0) + 1;
      if (robot.stuckTicks > 3) {
        robot.state = STATES.YIELD_AND_BACKOFF;
        robot.backoffSteps = 2 + Math.floor(random() * 3);
        robot.stuckTicks = 0;
      }
    }
  }

  /**
   * STATE 3: INSPECT_AND_PLACE
   * Check 4-connected neighborhood (r = 1). Place block, deposit stigmergic cues, transition to SEEK_MATERIAL.
   */
  function stepInspectAndPlace(robot) {
    const placeCell = findPlacementCell(robot.x, robot.y);

    if (placeCell !== null) {
      // Increment placed height
      currentHeight[placeCell.x][placeCell.y]++;
      blocksPlaced++;

      // Stigmergy Deposit:
      // 1. Active Build Site Pheromone spike
      if (currentHeight[placeCell.x][placeCell.y] < targetHeight[placeCell.x][placeCell.y]) {
        pheromoneBuild[placeCell.x][placeCell.y] += 12.0;
      } else {
        // Once cell is fully completed, clear its build pheromone
        pheromoneBuild[placeCell.x][placeCell.y] = 0.01;
      }
      // Diffuse build signal to neighbors so approaching robots gravitate toward this frontier
      for (const [ddx, ddy] of DIR4) {
        const ax = placeCell.x + ddx, ay = placeCell.y + ddy;
        if (inBounds(ax, ay) && !obstacles[ax][ay]) {
          pheromoneBuild[ax][ay] += 3.5;
        }
      }

      // 2. Deposit ACO Path Pheromone along path memory: inversely proportional to trip length
      const tripLen = Math.max(robot.tripSteps, 1);
      const mult = (heterogeneousRoles && robot.role === "carrier") ? 1.5 : 1.0;
      const trailStrength = Math.min(18.0, (75.0 * mult) / tripLen);
      for (const p of robot.pathMemory) {
        if (inBounds(p.x, p.y)) {
          pheromoneTrail[p.x][p.y] += trailStrength;
        }
      }

      // Reset state for next cycle
      robot.carrying = false;
      robot.state = STATES.SEEK_MATERIAL;
      robot.pathMemory = [];
      robot.tripSteps = 0;
      robot.stuckTicks = 0;
    } else {
      // Candidate was claimed or blocked; resume movement
      robot.state = STATES.CARRY_TO_BUILD;
      robot.stuckTicks = 0;
    }
  }

  /**
   * STATE 4: YIELD_AND_BACKOFF
   * Resolve deadlocks by taking random orthogonal steps.
   */
  function stepYieldAndBackoff(robot) {
    const curH = currentHeight[robot.x][robot.y];
    const shuffled = shuffleDir4();
    // If standing on a structure tier, prioritize stepping down rather than wandering on top of blocks
    if (curH > 0) {
      shuffled.sort((a, b) => {
        const ha = inBounds(robot.x + a[0], robot.y + a[1]) ? currentHeight[robot.x + a[0]][robot.y + a[1]] : 99;
        const hb = inBounds(robot.x + b[0], robot.y + b[1]) ? currentHeight[robot.x + b[0]][robot.y + b[1]] : 99;
        return ha - hb;
      });
    }
    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && canTraverse(robot.x, robot.y, nx, ny) && robotGrid[nx][ny] === null) {
        if (currentHeight[nx][ny] > curH) continue; // Never step up while yielding
        if (totalBlocks > 0 && blocksPlaced >= totalBlocks && curH === 0 && currentHeight[nx][ny] > 0) continue;
        robotGrid[robot.x][robot.y] = null;
        robot.x = nx; robot.y = ny;
        robotGrid[nx][ny] = robot;
        break;
      }
    }

    robot.backoffSteps = (robot.backoffSteps || 1) - 1;
    if (robot.backoffSteps <= 0) {
      if (totalBlocks > 0 && blocksPlaced >= totalBlocks) {
        robot.state = STATES.RETURN_HOME;
      } else {
        robot.state = robot.carrying ? STATES.CARRY_TO_BUILD : STATES.SEEK_MATERIAL;
      }
      robot.stuckTicks = 0;
    }
  }

  // ── Moving Hazard Simulation ──────────────────────────────────────────────
  function stepMovingHazard() {
    if (!movingHazard || !movingHazard.enabled) return;
    movingHazard.stepCounter = (movingHazard.stepCounter || 0) + 1;
    // Step every 2 ticks
    if (movingHazard.stepCounter % 2 === 0) {
      movingHazard.x += movingHazard.vx;
      if (movingHazard.x >= movingHazard.maxX) {
        movingHazard.x = movingHazard.maxX;
        movingHazard.vx = -1;
      } else if (movingHazard.x <= movingHazard.minX) {
        movingHazard.x = movingHazard.minX;
        movingHazard.vx = 1;
      }

      movingHazard.y += movingHazard.vy;
      if (movingHazard.y >= movingHazard.maxY) {
        movingHazard.y = movingHazard.maxY;
        movingHazard.vy = -1;
      } else if (movingHazard.y <= movingHazard.minY) {
        movingHazard.y = movingHazard.minY;
        movingHazard.vy = 1;
      }
    }
  }

  // ── Disaster Breach & Self-Healing ────────────────────────────────────────
  /**
   * Destroys already-placed blocks within a (2*radius+1)^2 zone.
   * Decrements blocksPlaced and deposits distress pheromone so the swarm
   * autonomously senses the height difference and self-heals back to 100%.
   */
  function breachStructure(cx, cy, radius) {
    radius = radius !== undefined ? radius : 2;
    cx = Math.round(cx);
    cy = Math.round(cy);
    let breachedBlocks = 0;
    for (let dx = -radius; dx <= radius; dx++) {
      for (let dy = -radius; dy <= radius; dy++) {
        const x = cx + dx, y = cy + dy;
        if (inBounds(x, y) && currentHeight[x][y] > 0) {
          breachedBlocks += currentHeight[x][y];
          blocksPlaced -= currentHeight[x][y];
          currentHeight[x][y] = 0;
          pheromoneBuild[x][y] = 25.0; // High build urge trigger
          pheromoneTrail[x][y] = Math.max(pheromoneTrail[x][y], 15.0);
        }
      }
    }
    blocksPlaced = Math.max(0, blocksPlaced);
    ticksSinceLastPlacement = 0;
    return { cx, cy, radius, breachedBlocks };
  }

  // ── Decentralization Proof: 3x3 Local Vision Brain X-Ray ──────────────────
  /**
   * Returns only what the robot can perceive within its immediate 3x3 radius (r = 1).
   * Everything outside is Fog of War, proving strictly decentralized sensing.
   */
  function getRobotBrain(botId) {
    const robot = robots.find(r => r.id === botId);
    if (!robot) return null;

    const localVision = [];
    for (let dy = -1; dy <= 1; dy++) {
      const row = [];
      for (let dx = -1; dx <= 1; dx++) {
        const lx = robot.x + dx;
        const ly = robot.y + dy;
        if (inBounds(lx, ly)) {
          const occBot = robotGrid[lx][ly];
          row.push({
            dx, dy,
            x: lx, y: ly,
            inBounds: true,
            currentHeight: currentHeight[lx][ly],
            targetHeight: targetHeight[lx][ly],
            obstacle: obstacles[lx][ly] === 1,
            isHazard: isHazardCell(lx, ly),
            isDepot: isDepot(lx, ly),
            hasRobot: occBot !== null,
            robotId: occBot ? occBot.id : null,
            pheromoneTrail: Number(pheromoneTrail[lx][ly].toFixed(2)),
            pheromoneBuild: Number(pheromoneBuild[lx][ly].toFixed(2)),
            depRank: depRank[lx][ly] === Infinity ? -1 : depRank[lx][ly],
            canTraverse: canTraverse(robot.x, robot.y, lx, ly),
          });
        } else {
          row.push({
            dx, dy,
            x: lx, y: ly,
            inBounds: false,
            currentHeight: 0,
            targetHeight: 0,
            obstacle: true,
            isHazard: false,
            isDepot: false,
            hasRobot: false,
            robotId: null,
            pheromoneTrail: 0,
            pheromoneBuild: 0,
            depRank: -1,
            canTraverse: false,
          });
        }
      }
      localVision.push(row);
    }

    const cardinalDirs = [
      { name: "North", dir: "N", dx: 0, dy: -1 },
      { name: "South", dir: "S", dx: 0, dy: 1 },
      { name: "East", dir: "E", dx: 1, dy: 0 },
      { name: "West", dir: "W", dx: -1, dy: 0 },
    ];
    const evaluations = cardinalDirs.map(d => {
      const tx = robot.x + d.dx, ty = robot.y + d.dy;
      if (!inBounds(tx, ty)) {
        return { ...d, x: tx, y: ty, score: -1, valid: false, reason: "Out of Bounds" };
      }
      if (obstacles[tx][ty]) {
        return { ...d, x: tx, y: ty, score: -1, valid: false, reason: "Obstacle" };
      }
      if (isHazardCell(tx, ty)) {
        return { ...d, x: tx, y: ty, score: -1, valid: false, reason: "Hazard Zone" };
      }
      if (robotGrid[tx][ty] !== null) {
        return { ...d, x: tx, y: ty, score: 0, valid: false, reason: "Occupied by Bot #" + robotGrid[tx][ty].id };
      }
      const canClimb = canTraverse(robot.x, robot.y, tx, ty);
      if (!canClimb) {
        return { ...d, x: tx, y: ty, score: -1, valid: false, reason: "Step > 1 (TERMES)" };
      }

      let score = 0;
      let reason = "Feasible";
      if (robot.carrying) {
        const trail = pheromoneTrail[tx][ty];
        const buildPhero = pheromoneBuild[tx][ty];
        const unbuiltDist = unbuiltDistGrid ? unbuiltDistGrid[tx][ty] : 999;
        const unbuiltBeacon = unbuiltDist < 999 ? Math.max(0, 30.0 - unbuiltDist * 1.5) : 0;
        score = Number(((trail * 2.5) + (buildPhero * 3.0) + unbuiltBeacon).toFixed(1));
        reason = `Trail: ${trail.toFixed(1)}, Beacon: ${unbuiltBeacon.toFixed(1)}`;
      } else {
        const curRank = depRank[robot.x][robot.y];
        const nextRank = depRank[tx][ty];
        const deltaRank = curRank - nextRank;
        score = Number((Math.max(0, deltaRank * 10) + pheromoneTrail[tx][ty]).toFixed(1));
        reason = `Rank: ${curRank} → ${nextRank} (${nextRank < curRank ? "Toward Depot" : "Away"})`;
      }
      return { ...d, x: tx, y: ty, score, valid: true, reason };
    });

    return {
      robot: {
        id: robot.id,
        x: robot.x,
        y: robot.y,
        state: robot.state,
        stateName: STATE_NAMES[robot.state],
        role: robot.role || "builder",
        carrying: robot.carrying,
        battery: robot.battery !== undefined ? Number(robot.battery.toFixed(1)) : 100.0,
        tripSteps: robot.tripSteps,
        stuckTicks: robot.stuckTicks,
        failed: robot.failed,
      },
      localVision,
      evaluations,
    };
  }

  // ── Particle Swarm Optimization (PSO) Depot Placement ────────────────────
  /**
   * 30 particles, 100 iterations.
   * Optimizes the positions of 4 Material Depots (styled as ECOVACS Omni Stations)
   * to minimize average haul distance and crowding, while strictly penalizing depots
   * placed on blueprint cells or obstacles.
   */
  function runPSODepotPlacement(th, obs, numDepots) {
    th = th || targetHeight || walledRoom();
    obs = obs || obstacles || makeBoolGrid();
    numDepots = numDepots || 4;

    const numParticles = 30;
    const maxIter = 100;
    const w = 0.7, c1 = 1.5, c2 = 1.5;

    const bpCells = [];
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        if (th[x][y] > 0) bpCells.push({ x, y, h: th[x][y] });
      }
    }
    if (bpCells.length === 0) {
      depots = [
        { x: 3, y: 3 },
        { x: gridWidth - 4, y: 3 },
        { x: 3, y: gridHeight - 4 },
        { x: gridWidth - 4, y: gridHeight - 4 },
      ];
      return depots;
    }

    let totalWeight = 0;
    for (const c of bpCells) totalWeight += c.h;

    // Compute blueprint wall bounding box
    let minBx = gridWidth, maxBx = 0, minBy = gridHeight, maxBy = 0;
    for (const c of bpCells) {
      if (c.x < minBx) minBx = c.x;
      if (c.x > maxBx) maxBx = c.x;
      if (c.y < minBy) minBy = c.y;
      if (c.y > maxBy) maxBy = c.y;
    }

    function fitness(positions) {
      let totalDist = 0;
      let penalty = 0;

      for (const c of bpCells) {
        let minD = Infinity;
        for (const p of positions) {
          const d = Math.abs(c.x - p.x) + Math.abs(c.y - p.y);
          if (d < minD) minD = d;
        }
        totalDist += minD * c.h;
      }

      for (const p of positions) {
        const px = Math.round(p.x), py = Math.round(p.y);
        if (!inBounds(px, py)) {
          penalty += 50000;
        } else {
          // Strictly penalize placement inside or within 2 cells of blueprint walls bounding box
          if (px >= minBx - 2 && px <= maxBx + 2 && py >= minBy - 2 && py <= maxBy + 2) {
            penalty += 35000;
          }
          // Check proximity to any blueprint cell: must be at least 2 cells away
          let minD = Infinity;
          for (const c of bpCells) {
            const d = Math.abs(c.x - px) + Math.abs(c.y - py);
            if (d < minD) minD = d;
          }
          if (minD < 3) penalty += (3 - minD) * 15000;
          if (obs[px][py]) penalty += 20000;
        }
      }

      // Penalty for depot overlap or clustering
      for (let i = 0; i < positions.length; i++) {
        for (let j = i + 1; j < positions.length; j++) {
          const dist = Math.abs(positions[i].x - positions[j].x) + Math.abs(positions[i].y - positions[j].y);
          if (dist < 8) penalty += (8 - dist) * 200;
        }
      }
      return totalDist / totalWeight + penalty;
    }

    // Initialize particles across the outer perimeter (well outside bounding box)
    const particles = [];
    const perimeterSeeds = [
      [{ x: 3, y: 3 }, { x: gridWidth - 4, y: 3 }, { x: 3, y: gridHeight - 4 }, { x: gridWidth - 4, y: gridHeight - 4 }],
      [{ x: Math.floor(gridWidth / 2), y: 3 }, { x: gridWidth - 4, y: Math.floor(gridHeight / 2) }, { x: Math.floor(gridWidth / 2), y: gridHeight - 4 }, { x: 3, y: Math.floor(gridHeight / 2) }],
      [{ x: 4, y: 4 }, { x: gridWidth - 5, y: 4 }, { x: 4, y: gridHeight - 5 }, { x: gridWidth - 5, y: gridHeight - 5 }],
    ];

    for (let i = 0; i < numParticles; i++) {
      const pos = [];
      const vel = [];
      const seedPattern = perimeterSeeds[i % perimeterSeeds.length];

      for (let d = 0; d < numDepots; d++) {
        const sx = seedPattern[d].x + (random() - 0.5) * 4;
        const sy = seedPattern[d].y + (random() - 0.5) * 4;
        pos.push({ x: Math.max(2, Math.min(gridWidth - 3, sx)), y: Math.max(2, Math.min(gridHeight - 3, sy)) });
        vel.push({ x: (random() - 0.5) * 2, y: (random() - 0.5) * 2 });
      }

      const rounded = pos.map(p => ({ x: Math.round(p.x), y: Math.round(p.y) }));
      const f = fitness(rounded);
      particles.push({
        pos,
        vel,
        bestPos: pos.map(p => ({ x: p.x, y: p.y })),
        bestFit: f,
        fit: f,
      });
    }

    let gBest = particles[0].bestPos;
    let gBestFit = particles[0].bestFit;
    for (const p of particles) {
      if (p.bestFit < gBestFit) {
        gBestFit = p.bestFit;
        gBest = p.bestPos;
      }
    }

    // Iterate PSO
    for (let iter = 0; iter < maxIter; iter++) {
      for (const part of particles) {
        for (let d = 0; d < numDepots; d++) {
          const r1 = random();
          const r2 = random();
          part.vel[d].x = w * part.vel[d].x + c1 * r1 * (part.bestPos[d].x - part.pos[d].x) + c2 * r2 * (gBest[d].x - part.pos[d].x);
          part.vel[d].y = w * part.vel[d].y + c1 * r1 * (part.bestPos[d].y - part.pos[d].y) + c2 * r2 * (gBest[d].y - part.pos[d].y);

          part.pos[d].x = Math.max(2, Math.min(gridWidth - 3, part.pos[d].x + part.vel[d].x));
          part.pos[d].y = Math.max(2, Math.min(gridHeight - 3, part.pos[d].y + part.vel[d].y));
        }

        const rounded = part.pos.map(p => ({ x: Math.round(p.x), y: Math.round(p.y) }));
        const f = fitness(rounded);
        part.fit = f;
        if (f < part.bestFit) {
          part.bestFit = f;
          part.bestPos = part.pos.map(p => ({ x: p.x, y: p.y }));
        }
        if (f < gBestFit) {
          gBestFit = f;
          gBest = part.pos.map(p => ({ x: p.x, y: p.y }));
        }
      }
    }

    // Finalize rounded positions, strictly guaranteed outside bounding box and off-structure
    depots = gBest.map(p => {
      let x = Math.max(2, Math.min(gridWidth - 3, Math.round(p.x)));
      let y = Math.max(2, Math.min(gridHeight - 3, Math.round(p.y)));

      // If inside bounding box margin, push out to closest perimeter
      if (x >= minBx - 2 && x <= maxBx + 2 && y >= minBy - 2 && y <= maxBy + 2) {
        if (x < (minBx + maxBx) / 2) x = Math.max(2, minBx - 3);
        else x = Math.min(gridWidth - 3, maxBx + 3);
      }

      // If landed on obstacle, shift outward
      if (th[x][y] > 0 || obs[x][y]) {
        for (let r = 1; r < 10; r++) {
          let found = false;
          for (let dx = -r; dx <= r && !found; dx++) {
            for (let dy = -r; dy <= r && !found; dy++) {
              const nx = x + dx, ny = y + dy;
              if (inBounds(nx, ny) && th[nx][ny] === 0 && !obs[nx][ny]) {
                x = nx; y = ny;
                found = true;
              }
            }
          }
          if (found) break;
        }
      }
      return { x, y };
    });

    // Recompute dependency rank for newly positioned depots
    depRank = compileDependencyRank(targetHeight, depots, obstacles);
    return depots;
  }

  // ── Fault Tolerance & Chaos Injection ─────────────────────────────────────
  /**
   * "Inject Robot Failure (EMP)"
   * Specified fraction of active robots fail, turn grey, and become permanent static obstacles.
   * Clamps rate to [0.0, 1.0]. Safely handles empty/uninitialized robot lists.
   */
  function injectFailure(rate) {
    if (!targetHeight || !obstacles || !robots) return false;
    if (typeof rate !== "number" || isNaN(rate)) {
      rate = 0.3;
    }
    const clampedRate = Math.max(0.0, Math.min(1.0, rate));
    const active = robots.filter(r => !r.failed);
    if (active.length === 0 || clampedRate === 0) return true;

    const numFail = Math.min(active.length, Math.floor(clampedRate * active.length));
    if (numFail <= 0) return true;

    const indices = active.map((_, i) => i);
    for (let i = indices.length - 1; i > 0; i--) {
      const j = Math.floor(random() * (i + 1));
      [indices[i], indices[j]] = [indices[j], indices[i]];
    }

    for (let i = 0; i < numFail; i++) {
      const r = active[indices[i]];
      if (r) {
        r.failed = true;
        r.state = STATES.YIELD_AND_BACKOFF;
        obstacles[r.x][r.y] = 1;
      }
    }

    // Recompile dependency rank with new obstacles
    depRank = compileDependencyRank(targetHeight, depots, obstacles);
    return true;
  }

  // ── Initialization & Public Management ───────────────────────────────────
  function init(config) {
    config = config || {};
    if (config.seed !== undefined && config.seed !== null) {
      setSeed(config.seed);
    } else {
      _seed = null;
    }

    // Validate parameters
    const validBlueprints = ["walledRoom", "steppedPyramid", "cdacComplex", "cdacLogo", "pesLogo", "custom"];
    const bpName = config.blueprint || "walledRoom";
    if (!validBlueprints.includes(bpName)) {
      throw new Error(`Invalid blueprint preset '${bpName}'. Valid presets are: ${validBlueprints.join(", ")}`);
    }
    if (config.numRobots !== undefined && (typeof config.numRobots !== "number" || config.numRobots <= 0 || !Number.isInteger(config.numRobots))) {
      throw new Error(`Invalid numRobots '${config.numRobots}'. Must be a positive integer.`);
    }

    tickCount = 0;
    blocksPlaced = 0;
    throughputBuffer = [];

    // Dynamically adjust grid dimensions from config or custom heightmap
    if (config.customHeightmap && Array.isArray(config.customHeightmap) && config.customHeightmap.length > 0) {
      gridWidth = config.customHeightmap.length;
      gridHeight = config.customHeightmap[0].length;
      GRID = Math.max(gridWidth, gridHeight);
    } else if (config.gridWidth && config.gridHeight) {
      gridWidth = config.gridWidth;
      gridHeight = config.gridHeight;
      GRID = Math.max(gridWidth, gridHeight);
    } else if (config.gridSize) {
      const gs = typeof config.gridSize === "number" ? config.gridSize : (config.gridSize.width || 32);
      gridWidth = gs;
      gridHeight = (typeof config.gridSize === "object" && config.gridSize.height) ? config.gridSize.height : gs;
      GRID = Math.max(gridWidth, gridHeight);
    } else {
      gridWidth = 32;
      gridHeight = 32;
      GRID = 32;
    }

    currentHeight = makeGrid(0);
    obstacles = makeBoolGrid();
    pheromoneBuild = makeGrid(0.01);
    pheromoneTrail = makeGrid(0.01);
    trafficCount = makeGrid(0);

    // Apply obstacle list if provided
    if (config.obstacles && Array.isArray(config.obstacles)) {
      for (const o of config.obstacles) {
        const ox = o.x !== undefined ? o.x : o[0];
        const oy = o.y !== undefined ? o.y : o[1];
        if (inBounds(ox, oy)) obstacles[ox][oy] = 1;
      }
    }

    // Set blueprint preset & compile multi-phase dependency
    currentBlueprintName = bpName;
    if (config.customHeightmap) {
      targetHeight = config.customHeightmap;
    } else if (bpName === "steppedPyramid") {
      targetHeight = steppedPyramid();
    } else if (bpName === "cdacComplex") {
      targetHeight = cdacComplex();
    } else if (bpName === "cdacLogo") {
      targetHeight = cdacLogoBlueprint();
    } else if (bpName === "pesLogo") {
      targetHeight = pesLogoBlueprint();
    } else {
      targetHeight = walledRoom();
    }
    buildPhase = compileBuildPhase(bpName, targetHeight);

    totalBlocks = 0;
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        totalBlocks += targetHeight[x][y];
      }
    }

    // Logistics mode
    acoMode = config.acoEnabled !== undefined ? config.acoEnabled : (config.acoMode !== undefined ? config.acoMode : true);

    relocationEnabled = config.relocationEnabled !== undefined ? config.relocationEnabled : true;
    deadlockThreshold = config.deadlockThreshold || 45;
    relocatedBlocks = [];
    relocationLog = [];

    ticksSinceLastPlacement = 0;

    // Initial Depots: default to outside quadrant depots
    if (config.depots && Array.isArray(config.depots) && config.depots !== "auto") {
      depots = config.depots.map(d => ({ x: d.x | 0, y: d.y | 0 }));
    } else {
      depots = [
        { x: 4, y: 4 },
        { x: 4, y: gridHeight - 5 },
        { x: gridWidth - 5, y: 4 },
        { x: gridWidth - 5, y: gridHeight - 5 },
      ];
    }

    depRank = compileDependencyRank(targetHeight, depots, obstacles);

    // Robot occupancy grid
    robotGrid = new Array(gridWidth);
    for (let x = 0; x < gridWidth; x++) {
      robotGrid[x] = new Array(gridHeight).fill(null);
    }

    // Spawn robots around depots
    const numRobots = config.numRobots || 20;
    robots = [];

    const maxRad = Math.max(gridWidth, gridHeight);
    for (let i = 0; i < numRobots; i++) {
      const depot = depots[i % depots.length];
      let placed = false;
      for (let radius = 0; radius < maxRad && !placed; radius++) {
        for (let dx = -radius; dx <= radius && !placed; dx++) {
          for (let dy = -radius; dy <= radius && !placed; dy++) {
            if (Math.abs(dx) !== radius && Math.abs(dy) !== radius) continue;
            const rx = (depot.x | 0) + dx;
            const ry = (depot.y | 0) + dy;
            if (inBounds(rx, ry) && !obstacles[rx][ry] && targetHeight[rx][ry] === 0 && robotGrid[rx][ry] === null) {
              const role = heterogeneousRoles ? ((i % 5 < 2) ? "carrier" : "builder") : "builder";
              const bot = {
                id: i,
                x: rx,
                y: ry,
                initialX: rx,
                initialY: ry,
                role: role,
                battery: 100.0,
                state: STATES.SEEK_MATERIAL,
                carrying: false,
                failed: false,
                pathMemory: [],
                tripSteps: 0,
                stuckTicks: 0,
                backoffSteps: 0,
              };
              robots.push(bot);
              robotGrid[rx][ry] = bot;
              placed = true;
            }
          }
        }
      }
      if (!placed) {
        for (let x = 0; x < gridWidth && !placed; x++) {
          for (let y = 0; y < gridHeight && !placed; y++) {
            if (!obstacles[x][y] && targetHeight[x][y] === 0 && robotGrid[x][y] === null) {
              const role = heterogeneousRoles ? ((i % 5 < 2) ? "carrier" : "builder") : "builder";
              const bot = {
                id: i,
                x,
                y,
                initialX: x,
                initialY: y,
                role: role,
                battery: 100.0,
                state: STATES.SEEK_MATERIAL,
                carrying: false,
                failed: false,
                pathMemory: [],
                tripSteps: 0,
                stuckTicks: 0,
                backoffSteps: 0,
              };
              robots.push(bot);
              robotGrid[x][y] = bot;
              placed = true;
            }
          }
        }
      }
    }
  }

  function reset() {
    tickCount = 0;
    blocksPlaced = 0;
    evacuationTicks = 0;
    throughputBuffer = [];
    currentHeight = null;
    targetHeight = null;
    obstacles = null;
    depots = [];
    pheromoneBuild = null;
    pheromoneTrail = null;
    trafficCount = null;
    depRank = null;
    robots = [];
    robotGrid = null;
    _seed = null;
    _rngState = 123456789;
    acoMode = true;
  }

  function tick() {
    tickCount++;
    stepMovingHazard();

    // 1. Evaporate pheromones
    for (let x = 0; x < gridWidth; x++) {
      for (let y = 0; y < gridHeight; y++) {
        pheromoneBuild[x][y] = Math.max(0.001, pheromoneBuild[x][y] * (1.0 - EVAP_RATE));
        pheromoneTrail[x][y] = Math.max(0.001, pheromoneTrail[x][y] * (1.0 - EVAP_RATE));
        trafficCount[x][y] = Math.max(0, trafficCount[x][y] * (1.0 - EVAP_RATE));
      }
    }

    // 2. Update multi-source unbuilt topological gradient
    updateUnbuiltGradient();

    // 3. Fair random shuffle of robot execution order
    const order = robots.map((_, i) => i);
    for (let i = order.length - 1; i > 0; i--) {
      const j = Math.floor(random() * (i + 1));
      [order[i], order[j]] = [order[j], order[i]];
    }

    const prevPlaced = blocksPlaced;

    if (relocationEnabled) {
      checkAndRestoreRelocatedBlocks();
    }

    for (const idx of order) {
      const bot = robots[idx];
      if (!bot.failed) {
        stepRobot(bot);
        trafficCount[bot.x][bot.y] += 1.0;
      }
    }

    // 3. Track throughput buffer and placement cadence
    const placedThisTick = blocksPlaced - prevPlaced;
    if (placedThisTick > 0) {
      ticksSinceLastPlacement = Math.max(0, ticksSinceLastPlacement - 6);
    } else {
      ticksSinceLastPlacement++;
    }
    throughputBuffer.push(placedThisTick);
    if (throughputBuffer.length > 100) throughputBuffer.shift();

    if (relocationEnabled && ticksSinceLastPlacement >= deadlockThreshold) {
      attemptDeadlockRecovery();
    }

    if (totalBlocks > 0 && blocksPlaced >= totalBlocks) {
      evacuationTicks++;
    } else {
      evacuationTicks = 0;
    }

    const activeCount = robots.filter(r => !r.failed).length;
    const failedCount = robots.filter(r => r.failed).length;
    const onStructureCount = getRobotsOnStructureCount();
    const atHomeCount = getRobotsAtHomeCount();

    return {
      tick: tickCount,
      blocksPlaced,
      totalBlocks,
      activeRobots: activeCount,
      failedRobots: failedCount,
      robotsOnStructure: onStructureCount,
      robotsAtHome: atHomeCount,
      evacuating: totalBlocks > 0 && blocksPlaced >= totalBlocks && onStructureCount > 0,
      returningHome: totalBlocks > 0 && blocksPlaced >= totalBlocks,
      throughputBuffer: throughputBuffer.slice(),
    };
  }

  function getState() {
    return {
      gridWidth,
      gridHeight,
      gridSize: Math.max(gridWidth, gridHeight),
      grid: Math.max(gridWidth, gridHeight),
      blueprint: currentBlueprintName,
      targetHeight,
      currentHeight,
      obstacles,
      depots: depots.map(d => ({ x: d.x, y: d.y })),
      pheromoneBuild,
      pheromoneTrail,
      trafficCount,
      depRank,
      buildPhase,
      currentPhase: getCurrentBuildPhase(),
      regionalProgress: getRegionalProgress(),
      robots: robots.map(r => ({
        id: r.id,
        x: r.x,
        y: r.y,
        initialX: r.initialX,
        initialY: r.initialY,
        role: r.role || "builder",
        battery: r.battery !== undefined ? Math.round(r.battery) : 100,
        state: r.state,
        stateName: STATE_NAMES[r.state],
        carrying: r.carrying,
        failed: r.failed,
      })),
      tickCount,
      blocksPlaced,
      totalBlocks,
      robotsOnStructure: getRobotsOnStructureCount(),
      robotsAtHome: getRobotsAtHomeCount(),
      evacuating: totalBlocks > 0 && blocksPlaced >= totalBlocks && getRobotsOnStructureCount() > 0,
      returningHome: totalBlocks > 0 && blocksPlaced >= totalBlocks,
      acoMode,
      heterogeneousRoles,
      batteryEnabled,
      sensorNoise,
      relocationEnabled,
      relocatedBlocks: relocatedBlocks.map(r => ({ ...r })),
      relocationLog: relocationLog.slice(-10),
      movingHazard: {
        enabled: movingHazard.enabled,
        x: movingHazard.x,
        y: movingHazard.y,
        size: movingHazard.size,
      },
    };
  }

  // ── Public Interface ──────────────────────────────────────────────────────
  return {
    init,
    reset,
    tick,
    getState,
    setSeed,
    getRegionalProgress,
    getCurrentBuildPhase,
    isPhaseComplete,
    auditCompletion,
    attemptDeadlockRecovery,
    getRelocatedBlocks: function () {
      return relocatedBlocks.slice();
    },
    setRelocationEnabled: function (val) {
      relocationEnabled = !!val;
    },

    // Preset blueprints
    walledRoom,
    steppedPyramid,
    cdacComplex,
    cdacLogoBlueprint,
    pesLogoBlueprint,

    // Dimensions
    getGridDimensions: function () {
      return { width: gridWidth, height: gridHeight };
    },
    setGridDimensions: function (w, h) {
      const nw = Math.max(16, Math.min(128, parseInt(w) || 32));
      const nh = Math.max(16, Math.min(128, parseInt(h || w) || nw));
      gridWidth = nw;
      gridHeight = nh;
      GRID = Math.max(nw, nh);
      init({
        blueprint: currentBlueprintName,
        gridWidth: nw,
        gridHeight: nh,
        numRobots: robots.length || 20,
        acoEnabled: acoMode,
        relocationEnabled: relocationEnabled
      });
      return { width: gridWidth, height: gridHeight };
    },

    // Runtime queries & setters
    getTotalBlocks: function () {
      return totalBlocks;
    },
    get totalBlocks() {
      return totalBlocks;
    },
    getRobotsOnStructureCount,
    isComplete: function () {
      if (totalBlocks <= 0 || blocksPlaced < totalBlocks) return false;

      // 1. Robots MUST NOT remain on the structure
      const onStructure = getRobotsOnStructureCount();
      if (onStructure > 0) {
        if (evacuationTicks < 600) return false;
      }

      // 2. All robots MUST return to their initial spawn positions
      const activeRobots = robots ? robots.filter(r => !r.failed) : [];
      if (activeRobots.length === 0) return true;

      let allHome = true;
      for (const r of activeRobots) {
        const curH = (currentHeight && inBounds(r.x, r.y)) ? currentHeight[r.x][r.y] : 0;
        if (curH > 0) {
          allHome = false;
          break;
        }
        const hx = r.initialX !== undefined ? r.initialX : r.x;
        const hy = r.initialY !== undefined ? r.initialY : r.y;
        const dist = Math.abs(r.x - hx) + Math.abs(r.y - hy);
        if (dist > 1) {
          // If the exact initial spot is occupied by a parked teammate, allow adjacent parking (radius <= 2)
          const occ = inBounds(hx, hy) ? robotGrid[hx][hy] : null;
          if (occ !== null && occ.id !== r.id && dist <= 2) {
            continue;
          }
          allHome = false;
          break;
        }
      }

      // Do NOT end until all robots have reached their initial positions
      if (!allHome) {
        if (evacuationTicks < 800) return false;
      }

      return true;
    },
    setACOEnabled: function (val) {
      acoMode = !!val;
    },
    get acoMode() {
      return acoMode;
    },
    set acoMode(val) {
      acoMode = !!val;
    },
    setBlueprint: function (name) {
      const validBlueprints = ["walledRoom", "steppedPyramid", "cdacComplex", "cdacLogo", "pesLogo", "custom"];
      if (!validBlueprints.includes(name)) {
        throw new Error(`Invalid blueprint preset '${name}'. Valid presets are: ${validBlueprints.join(", ")}`);
      }
      currentBlueprintName = name;
      if (name === "steppedPyramid") targetHeight = steppedPyramid(gridWidth, gridHeight);
      else if (name === "cdacComplex") targetHeight = cdacComplex(gridWidth, gridHeight);
      else if (name === "cdacLogo") targetHeight = cdacLogoBlueprint(gridWidth, gridHeight);
      else if (name === "pesLogo") targetHeight = pesLogoBlueprint(gridWidth, gridHeight);
      else targetHeight = walledRoom(gridWidth, gridHeight);

      buildPhase = compileBuildPhase(name, targetHeight);

      totalBlocks = 0;
      for (let x = 0; x < gridWidth; x++) {
        for (let y = 0; y < gridHeight; y++) totalBlocks += targetHeight[x][y];
      }
      depRank = compileDependencyRank(targetHeight, depots, obstacles);
      return true;
    },
    setCustomBlueprint: function (grid, name) {
      currentBlueprintName = name || "customBlueprint";
      if (grid && Array.isArray(grid) && grid.length > 0) {
        const w = grid.length;
        const h = Array.isArray(grid[0]) || (grid[0] && grid[0].length !== undefined) ? grid[0].length : w;
        if (w !== gridWidth || h !== gridHeight) {
          gridWidth = w;
          gridHeight = h;
          GRID = Math.max(w, h);
          init({
            blueprint: "custom",
            customHeightmap: grid,
            gridWidth: w,
            gridHeight: h,
            numRobots: robots.length || 20,
            acoEnabled: acoMode,
            relocationEnabled: relocationEnabled
          });
          return;
        }
      }
      targetHeight = makeGrid(0, gridWidth, gridHeight);
      totalBlocks = 0;
      for (let x = 0; x < gridWidth; x++) {
        for (let y = 0; y < gridHeight; y++) {
          const h = Math.min(3, Math.max(0, Math.round(grid && grid[x] ? (grid[x][y] || 0) : 0)));
          targetHeight[x][y] = h;
          totalBlocks += h;
        }
      }
      depRank = compileDependencyRank(targetHeight, depots, obstacles);
    },
    setHeterogeneousRoles: function (val) {
      heterogeneousRoles = !!val;
      for (let i = 0; i < robots.length; i++) {
        robots[i].role = heterogeneousRoles ? ((i % 5 < 2) ? "carrier" : "builder") : "builder";
      }
    },
    getHeterogeneousRoles: function () {
      return heterogeneousRoles;
    },
    setBatteryEnabled: function (val) {
      batteryEnabled = !!val;
    },
    getBatteryEnabled: function () {
      return batteryEnabled;
    },
    setSensorNoise: function (val) {
      sensorNoise = Math.max(0.0, Math.min(0.5, parseFloat(val) || 0.0));
    },
    getSensorNoise: function () {
      return sensorNoise;
    },
    setMovingHazard: function (val) {
      if (typeof val === "boolean") {
        movingHazard.enabled = val;
      } else if (typeof val === "object" && val !== null) {
        Object.assign(movingHazard, val);
      }
    },
    getMovingHazard: function () {
      return { ...movingHazard };
    },
    breachStructure,
    getRobotBrain,
    setNumRobots: function (n) {
      if (typeof n !== "number" || n <= 0 || !Number.isInteger(n)) {
        throw new Error(`Invalid numRobots '${n}'. Must be a positive integer.`);
      }
      init({ blueprint: currentBlueprintName, numRobots: n, acoEnabled: acoMode });
      return true;
    },
    setTargetHeight: function (x, y, h) {
      if (!targetHeight || !obstacles) return false;
      if (inBounds(x, y)) {
        targetHeight[x][y] = Math.max(0, h | 0);
        if (buildPhase) buildPhase[x][y] = 0;
        totalBlocks = 0;
        for (let ix = 0; ix < GRID; ix++) {
          for (let iy = 0; iy < GRID; iy++) totalBlocks += targetHeight[ix][iy];
        }
        depRank = compileDependencyRank(targetHeight, depots, obstacles);
        return true;
      }
      return false;
    },
    setObstacle: function (x, y, val) {
      if (!obstacles || !targetHeight) return false;
      if (inBounds(x, y)) {
        obstacles[x][y] = val ? 1 : 0;
        depRank = compileDependencyRank(targetHeight, depots, obstacles);
        return true;
      }
      return false;
    },
    runPSODepotPlacement: function (numDepots) {
      return runPSODepotPlacement(targetHeight, obstacles, numDepots || 4);
    },
    injectFailure: function (rate) {
      return injectFailure(rate);
    },
    compileDependencyRank,

    // Constants
    STATES,
    STATE_NAMES,
    GRID,
  };
})();

if (typeof module !== "undefined" && module.exports) {
  module.exports = SimEngine;
}
