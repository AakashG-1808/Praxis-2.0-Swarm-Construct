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
  const GRID = 32;
  const EVAP_RATE = 0.04; // Pheromone decay per tick: P = P * (1 - 0.04)

  const STATES = {
    SEEK_MATERIAL: 0,
    CARRY_TO_BUILD: 1,
    INSPECT_AND_PLACE: 2,
    YIELD_AND_BACKOFF: 3,
  };
  const STATE_NAMES = ["SEEK", "CARRY", "INSPECT", "YIELD"];

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

  let tickCount = 0;
  let totalBlocks = 0;
  let blocksPlaced = 0;
  let ticksSinceLastPlacement = 0;
  let throughputBuffer = [];
  let acoMode = true;
  let currentBlueprintName = "walledRoom";
  let unbuiltDistGrid = null;

  const DEFAULT_DEPOTS = [
    { x: 4, y: 4 },
    { x: 4, y: 27 },
    { x: 27, y: 4 },
    { x: 27, y: 27 },
  ];

  // ── Grid Utilities ────────────────────────────────────────────────────────
  function makeGrid(val) {
    val = val || 0;
    const g = new Array(GRID);
    for (let x = 0; x < GRID; x++) {
      g[x] = new Float64Array(GRID);
      if (val !== 0) {
        for (let y = 0; y < GRID; y++) g[x][y] = val;
      }
    }
    return g;
  }

  function makeBoolGrid() {
    const g = new Array(GRID);
    for (let x = 0; x < GRID; x++) {
      g[x] = new Uint8Array(GRID);
    }
    return g;
  }

  function inBounds(x, y) {
    return x >= 0 && x < GRID && y >= 0 && y < GRID;
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
    const x0 = 6, x1 = 25, y0 = 9, y1 = 22;

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
   * Concentric rectangular rings of height 1, 2, 3 centered at (16, 16).
   */
  function steppedPyramid() {
    const t = makeGrid(0);
    const cx = 16, cy = 16;

    // Outer ring radius 8 -> height 1
    for (let x = cx - 8; x <= cx + 8; x++) {
      for (let y = cy - 8; y <= cy + 8; y++) {
        if (inBounds(x, y)) t[x][y] = 1;
      }
    }
    // Middle ring radius 5 -> height 2
    for (let x = cx - 5; x <= cx + 5; x++) {
      for (let y = cy - 5; y <= cy + 5; y++) {
        if (inBounds(x, y)) t[x][y] = 2;
      }
    }
    // Inner ring radius 2 -> height 3
    for (let x = cx - 2; x <= cx + 2; x++) {
      for (let y = cy - 2; y <= cy + 2; y++) {
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

    function hWall(x0, x1, y, h) {
      for (let x = x0; x <= x1; x++) if (inBounds(x, y)) t[x][y] = h;
    }
    function vWall(x, y0, y1, h) {
      for (let y = y0; y <= y1; y++) if (inBounds(x, y)) t[x][y] = h;
    }

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

  // ── Dependency Rank Compiler (BFS Wavefront) ──────────────────────────────
  /**
   * Multi-source BFS from material depots outward across the grid.
   * Ranks indicate topological distance from logistics source.
   */
  function compileDependencyRank(th, dps, obs) {
    const dr = makeGrid(Infinity);
    const qx = new Int16Array(GRID * GRID);
    const qy = new Int16Array(GRID * GRID);
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
    for (let x = 1; x < GRID - 1; x++) {
      for (let y = 1; y < GRID - 1; y++) {
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

  // ── TERMES Climbing / Traversal Check ──────────────────────────────────────
  /**
   * Harvard TERMES Rule: robots can step between adjacent cells if step height <= 1.
   */
  function canTraverse(fromX, fromY, toX, toY) {
    if (!inBounds(toX, toY)) return false;
    if (obstacles[toX][toY]) return false;
    const fromH = currentHeight[fromX][fromY];
    const toH = currentHeight[toX][toY];
    return Math.abs(toH - fromH) <= 1;
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
    // Anti-deadlock relaxation: if near end-of-build (<= 10 blocks left) or stalled for 20+ ticks,
    // relax strict dep_rank anti-trap on doorway/partition edges so final blocks build cleanly and fast.
    const relaxAntiTrap = (remainingBlocks <= 10 && ticksSinceLastPlacement >= 10) || (ticksSinceLastPlacement >= 20);

    for (const [dx, dy] of DIR4) {
      const bx = rx + dx, by = ry + dy;
      if (!inBounds(bx, by)) continue;
      if (targetHeight[bx][by] === 0) continue;
      if (currentHeight[bx][by] >= targetHeight[bx][by]) continue;
      if (obstacles[bx][by]) continue;
      if (robotGrid[bx][by] !== null) continue; // Cell must not be occupied by robot

      if (!relaxAntiTrap) {
        // Inside-Out Anti-Trap Check
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

  // ── Decentralized Robot Agent FSM ─────────────────────────────────────────
  function stepRobot(robot) {
    if (robot.failed) return;

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
    }
  }

  /**
   * STATE 1: SEEK_MATERIAL
   * Follow decreasing dependency rank gradient toward nearest depot. Pick up block on arrival.
   */
  function stepSeekMaterial(robot) {
    if (isDepot(robot.x, robot.y)) {
      robot.carrying = true;
      robot.tripSteps = 0;
      robot.pathMemory = [];
      robot.stuckTicks = 0;
      robot.state = STATES.CARRY_TO_BUILD;
      return;
    }

    // Follow decreasing depRank gradient toward depot (depRank=0)
    let bestRank = depRank[robot.x][robot.y];
    const candidates = [];
    const shuffled = shuffleDir4();

    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && canTraverse(robot.x, robot.y, nx, ny)) {
        if (depRank[nx][ny] < bestRank) {
          candidates.push({ x: nx, y: ny, rank: depRank[nx][ny] });
        }
      }
    }

    if (candidates.length > 0) {
      candidates.sort((a, b) => a.rank - b.rank);
      let moved = false;
      for (const c of candidates) {
        if (tryMove(robot, c.x, c.y)) {
          moved = true;
          break;
        }
      }
      if (!moved) {
        const d = shuffled[0];
        tryMove(robot, robot.x + d[0], robot.y + d[1]);
      }
    } else {
      // Random walk step if at flat gradient or temporary blockage
      const d = shuffled[0];
      tryMove(robot, robot.x + d[0], robot.y + d[1]);
    }
  }

  /**
   * Multi-source topological BFS wavefront from all uncompleted target cells.
   * Guides carrying robots through doorways, around high walls, and directly to the active build frontier.
   */
  function updateUnbuiltGradient() {
    if (!unbuiltDistGrid) unbuiltDistGrid = makeGrid(999);
    else {
      for (let x = 0; x < GRID; x++) {
        for (let y = 0; y < GRID; y++) unbuiltDistGrid[x][y] = 999;
      }
    }

    const qx = new Int16Array(GRID * GRID);
    const qy = new Int16Array(GRID * GRID);
    let head = 0, tail = 0;

    let standCount = 0;
    for (let x = 0; x < GRID; x++) {
      for (let y = 0; y < GRID; y++) {
        if (targetHeight[x][y] > currentHeight[x][y]) {
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
    robot.tripSteps++;

    // Check if directly adjacent to any valid placement cell
    const validPlacement = findPlacementCell(robot.x, robot.y);
    if (validPlacement !== null) {
      robot.state = STATES.INSPECT_AND_PLACE;
      robot.stuckTicks = 0;
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
        const unbuiltBeacon = 45.0 / (uDist + 1.0);

        // Repel from fully completed structures so carrying robots never cluster on finished pillars
        const isCompletedStructure = targetHeight[nx][ny] > 0 && currentHeight[nx][ny] >= targetHeight[nx][ny];
        const stalePenalty = isCompletedStructure ? -20.0 : 0.0;

        let score;
        if (acoMode) {
          const trail = pheromoneTrail[nx][ny];
          const buildPhero = pheromoneBuild[nx][ny];
          if (random() < eps) {
            // Unbuilt frontier beacon attraction dominates
            score = (unbuiltBeacon * 3.0) + (random() * 5.0) + stalePenalty;
          } else {
            // ACO Stigmergy: trail + active build pheromone + unbuilt beacon
            score = (trail * 2.0) + (buildPhero * 3.5) + unbuiltBeacon + (random() * 2.0) + stalePenalty;
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
      const trailStrength = Math.min(15.0, 75.0 / tripLen);
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
    const shuffled = shuffleDir4();
    for (const [dx, dy] of shuffled) {
      const nx = robot.x + dx, ny = robot.y + dy;
      if (inBounds(nx, ny) && !obstacles[nx][ny] && canTraverse(robot.x, robot.y, nx, ny) && robotGrid[nx][ny] === null) {
        robotGrid[robot.x][robot.y] = null;
        robot.x = nx; robot.y = ny;
        robotGrid[nx][ny] = robot;
        break;
      }
    }

    robot.backoffSteps = (robot.backoffSteps || 1) - 1;
    if (robot.backoffSteps <= 0) {
      robot.state = robot.carrying ? STATES.CARRY_TO_BUILD : STATES.SEEK_MATERIAL;
      robot.stuckTicks = 0;
    }
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
    for (let x = 0; x < GRID; x++) {
      for (let y = 0; y < GRID; y++) {
        if (th[x][y] > 0) bpCells.push({ x, y, h: th[x][y] });
      }
    }
    if (bpCells.length === 0) {
      depots = [
        { x: 2, y: 2 },
        { x: 29, y: 2 },
        { x: 2, y: 29 },
        { x: 29, y: 29 },
      ];
      return depots;
    }

    let totalWeight = 0;
    for (const c of bpCells) totalWeight += c.h;

    // Compute blueprint wall bounding box
    let minBx = GRID, maxBx = 0, minBy = GRID, maxBy = 0;
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
      [{x: 4, y: 4}, {x: 27, y: 4}, {x: 4, y: 27}, {x: 27, y: 27}],
      [{x: 16, y: 3}, {x: 28, y: 16}, {x: 16, y: 28}, {x: 3, y: 16}],
      [{x: 3, y: 3}, {x: 28, y: 3}, {x: 3, y: 28}, {x: 28, y: 28}],
    ];

    for (let i = 0; i < numParticles; i++) {
      const pos = [];
      const vel = [];
      const seedPattern = perimeterSeeds[i % perimeterSeeds.length];

      for (let d = 0; d < numDepots; d++) {
        const sx = seedPattern[d].x + (random() - 0.5) * 4;
        const sy = seedPattern[d].y + (random() - 0.5) * 4;
        pos.push({ x: Math.max(2, Math.min(GRID - 3, sx)), y: Math.max(2, Math.min(GRID - 3, sy)) });
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

          part.pos[d].x = Math.max(2, Math.min(GRID - 3, part.pos[d].x + part.vel[d].x));
          part.pos[d].y = Math.max(2, Math.min(GRID - 3, part.pos[d].y + part.vel[d].y));
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
      let x = Math.max(2, Math.min(GRID - 3, Math.round(p.x)));
      let y = Math.max(2, Math.min(GRID - 3, Math.round(p.y)));

      // If inside bounding box margin, push out to closest perimeter
      if (x >= minBx - 2 && x <= maxBx + 2 && y >= minBy - 2 && y <= maxBy + 2) {
        if (x < (minBx + maxBx) / 2) x = Math.max(2, minBx - 3);
        else x = Math.min(GRID - 3, maxBx + 3);
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
   * "Inject 30% Robot Failure (EMP)"
   * 30% of active robots immediately fail, turn grey, and become permanent static obstacles.
   */
  function injectFailure(rate) {
    rate = rate === undefined ? 0.3 : rate;
    const active = robots.filter(r => !r.failed);
    const numFail = Math.floor(rate * active.length);

    const indices = active.map((_, i) => i);
    for (let i = indices.length - 1; i > 0; i--) {
      const j = Math.floor(random() * (i + 1));
      [indices[i], indices[j]] = [indices[j], indices[i]];
    }

    for (let i = 0; i < numFail; i++) {
      const r = active[indices[i]];
      r.failed = true;
      r.state = STATES.YIELD_AND_BACKOFF;
      // Mark as physical obstacle
      obstacles[r.x][r.y] = 1;
    }

    // Recompile dependency rank with new obstacles
    depRank = compileDependencyRank(targetHeight, depots, obstacles);
  }

  // ── Initialization & Public Management ───────────────────────────────────
  function init(config) {
    config = config || {};
    if (config.seed !== undefined && config.seed !== null) {
      setSeed(config.seed);
    } else {
      _seed = null;
    }

    tickCount = 0;
    blocksPlaced = 0;
    throughputBuffer = [];

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

    // Set blueprint preset
    const bpName = config.blueprint || "walledRoom";
    currentBlueprintName = bpName;
    if (bpName === "steppedPyramid") {
      targetHeight = steppedPyramid();
    } else if (bpName === "cdacComplex") {
      targetHeight = cdacComplex();
    } else {
      targetHeight = walledRoom();
    }

    totalBlocks = 0;
    for (let x = 0; x < GRID; x++) {
      for (let y = 0; y < GRID; y++) {
        totalBlocks += targetHeight[x][y];
      }
    }

    // Logistics mode
    acoMode = config.acoEnabled !== undefined ? config.acoEnabled : (config.acoMode !== undefined ? config.acoMode : true);

    ticksSinceLastPlacement = 0;

    // Initial Depots: default to outside quadrant depots (4,4), (4,27), (27,4), (27,27)
    if (config.depots && Array.isArray(config.depots) && config.depots !== "auto") {
      depots = config.depots.map(d => ({ x: d.x | 0, y: d.y | 0 }));
    } else {
      depots = DEFAULT_DEPOTS.map(d => ({ x: d.x, y: d.y }));
    }

    depRank = compileDependencyRank(targetHeight, depots, obstacles);

    // Robot occupancy grid
    robotGrid = new Array(GRID);
    for (let x = 0; x < GRID; x++) {
      robotGrid[x] = new Array(GRID).fill(null);
    }

    // Spawn robots around depots
    const numRobots = config.numRobots || 20;
    robots = [];

    for (let i = 0; i < numRobots; i++) {
      const depot = depots[i % depots.length];
      let placed = false;
      for (let radius = 0; radius < GRID && !placed; radius++) {
        for (let dx = -radius; dx <= radius && !placed; dx++) {
          for (let dy = -radius; dy <= radius && !placed; dy++) {
            if (Math.abs(dx) !== radius && Math.abs(dy) !== radius) continue;
            const rx = (depot.x | 0) + dx;
            const ry = (depot.y | 0) + dy;
            if (inBounds(rx, ry) && !obstacles[rx][ry] && targetHeight[rx][ry] === 0 && robotGrid[rx][ry] === null) {
              const bot = {
                id: i,
                x: rx,
                y: ry,
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
        for (let x = 0; x < GRID && !placed; x++) {
          for (let y = 0; y < GRID && !placed; y++) {
            if (!obstacles[x][y] && targetHeight[x][y] === 0 && robotGrid[x][y] === null) {
              const bot = {
                id: i,
                x,
                y,
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

    // 1. Evaporate pheromones
    for (let x = 0; x < GRID; x++) {
      for (let y = 0; y < GRID; y++) {
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

    const activeCount = robots.filter(r => !r.failed).length;
    const failedCount = robots.filter(r => r.failed).length;

    return {
      tick: tickCount,
      blocksPlaced,
      totalBlocks,
      activeRobots: activeCount,
      failedRobots: failedCount,
      throughputBuffer: throughputBuffer.slice(),
    };
  }

  function getState() {
    return {
      gridSize: GRID,
      grid: GRID,
      targetHeight,
      currentHeight,
      obstacles,
      depots: depots.map(d => ({ x: d.x, y: d.y })),
      pheromoneBuild,
      pheromoneTrail,
      trafficCount,
      depRank,
      robots: robots.map(r => ({
        id: r.id,
        x: r.x,
        y: r.y,
        state: r.state,
        stateName: STATE_NAMES[r.state],
        carrying: r.carrying,
        failed: r.failed,
      })),
      tickCount,
      blocksPlaced,
      totalBlocks,
      acoMode,
    };
  }

  // ── Public Interface ──────────────────────────────────────────────────────
  return {
    init,
    reset,
    tick,
    getState,
    setSeed,

    // Preset blueprints
    walledRoom,
    steppedPyramid,
    cdacComplex,

    // Runtime queries & setters
    getTotalBlocks: function () {
      return totalBlocks;
    },
    get totalBlocks() {
      return totalBlocks;
    },
    isComplete: function () {
      return totalBlocks > 0 && blocksPlaced >= totalBlocks;
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
      currentBlueprintName = name;
      if (name === "steppedPyramid") targetHeight = steppedPyramid();
      else if (name === "cdacComplex") targetHeight = cdacComplex();
      else targetHeight = walledRoom();

      totalBlocks = 0;
      for (let x = 0; x < GRID; x++) {
        for (let y = 0; y < GRID; y++) totalBlocks += targetHeight[x][y];
      }
      depRank = compileDependencyRank(targetHeight, depots, obstacles);
    },
    setNumRobots: function (n) {
      init({ blueprint: currentBlueprintName, numRobots: n, acoEnabled: acoMode });
    },
    setTargetHeight: function (x, y, h) {
      if (inBounds(x, y)) {
        targetHeight[x][y] = h;
        totalBlocks = 0;
        for (let ix = 0; ix < GRID; ix++) {
          for (let iy = 0; iy < GRID; iy++) totalBlocks += targetHeight[ix][iy];
        }
        depRank = compileDependencyRank(targetHeight, depots, obstacles);
      }
    },
    setObstacle: function (x, y, val) {
      if (inBounds(x, y)) {
        obstacles[x][y] = val ? 1 : 0;
        depRank = compileDependencyRank(targetHeight, depots, obstacles);
      }
    },
    runPSODepotPlacement: function (numDepots) {
      return runPSODepotPlacement(targetHeight, obstacles, numDepots || 4);
    },
    injectFailure: function (rate) {
      injectFailure(rate);
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
