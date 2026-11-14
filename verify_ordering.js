/**
 * verify_ordering.js — Rigorous Correctness Verification Suite for SimEngine (JavaScript)
 * Tests:
 * 1. Strict Ordering Invariant: Pillar must reach 27/27 before ANY perimeter cell block is placed.
 * 2. Cell-by-cell blueprint accuracy (no missing, no excess blocks).
 * 3. Openings preserved (doorway and window remain height 0).
 * 4. Interior preserved (no filler blocks in room interior).
 * 5. Regional progress reporting (pillar, perimeter, total).
 * 6. Multi-seed evaluation in both ACO and Baseline modes.
 * 7. Honest timeout labeling.
 */

const SimEngine = require("./sim-engine.js");

const SEEDS = [42, 101, 202, 303, 404];
const MODES = [
  { name: "ACO Stigmergy", aco: true },
  { name: "Naive Baseline", aco: false },
];
const MAX_TICKS = 2500;

function runOrderingCheck() {
  console.log("=".repeat(80));
  console.log(" JS SIMENGINE WALLED ROOM BUILD ORDER & CORRECTNESS VERIFICATION SUITE");
  console.log("=".repeat(80));

  const totalRuns = SEEDS.length * MODES.length;
  let passedRuns = 0;
  const allViolations = [];

  for (const mode of MODES) {
    for (const seed of SEEDS) {
      SimEngine.init({
        blueprint: "walledRoom",
        numRobots: 20,
        acoEnabled: mode.aco,
        seed: seed,
      });

      let violation = null;
      let pillarDoneTick = null;

      while (SimEngine.getState().tickCount < MAX_TICKS && !SimEngine.isComplete()) {
        SimEngine.tick();
        const reg = SimEngine.getRegionalProgress();
        const pPlc = reg.pillar.placed;
        const wPlc = reg.perimeter.placed;

        if (pillarDoneTick === null && pPlc === 27) {
          pillarDoneTick = SimEngine.getState().tickCount;
        }

        // INVARIANT: Zero perimeter blocks can be placed while pillar is incomplete (< 27)
        if (pPlc < 27 && wPlc > 0) {
          violation = `Tick ${SimEngine.getState().tickCount}: perimeter placed=${wPlc} while pillar placed=${pPlc}/27`;
          break;
        }
      }

      const state = SimEngine.getState();
      const reg = SimEngine.getRegionalProgress();
      const isComp = SimEngine.isComplete();

      // Check cell-by-cell match
      let exactMatch = true;
      for (let x = 0; x < 32; x++) {
        for (let y = 0; y < 32; y++) {
          if (state.currentHeight[x][y] !== state.targetHeight[x][y]) {
            exactMatch = false;
            break;
          }
        }
      }

      const doorwayOpen = state.currentHeight[15][22] === 0 && state.currentHeight[16][22] === 0;
      const windowOpen = state.currentHeight[25][15] === 0 && state.currentHeight[25][16] === 0;

      // Check interior cells (excluding pillar [14..16, 14..16] and perimeter walls)
      let interiorClean = true;
      for (let x = 7; x < 25; x++) {
        for (let y = 10; y < 22; y++) {
          if (!(x >= 14 && x <= 16 && y >= 14 && y <= 16)) {
            if (state.currentHeight[x][y] !== 0) {
              interiorClean = false;
            }
          }
        }
      }

      const status = isComp ? "COMPLETED" : "TIMED_OUT";
      const passed = violation === null && isComp && exactMatch && doorwayOpen && windowOpen && interiorClean;

      if (passed) {
        passedRuns++;
      } else if (violation) {
        allViolations.push({ mode: mode.name, seed, violation });
      }

      console.log(
        `[${mode.name.padEnd(14)} | Seed ${String(seed).padStart(3)}] ` +
        `Status: ${status.padEnd(9)} | Ticks: ${String(state.tickCount).padStart(4)} | ` +
        `Pillar Done: tick ${String(pillarDoneTick).padStart(4)} | ` +
        `Pillar: ${String(reg.pillar.placed).padStart(2)}/27 (${reg.pillar.pct}%) | ` +
        `Perim: ${String(reg.perimeter.placed).padStart(3)}/120 (${reg.perimeter.pct}%) | ` +
        `Cell-by-cell: ${String(exactMatch).padEnd(5)} | Openings: ${String(doorwayOpen && windowOpen).padEnd(5)} | ` +
        `Passed: ${passed}`
      );
    }
  }

  console.log("\n" + "=".repeat(80));
  console.log(` SUMMARY: ${passedRuns} / ${totalRuns} runs PASSED all invariants.`);
  if (allViolations.length > 0) {
    console.log(` VIOLATIONS ENCOUNTERED: ${allViolations.length}`);
    for (const v of allViolations) {
      console.log(`  - ${v.mode} Seed ${v.seed}: ${v.violation}`);
    }
  } else {
    console.log(" ZERO ORDERING VIOLATIONS: Pillar was strictly 100% complete before any perimeter wall block!");
  }
  console.log("=".repeat(80));

  if (passedRuns < totalRuns) {
    process.exit(1);
  } else {
    process.exit(0);
  }
}

runOrderingCheck();
