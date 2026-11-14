const SimEngine = require("./sim-engine.js");

console.log("================================================================================");
console.log(" TESTING EVACUATION & RETURN HOME LOGIC ACROSS BLUEPRINTS");
console.log("================================================================================");

const blueprints = ["walledRoom", "steppedPyramid", "cdacComplex"];

for (const bp of blueprints) {
  SimEngine.init({
    blueprint: bp,
    gridSize: 32,
    numRobots: 20,
    acoEnabled: true,
    failureRate: 0.0,
    seed: 42
  });

  let maxTicks = 4000;
  let blocksFinishedTick = -1;
  let allBotsHomeTick = -1;

  for (let t = 1; t <= maxTicks; t++) {
    const res = SimEngine.tick();

    if (res.blocksPlaced >= res.totalBlocks && blocksFinishedTick === -1) {
      blocksFinishedTick = t;
    }

    if (SimEngine.isComplete()) {
      allBotsHomeTick = t;
      break;
    }
  }

  const state = SimEngine.getState();
  const onStructure = SimEngine.getRobotsOnStructureCount();
  const activeRobots = state.robots.filter(r => !r.failed);
  
  let unparked = 0;
  for (const r of activeRobots) {
    const hx = r.initialX !== undefined ? r.initialX : r.x;
    const hy = r.initialY !== undefined ? r.initialY : r.y;
    const dist = Math.abs(r.x - hx) + Math.abs(r.y - hy);
    if (dist > 2) unparked++;
  }

  console.log(`[${bp}] Finished: ${blocksFinishedTick}t | All Home: ${allBotsHomeTick}t | On Structure: ${onStructure} | Unparked: ${unparked}`);
  if (onStructure > 0 || unparked > 0 || allBotsHomeTick === -1) {
    console.error(`FAIL: ${bp} had robots stranded!`);
    process.exit(1);
  } else {
    console.log(`PASS: ${bp} evacuated 100% of robots to home base!`);
  }
}

console.log("================================================================================");
console.log(" ALL RETURN HOME TESTS PASSED!");
console.log("================================================================================");
