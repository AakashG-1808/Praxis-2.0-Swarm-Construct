const Sim = require("./sim-engine.js");
const blueprints = ["walledRoom", "steppedPyramid", "cdacComplex", "cdacLogo", "pesLogo"];
const botCounts = [10, 20, 30, 50];

console.log("Testing swarm completion across blueprints and bot counts...");

for (const bp of blueprints) {
  for (const n of botCounts) {
    Sim.init({ blueprint: bp, numRobots: n, acoEnabled: true, seed: 42 });
    let t = 0;
    const maxT = 3500;
    while (t < maxT && !Sim.isComplete()) {
      Sim.tick();
      t++;
    }
    const s = Sim.getState();
    const pct = ((s.blocksPlaced / s.totalBlocks) * 100).toFixed(1);
    console.log(
      "BP: " + bp.padEnd(14) +
      " | Bots: " + String(n).padStart(2) +
      " | Ticks: " + String(t).padStart(4) +
      " | Placed: " + String(s.blocksPlaced).padStart(3) + "/" + s.totalBlocks +
      " (" + pct + "%)" +
      " | Complete: " + Sim.isComplete() +
      " | Stuck: " + s.robots.filter(r => r.stuckTicks > 3).length +
      " | OnStructure: " + s.robotsOnStructure
    );
  }
}
