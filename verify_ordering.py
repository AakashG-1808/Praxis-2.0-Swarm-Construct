"""
verify_ordering.py — Rigorous Correctness Verification Suite for Walled Room Build Order
Tests:
1. Strict Ordering Invariant: Pillar must reach 27/27 before ANY perimeter cell block is placed.
2. Cell-by-cell blueprint accuracy (no missing, no excess blocks).
3. Openings preserved (doorway and window remain height 0).
4. Interior preserved (no filler blocks in room interior).
5. Regional progress reporting (pillar, perimeter, total).
6. Multi-seed evaluation in both ACO and Baseline modes.
7. Honest timeout labeling.
"""

import sys
import numpy as np
from run_experiments import SwarmSimulation, blueprint_walled_room

SEEDS = [42, 101, 202, 303, 404]
MODES = [("ACO Stigmergy", True), ("Naive Baseline", False)]
MAX_TICKS = 2500

def run_ordering_check():
    print("=" * 80)
    print(" WALLED ROOM BUILD ORDER & CORRECTNESS VERIFICATION SUITE")
    print("=" * 80)

    total_runs = len(SEEDS) * len(MODES)
    passed_runs = 0
    all_violations = []

    results_table = []

    for mode_name, aco_flag in MODES:
        for seed in SEEDS:
            sim = SwarmSimulation(
                blueprint_func=blueprint_walled_room,
                num_robots=20,
                aco_enabled=aco_flag,
                failure_rate=0.0,
                seed=seed,
            )

            violation = None
            pillar_done_tick = None

            while sim.tick_count < MAX_TICKS and not sim.is_complete():
                sim.tick()
                reg = sim.get_regional_progress()
                p_plc = reg["pillar"]["placed"]
                w_plc = reg["perimeter"]["placed"]

                if pillar_done_tick is None and p_plc == 27:
                    pillar_done_tick = sim.tick_count

                # INVARIANT: Zero perimeter blocks can be placed while pillar is incomplete (< 27)
                if p_plc < 27 and w_plc > 0:
                    violation = f"Tick {sim.tick_count}: perimeter placed={w_plc} while pillar placed={p_plc}/27"
                    break

            reg = sim.get_regional_progress()
            is_comp = sim.is_complete()
            exact_match = np.array_equal(sim.current_height, sim.target_height)
            doorway_open = (sim.current_height[15, 22] == 0 and sim.current_height[16, 22] == 0)
            window_open = (sim.current_height[25, 15] == 0 and sim.current_height[25, 16] == 0)
            
            # Interior cells check (excluding pillar [14..16, 14..16] and perimeter walls)
            interior_clean = True
            for x in range(7, 25):
                for y in range(10, 22):
                    if not (14 <= x <= 16 and 14 <= y <= 16):
                        if sim.current_height[x, y] != 0:
                            interior_clean = False

            status = "COMPLETED" if is_comp else "TIMED_OUT"
            passed = (violation is None and is_comp and exact_match and doorway_open and window_open and interior_clean)

            if passed:
                passed_runs += 1
            elif violation:
                all_violations.append((mode_name, seed, violation))

            results_table.append({
                "mode": mode_name,
                "seed": seed,
                "status": status,
                "ticks": sim.tick_count,
                "pillar_tick": pillar_done_tick,
                "pillar_prog": f"{reg['pillar']['placed']}/{reg['pillar']['total']}",
                "perim_prog": f"{reg['perimeter']['placed']}/{reg['perimeter']['total']}",
                "total_prog": f"{reg['total']['placed']}/{reg['total']['total']}",
                "exact_match": exact_match,
                "door_win_open": doorway_open and window_open,
                "interior_clean": interior_clean,
                "passed": passed,
            })

            print(f"[{mode_name:14s} | Seed {seed:3d}] "
                  f"Status: {status:9s} | Ticks: {sim.tick_count:4d} | "
                  f"Pillar Done: tick {str(pillar_done_tick):4s} | "
                  f"Pillar: {reg['pillar']['placed']:2d}/27 ({reg['pillar']['pct']:5.1f}%) | "
                  f"Perim: {reg['perimeter']['placed']:3d}/120 ({reg['perimeter']['pct']:5.1f}%) | "
                  f"Cell-by-cell: {str(exact_match):5s} | Openings: {str(doorway_open and window_open):5s} | "
                  f"Passed: {passed}")

    print("\n" + "=" * 80)
    print(f" SUMMARY: {passed_runs} / {total_runs} runs PASSED all invariants.")
    if all_violations:
        print(f" VIOLATIONS ENCOUNTERED: {len(all_violations)}")
        for m, s, v in all_violations:
            print(f"  - {m} Seed {s}: {v}")
    else:
        print(" ZERO ORDERING VIOLATIONS: Pillar was strictly 100% complete before any perimeter wall block!")
    print("=" * 80)

    if passed_runs < total_runs:
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == "__main__":
    run_ordering_check()
