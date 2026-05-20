import time
import csv
from pathlib import Path

import numpy as np
import mppi_cpp as m


EPISODE_LENGTHS = [40, 80, 120, 160, 200]
OUTPUT_PATH = (
    Path(__file__).resolve().parents[1]
    / "mppi_comparison"
    / "cpu_based"
    / "data"
    / "cpp_api_3_call_cpu_sweep.csv"
)
FIELDNAMES = [
    "episode_steps",
    "total_command_time",
    "average_command_time",
    "final_theta",
    "final_theta_dot",
    "final_cost",
]


def make_controller():
    cfg = m.Config()
    cfg.nx = 2
    cfg.nu = 1
    cfg.K = 200
    cfg.T = 15
    cfg.lambda_ = 1.0
    cfg.uScale = 1.0
    cfg.uPerCommand = 1

    cfg.uMin = np.array([-2.0])
    cfg.uMax = np.array([2.0])
    cfg.uInit = np.array([0.0])
    cfg.noiseMu = np.array([0.0])
    cfg.noiseSigmaDiag = np.array([10.0])

    dynamics = m.PendulumDynamics()
    cost = m.PendulumCost()
    controller = m.MPPIController(cfg, dynamics, cost)

    return controller, dynamics, cost


summary_rows = []

for episode_steps in EPISODE_LENGTHS:
    controller, dynamics, cost = make_controller()

    state = np.array([np.pi, 1.0])
    total_time = 0.0
    final_cost = 0.0

    for step in range(episode_steps):
        start = time.perf_counter()

        action = controller.command(state)
        state = dynamics.propagate(state, action, step)
        final_cost = cost.evaluate(state, action, step)

        elapsed = time.perf_counter() - start
        total_time += elapsed

    summary_rows.append({
        "episode_steps": episode_steps,
        "total_command_time": total_time,
        "average_command_time": total_time / episode_steps,
        "final_theta": float(state[0]),
        "final_theta_dot": float(state[1]),
        "final_cost": float(final_cost),
    })

OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
with OUTPUT_PATH.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
    writer.writeheader()
    writer.writerows(summary_rows)

print(f"Saved {OUTPUT_PATH}")
for row in summary_rows:
    print(row)
