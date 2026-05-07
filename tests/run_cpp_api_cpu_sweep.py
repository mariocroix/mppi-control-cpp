import time
import csv
import numpy as np
import mppi_cpp as m


EPISODE_LENGTHS = [40, 80, 120, 160, 200]


def make_controller():
    cfg = m.Config()
    cfg.nx = 2
    cfg.nu = 1
    cfg.K = 200
    cfg.T = 15
    cfg.lambda_ = 1.0
    cfg.uScale = 1.0

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

        elapsed = time.perf_counter() - start
        total_time += elapsed

        state = dynamics.propagate(state, action, 0)
        final_cost = cost.evaluate(state, action, 0)

    summary_rows.append({
        "episode_steps": episode_steps,
        "total_command_time": total_time,
        "average_command_time": total_time / episode_steps,
        "final_theta": float(state[0]),
        "final_theta_dot": float(state[1]),
        "final_cost": float(final_cost),
    })

with open("cpp_api_cpu_sweep.csv", "w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "episode_steps",
            "total_command_time",
            "average_command_time",
            "final_theta",
            "final_theta_dot",
            "final_cost",
        ],
    )
    writer.writeheader()
    writer.writerows(summary_rows)

print("Saved cpp_api_cpu_sweep.csv")
for row in summary_rows:
    print(row)