import time
import csv
import numpy as np
import mppi_cpp as m


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

state = np.array([np.pi, 1.0])

rows = []

start_total = time.perf_counter()

for step in range(40):
    start = time.perf_counter()

    action = controller.command(state)
    state = dynamics.propagate(state, action, 0)
    c = cost.evaluate(state, action, 0)

    end = time.perf_counter()

    rows.append({
        "step": step,
        "action": float(action[0]),
        "theta": float(state[0]),
        "theta_dot": float(state[1]),
        "cost": float(c),
        "step_time": end - start,
    })

end_total = time.perf_counter()

with open("cpp_api_results.csv", "w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=["step", "action", "theta", "theta_dot", "cost", "step_time"]
    )
    writer.writeheader()
    writer.writerows(rows)

print("Saved cpp_api_results.csv")
print("Total time:", end_total - start_total)
print("Average step time:", (end_total - start_total) / 40)