import time
import csv
import gym
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

env = gym.make("Pendulum-v1", render_mode="human")
env.reset()

state = np.array([np.pi, 1.0])
env.state = env.unwrapped.state = state.copy()

rows = []
total_command_time = 0.0

print("Initial state:", state)

for step in range(400):
    command_start = time.perf_counter()
    action = controller.command(state)
    elapsed = time.perf_counter() - command_start

    total_command_time += elapsed

    res = env.step(action)
    state = env.unwrapped.state.copy()

    c = cost.evaluate(state, action, 0)

    rows.append({
        "step": step,
        "action": float(action[0]),
        "theta": float(state[0]),
        "theta_dot": float(state[1]),
        "cost": float(c),
        "step_time": elapsed,
    })

    print(
        f"Step {step:02d} "
        f"| action: {action[0]: .4f} "
        f"| theta: {state[0]: .4f} "
        f"| theta_dot: {state[1]: .4f} "
        f"| cost: {c: .4f}"
    )

    env.render()

env.close()

with open("cpp_api_cpu_results.csv", "w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=["step", "action", "theta", "theta_dot", "cost", "step_time"]
    )
    writer.writeheader()
    writer.writerows(rows)

print("Saved cpp_api_gym_results.csv")
print("Total command time:", total_command_time)
print("Average command time:", total_command_time / 40)