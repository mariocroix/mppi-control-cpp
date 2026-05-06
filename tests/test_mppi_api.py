import numpy as np
import mppi_cpp as m

cfg = m.Config()
cfg.nx = 2
cfg.nu = 1

# Match original Python pendulum.py
cfg.K = 200          # N_SAMPLES
cfg.T = 15           # TIMESTEPS
cfg.lambda_ = 1.0
cfg.uScale = 1.0

cfg.uMin = np.array([-2.0])
cfg.uMax = np.array([2.0])
cfg.uInit = np.array([0.0])

cfg.noiseMu = np.array([0.0])
cfg.noiseSigmaDiag = np.array([10.0])  # noise_sigma = torch.tensor(10)

dyn = m.PendulumDynamics()
cost = m.PendulumCost()

ctrl = m.MPPIController(cfg, dyn, cost)

# Match original: env.state = [np.pi, 1]
state = np.array([np.pi, 1.0])

print("Initial state:", state)

for i in range(40):
    action = ctrl.command(state)
    state = dyn.propagate(state, action, 0)
    c = cost.evaluate(state, action, 0)

    print(
        f"Step {i} | action: {action[0]: .4f} "
        f"| theta: {state[0]: .4f} "
        f"| theta_dot: {state[1]: .4f} "
        f"| cost: {c: .4f}"
    )