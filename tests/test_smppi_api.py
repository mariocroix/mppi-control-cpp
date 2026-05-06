import numpy as np
import mppi_cpp as m


def run_pendulum_smppi():
    cfg = m.Config()

    cfg.nx = 2
    cfg.nu = 1

    # Match original Python MPPI example
    cfg.K = 200
    cfg.T = 15

    cfg.lambda_ = 1.0
    cfg.uScale = 1.0

    cfg.uMin = np.array([-2.0])
    cfg.uMax = np.array([2.0])
    cfg.uInit = np.array([0.0])

    cfg.noiseMu = np.array([0.0])

    # Match original:
    # noise_sigma = torch.tensor(10)
    cfg.noiseSigmaDiag = np.array([10.0])

    dynamics = m.PendulumDynamics()
    cost = m.PendulumCost()

    controller = m.SMPPIController(
        cfg,
        dynamics,
        cost,
        None,
        10.0,   # smoothness weight
        0.05    # dt
    )

    # Match original:
    # env.state = [np.pi, 1]
    state = np.array([np.pi, 1.0])

    print("Initial statesss:", state)

    for step in range(40):
        action = controller.command(state)

        state = dynamics.propagate(state, action, 0)

        current_cost = cost.evaluate(state, action, 0)

        print(
            f"Step {step:02d} "
            f"| action: {action[0]: .4f} "
            f"| theta: {state[0]: .4f} "
            f"| theta_dot: {state[1]: .4f} "
            f"| cost: {current_cost: .4f}"
        )

    print("\nFinal action sequence:")
    print(controller.get_action_sequence())


if __name__ == "__main__":
    run_pendulum_smppi()