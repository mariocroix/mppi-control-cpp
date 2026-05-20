import argparse
import time

import numpy as np
import mppi_cpp as m


def make_config():
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
    return cfg


def main():
    parser = argparse.ArgumentParser(description="Run a pendulum MPPI 3-call API demo.")
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()

    env = None
    if args.render:
        try:
            import gym

            env = gym.make("Pendulum-v1", render_mode="human")
            env.reset()
        except Exception as exc:
            print(f"Gym render unavailable: {exc}")
            env = None

    cfg = make_config()
    dynamics = m.PendulumDynamics()
    cost = m.PendulumCost()
    controller = m.MPPIController(cfg, dynamics, cost)
    state = np.array([np.pi, 1.0])

    if env is not None:
        env.state = env.unwrapped.state = state.copy()

    for step in range(args.steps):
        command_start = time.perf_counter()
        action = controller.command(state)
        command_time = time.perf_counter() - command_start

        state = dynamics.propagate(state, action, step)
        step_cost = cost.evaluate(state, action, step)

        if env is not None:
            env.step(action)
            env.unwrapped.state = state.copy()
            env.render()
            time.sleep(0.01)

        print(
            f"Step {step:03d} "
            f"| action: {action[0]: .4f} "
            f"| theta: {state[0]: .4f} "
            f"| theta_dot: {state[1]: .4f} "
            f"| cost: {step_cost: .6f} "
            f"| command_time: {command_time: .6f}"
        )

    if env is not None:
        env.close()


if __name__ == "__main__":
    main()
