#include <cassert>
#include <iostream>

#include "mppi/LinearDynamics.hpp"
#include "mppi/QuadraticCost.hpp"
#include "mppi/MPPIController.hpp"

using namespace mppi;

int main() {
    Matrix B(2, 2);
    B << 1.0, 0.0,
         0.0, -1.0;

    LinearDynamics dynamics(B);

    Vector goal(2);
    goal << 2.0, 2.0;

    QuadraticCost cost(goal);

    Config cfg;
    cfg.nx = 2;
    cfg.nu = 2;
    cfg.K = 100;
    cfg.T = 10;
    cfg.lambda = 1.0;
    cfg.uScale = 1.0;
    cfg.uPerCommand = 1;
    cfg.uMin = Vector::Constant(2, -0.5);
    cfg.uMax = Vector::Constant(2, 0.5);
    cfg.uInit = Vector::Zero(2);
    cfg.noiseMu = Vector::Zero(2);
    cfg.noiseSigmaDiag = Vector::Ones(2);

    MPPIController ctrl(cfg, dynamics, cost);

    Vector state(2);
    state << -3.0, -2.0;

    for (int i = 0; i < 10; ++i) {
        Vector action = ctrl.command(state);

        assert((action.array() <= cfg.uMax.array() + 1e-9).all());
        assert((action.array() >= cfg.uMin.array() - 1e-9).all());

        state = dynamics.propagate(state, action, 0);
    }

    std::cout << "test_mppi_bounds passed\n";
    return 0;
}