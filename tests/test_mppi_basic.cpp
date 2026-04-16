#include <cassert>
#include <iostream>

#include "mppi/LinearDynamics.hpp"
#include "mppi/QuadraticCost.hpp"
#include "mppi/MPPIController.hpp"

using namespace mppi;

int main() {
    Vector state(2);
    state << -3.0, -2.0;

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
    cfg.uMin = Vector::Constant(2, -1.0);
    cfg.uMax = Vector::Constant(2, 1.0);
    cfg.uInit = Vector::Zero(2);
    cfg.noiseMu = Vector::Zero(2);
    cfg.noiseSigmaDiag = Vector::Ones(2);

    MPPIController controller(cfg, dynamics, cost);

    double initialCost = cost.evaluate(state, Vector::Zero(2), 0);

    for (int step = 0; step < 5; ++step) {
        Vector action = controller.command(state);
        state = dynamics.propagate(state, action, 0);
    }

    double finalCost = cost.evaluate(state, Vector::Zero(2), 0);

    std::cout << "Initial cost: " << initialCost << "\n";
    std::cout << "Final cost: " << finalCost << "\n";

    assert(finalCost < initialCost);
    std::cout << "test_mppi_basic passed\n";

    return 0;
}