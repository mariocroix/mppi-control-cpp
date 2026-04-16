#include <cassert>
#include <iostream>
#include <cmath>

#include "mppi/SMPPIController.hpp"
#include "mppi/PendulumDynamics.hpp"
#include "mppi/PendulumCost.hpp"

using namespace mppi;

int main() {
    Config cfg;
    cfg.nx = 2;
    cfg.nu = 1;
    cfg.K = 700;
    cfg.T = 35;
    cfg.lambda = 0.5;
    cfg.uScale = 1.0;
    cfg.uPerCommand = 1;

    cfg.uMin = Vector::Constant(1, -2.0);
    cfg.uMax = Vector::Constant(1, 2.0);
    cfg.uInit = Vector::Zero(1);

    cfg.noiseMu = Vector::Zero(1);
    cfg.noiseSigmaDiag = Vector::Constant(1, 5.0);

    PendulumDynamics dynamics;
    PendulumCost cost;

    SMPPIController ctrl(cfg, dynamics, cost, nullptr, 0.5, 0.1);

    Vector state(2);
    state << M_PI, 0.0;

    Vector zeroAction = Vector::Zero(1);
    double initialCost = cost.evaluate(state, zeroAction, 0);

    for (int i = 0; i < 40; ++i) {
        Vector action = ctrl.command(state);
        state = dynamics.propagate(state, action, 0);
    }

    double finalCost = cost.evaluate(state, zeroAction, 0);

    std::cout << "Initial cost: " << initialCost << "\n";
    std::cout << "Final cost: " << finalCost << "\n";

    assert(finalCost < initialCost);
    std::cout << "test_smppi_basic passed\n";

    return 0;
}