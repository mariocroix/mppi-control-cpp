#include <iostream>
#include <cmath>

#include "mppi/cuda/CUDAPendulumMPPI.hpp"
#include "mppi/PendulumDynamics.hpp"
#include "mppi/PendulumCost.hpp"

using namespace mppi;

int main() {
    Config cfg;
    cfg.nx = 2;
    cfg.nu = 1;

    cfg.K = 200;
    cfg.T = 15;
    cfg.lambda = 1.0;
    cfg.uScale = 1.0;

    cfg.uMin = Vector::Constant(1, -2.0);
    cfg.uMax = Vector::Constant(1, 2.0);
    cfg.uInit = Vector::Zero(1);

    cfg.noiseMu = Vector::Zero(1);
    cfg.noiseSigmaDiag = Vector::Constant(1, 10.0);

    CUDAPendulumMPPI ctrl(cfg);

    PendulumDynamics dynamics;
    PendulumCost cost;

    Vector state(2);
    state << M_PI, 1.0;

    std::cout << "Initial state: " << state.transpose() << "\n";

    for (int i = 0; i < 40; ++i) {
        Vector action = ctrl.command(state);

        state = dynamics.propagate(state, action, 0);
        double c = cost.evaluate(state, action, 0);

        std::cout << "Step " << i
                  << " | action: " << action.transpose()
                  << " | theta: " << state(0)
                  << " | theta_dot: " << state(1)
                  << " | cost: " << c
                  << "\n";
    }

    return 0;
}