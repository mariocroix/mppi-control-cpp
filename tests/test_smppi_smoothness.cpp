#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>

#include "mppi/MPPIController.hpp"
#include "mppi/SMPPIController.hpp"
#include "mppi/PendulumDynamics.hpp"
#include "mppi/PendulumCost.hpp"

using namespace mppi;

static double totalActionVariation(const std::vector<double>& actions) {
    double total = 0.0;
    for (size_t i = 1; i < actions.size(); ++i) {
        total += std::abs(actions[i] - actions[i - 1]);
    }
    return total;
}

int main() {
    Config cfg;
    cfg.nx = 2;
    cfg.nu = 1;
    cfg.K = 300;
    cfg.T = 25;
    cfg.lambda = 0.5;
    cfg.uScale = 1.0;
    cfg.uPerCommand = 1;

    cfg.uMin = Vector::Constant(1, -2.0);
    cfg.uMax = Vector::Constant(1, 2.0);
    cfg.uInit = Vector::Zero(1);

    cfg.noiseMu = Vector::Zero(1);
    cfg.noiseSigmaDiag = Vector::Constant(1, 3.0);

    PendulumDynamics dynamics;
    PendulumCost cost;

    MPPIController mppi(cfg, dynamics, cost);
    SMPPIController smppi(cfg, dynamics, cost, nullptr, 0.5, 0.1);

    Vector stateMppi(2);
    stateMppi << M_PI, 0.0;

    Vector stateSmppi = stateMppi;

    std::vector<double> actionsMppi;
    std::vector<double> actionsSmppi;

    const int steps = 25;

    for (int i = 0; i < steps; ++i) {
        Vector a1 = mppi.command(stateMppi);
        Vector a2 = smppi.command(stateSmppi);

        actionsMppi.push_back(a1(0));
        actionsSmppi.push_back(a2(0));

        stateMppi = dynamics.propagate(stateMppi, a1, 0);
        stateSmppi = dynamics.propagate(stateSmppi, a2, 0);
    }

    double variationMppi = totalActionVariation(actionsMppi);
    double variationSmppi = totalActionVariation(actionsSmppi);

    std::cout << "MPPI variation: " << variationMppi << "\n";
    std::cout << "SMPPI variation: " << variationSmppi << "\n";

    assert(variationSmppi < variationMppi);
    std::cout << "test_smppi_smoothness passed\n";

    return 0;
}