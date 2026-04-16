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
    cfg.uMin = Vector::Constant(2, -1.0);
    cfg.uMax = Vector::Constant(2, 1.0);
    cfg.uInit = Vector::Zero(2);
    cfg.noiseMu = Vector::Zero(2);
    cfg.noiseSigmaDiag = Vector::Ones(2);

    Vector state(2);
    state << 0.0, 0.0;

    MPPIController ctrl1(cfg, dynamics, cost);
    MPPIController ctrl2(cfg, dynamics, cost);

    Vector a1 = ctrl1.command(state);
    Vector a2 = ctrl2.command(state);

    assert((a1 - a2).norm() < 1e-9);

    std::cout << "test_mppi_seed passed\n";
    return 0;
}