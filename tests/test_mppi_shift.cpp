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

    MPPIController controller(cfg, dynamics, cost);

    Vector state(2);
    state << -3.0, -2.0;

    controller.command(state);

    Matrix before = controller.getActionSequence();

    controller.shiftNominalTrajectory();

    Matrix after = controller.getActionSequence();

    assert(after.rows() == before.rows());
    assert(after.cols() == before.cols());

    for (int t = 0; t < before.rows() - 1; ++t) {
        assert((after.row(t) - before.row(t + 1)).norm() < 1e-9);
    }

    assert((after.row(after.rows() - 1).transpose() - cfg.uInit).norm() < 1e-9);

    std::cout << "test_mppi_shift passed\n";
    return 0;
}