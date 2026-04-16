#include <cassert>
#include <iostream>

#include "mppi/LinearDynamics.hpp"
#include "mppi/QuadraticCost.hpp"
#include "mppi/MPPIController.hpp"

using namespace mppi;

class SimpleTerminalCost : public TerminalCost {
public:
    double evaluate(const Vector& state) const override {
        return state.squaredNorm();
    }
};

int main() {
    Matrix B(2, 2);
    B << 1.0, 0.0,
         0.0, -1.0;

    LinearDynamics dynamics(B);

    Vector goal(2);
    goal << 2.0, 2.0;

    QuadraticCost cost(goal);
    SimpleTerminalCost terminalCost;

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

    MPPIController ctrl(cfg, dynamics, cost, &terminalCost);

    Vector state(2);
    state << -1.0, -1.0;

    Vector action = ctrl.command(state);

    assert(action.size() == 2);

    std::cout << "test_mppi_terminal passed\n";
    return 0;
}