#include <cassert>
#include <iostream>

#include "mppi/MPPIController.hpp"

using namespace mppi;

class Dynamics1D : public Dynamics {
public:
    Vector propagate(const Vector& state, const Vector& action, int) const override {
        Vector next(1);
        next << state(0) + action(0);
        return next;
    }
};

class Cost1D : public RunningCost {
public:
    double evaluate(const Vector& state, const Vector&, int) const override {
        double dx = state(0) - 1.0;
        return dx * dx;
    }
};

int main() {
    Dynamics1D dynamics;
    Cost1D cost;

    Config cfg;
    cfg.nx = 1;
    cfg.nu = 1;
    cfg.K = 50;
    cfg.T = 5;
    cfg.lambda = 1.0;
    cfg.uScale = 1.0;
    cfg.uPerCommand = 1;
    cfg.uMin = Vector::Constant(1, -2.0);
    cfg.uMax = Vector::Constant(1, 2.0);
    cfg.uInit = Vector::Zero(1);
    cfg.noiseMu = Vector::Zero(1);
    cfg.noiseSigmaDiag = Vector::Ones(1);

    MPPIController ctrl(cfg, dynamics, cost);

    Vector state(1);
    state << 0.0;

    Vector action = ctrl.command(state);

    assert(action.size() == 1);

    std::cout << "test_mppi_1d passed\n";
    return 0;
}