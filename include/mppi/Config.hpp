#pragma once

#include "Types.hpp"

namespace mppi {

    struct Config {
        int nx = 0;
        int nu = 0;
        int K = 100;
        int T = 15;
		int seed = 42;

        double lambda = 1.0;
        double uScale = 1.0; // Scales the control before applying it.
        int uPerCommand = 1; // How many actions from the optimized trajectory should be executed before replanning.

        Vector uMin; // Lower bound for each control dimension.
        Vector uMax; // Upper bound for each control dimension.
        Vector uInit;  // Default control used to initialize the trajectory.

        Vector noiseMu; // Mean of sampled control noise.
        Vector noiseSigmaDiag; // Variance of exploration noise. Controls exploration strength.
    };

}  // namespace mppi