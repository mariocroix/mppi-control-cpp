#pragma once

#include "Types.hpp"

namespace mppi {

    struct Config {
        int nx = 0;
        int nu = 0;
        int K = 100;
        int T = 15;

        double lambda = 1.0;
        double uScale = 1.0;
        int uPerCommand = 1;

        Vector uMin;
        Vector uMax;
        Vector uInit;

        Vector noiseMu;
        Vector noiseSigmaDiag;

        bool sampleNullAction = false;
    };

}  // namespace mppi