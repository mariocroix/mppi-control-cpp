#pragma once

#include "Types.hpp"

namespace mppi {

    class RunningCost {
    public:
        virtual ~RunningCost() = default;

        virtual double evaluate(
            const Vector& state,
            const Vector& action,
            int t
        ) const = 0;
    };

    class TerminalCost {
    public:
        virtual ~TerminalCost() = default;

        virtual double evaluate(const Vector& state) const = 0;
    };

}  // namespace mppi