#pragma once

#include "Types.hpp"

namespace mppi {

    class Dynamics {
    public:
        virtual ~Dynamics() = default;

        virtual Vector propagate(
            const Vector& state,
            const Vector& action,
            int t
        ) const = 0;
    };

}  // namespace mppi