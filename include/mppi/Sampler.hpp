#pragma once

#include <cstddef>

#include "mppi/Types.hpp"

namespace mppi {

class Sampler {
public:
    virtual ~Sampler() = default;

    virtual Control sample(std::size_t control_dim) = 0;
};

}  // namespace mppi
