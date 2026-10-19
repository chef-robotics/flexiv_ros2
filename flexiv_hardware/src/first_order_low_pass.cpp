#include "flexiv_hardware/first_order_low_pass.hpp"

#include <cmath>

namespace flexiv_hardware {

double FirstOrderLowPass::gain(double time_constant, double period)
{
    // Not `<= 0.0`: NaN compares false, and must pass through rather than poison the output.
    if (!(time_constant > 0.0) || !(period > 0.0)) {
        return 1.0;
    }
    return period / (time_constant + period);
}

std::optional<double> FirstOrderLowPass::update(double sample, double gain)
{
    // Written so that a NaN gain fails the range check too.
    if (!std::isfinite(sample) || !(gain > 0.0 && gain <= 1.0)) {
        return std::nullopt;
    }
    if (!output_) {
        output_ = sample;
    } else {
        *output_ += gain * (sample - *output_);
    }
    return output_;
}

void FirstOrderLowPass::reset()
{
    output_.reset();
}

} // namespace flexiv_hardware
