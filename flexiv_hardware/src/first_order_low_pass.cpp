#include "flexiv_hardware/first_order_low_pass.hpp"

#include <cmath>

namespace flexiv_hardware {

double FirstOrderLowPass::gain(double time_constant, double period)
{
    // Not `<= 0.0`: NaN compares false, and must pass through rather than poison the state.
    if (!(time_constant > 0.0) || !(period > 0.0)) {
        return 1.0;
    }
    return period / (time_constant + period);
}

double FirstOrderLowPass::update(double sample, double gain)
{
    if (!std::isfinite(state_)) {
        state_ = sample;
        return state_;
    }
    state_ += gain * (sample - state_);
    return state_;
}

void FirstOrderLowPass::reset()
{
    state_ = std::numeric_limits<double>::quiet_NaN();
}

} // namespace flexiv_hardware
