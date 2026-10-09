#ifndef FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_
#define FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_

#include <cmath>
#include <limits>

namespace flexiv_hardware {

/** Discrete first-order low-pass, y += gain * (x - y), seeded by the first sample after reset(). */
class FirstOrderLowPass
{
public:
    /** Gain for a time constant and period [s]; 1.0 (pass-through) unless both are positive. */
    static double gain(double time_constant, double period)
    {
        if (!(time_constant > 0.0) || !(period > 0.0)) {
            return 1.0;
        }
        return period / (time_constant + period);
    }

    double update(double sample, double gain)
    {
        if (!std::isfinite(state_)) {
            state_ = sample;
            return state_;
        }
        state_ += gain * (sample - state_);
        return state_;
    }

    void reset() { state_ = std::numeric_limits<double>::quiet_NaN(); }

private:
    double state_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_ */
