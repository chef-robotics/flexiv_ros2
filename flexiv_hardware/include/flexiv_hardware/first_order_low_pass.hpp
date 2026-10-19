#ifndef FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_
#define FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_

#include <limits>

namespace flexiv_hardware {

/** Discrete first-order low-pass, y += gain * (x - y), seeded by the first sample after reset(). */
class FirstOrderLowPass
{
public:
    /** Gain for a time constant and period [s]; 1.0 (pass-through) unless both are positive. */
    static double gain(double time_constant, double period);

    double update(double sample, double gain);

    void reset();

private:
    double state_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_ */
