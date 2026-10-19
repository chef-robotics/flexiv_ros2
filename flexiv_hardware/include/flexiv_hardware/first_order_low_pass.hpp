#ifndef FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_
#define FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_

#include <optional>

namespace flexiv_hardware {

/** Discrete first-order low-pass, y += gain * (x - y), seeded by the first sample after reset(). */
class FirstOrderLowPass
{
public:
    /** Gain for a time constant and period [s]; 1.0 (pass-through) unless both are positive. */
    static double gain(double time_constant, double period);

    /**
     * Filter `sample` and return the new output. A non-finite sample is ignored: the output holds,
     * or is 0.0 while the filter is still unseeded.
     */
    double update(double sample, double gain);

    void reset();

private:
    /** Last output y; empty until the first finite sample after construction or reset(). */
    std::optional<double> output_;
};

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_ */
