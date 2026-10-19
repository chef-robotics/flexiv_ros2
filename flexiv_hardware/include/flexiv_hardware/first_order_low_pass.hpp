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
     * Filter `sample` and return the new output, or std::nullopt, leaving the filter unchanged,
     * when `sample` is not finite or `gain` is not in (0, 1].
     */
    std::optional<double> update(double sample, double gain);

    void reset();

private:
    /** Last output y; empty until the first valid update() after construction or reset(). */
    std::optional<double> output_;
};

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__FIRST_ORDER_LOW_PASS_HPP_ */
