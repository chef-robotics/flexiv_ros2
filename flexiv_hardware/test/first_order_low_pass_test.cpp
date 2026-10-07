#include "flexiv_hardware/first_order_low_pass.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace flexiv_hardware {
namespace {

constexpr double kPeriod = 0.001;
constexpr double kTimeConstant = 0.02;

TEST(FirstOrderLowPassGain, ZeroTimeConstantPassesThrough)
{
    EXPECT_DOUBLE_EQ(FirstOrderLowPass::gain(0.0, kPeriod), 1.0);
}

TEST(FirstOrderLowPassGain, NonPositivePeriodPassesThrough)
{
    EXPECT_DOUBLE_EQ(FirstOrderLowPass::gain(kTimeConstant, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(FirstOrderLowPass::gain(kTimeConstant, -kPeriod), 1.0);
}

TEST(FirstOrderLowPassGain, FollowsThePeriodOverTimeConstantRatio)
{
    EXPECT_DOUBLE_EQ(
        FirstOrderLowPass::gain(kTimeConstant, kPeriod), kPeriod / (kTimeConstant + kPeriod));
}

TEST(FirstOrderLowPass, SeedsFromTheFirstSample)
{
    FirstOrderLowPass filter;
    EXPECT_DOUBLE_EQ(filter.update(1.5, 0.1), 1.5);
}

TEST(FirstOrderLowPass, UnitGainPassesThrough)
{
    FirstOrderLowPass filter;
    filter.update(0.0, 1.0);
    EXPECT_DOUBLE_EQ(filter.update(2.0, 1.0), 2.0);
    EXPECT_DOUBLE_EQ(filter.update(-3.0, 1.0), -3.0);
}

TEST(FirstOrderLowPass, SettlesOnAStepWithinFiveTimeConstants)
{
    FirstOrderLowPass filter;
    const double gain = FirstOrderLowPass::gain(kTimeConstant, kPeriod);
    filter.update(0.0, gain);
    double value = 0.0;
    const int steps = static_cast<int>(5.0 * kTimeConstant / kPeriod);
    for (int i = 0; i < steps; ++i) {
        value = filter.update(1.0, gain);
    }
    // One time constant in leaves 1/e of the step; five leave under 1%.
    EXPECT_NEAR(value, 1.0, 0.01);
    EXPECT_LT(value, 1.0);
}

TEST(FirstOrderLowPass, OneTimeConstantReachesAboutSixtyThreePercent)
{
    FirstOrderLowPass filter;
    const double gain = FirstOrderLowPass::gain(kTimeConstant, kPeriod);
    filter.update(0.0, gain);
    double value = 0.0;
    const int steps = static_cast<int>(kTimeConstant / kPeriod);
    for (int i = 0; i < steps; ++i) {
        value = filter.update(1.0, gain);
    }
    EXPECT_NEAR(value, 1.0 - std::exp(-1.0), 0.03);
}

TEST(FirstOrderLowPass, ResetReseedsFromTheNextSample)
{
    FirstOrderLowPass filter;
    filter.update(0.0, 0.5);
    filter.update(10.0, 0.5);
    filter.reset();
    EXPECT_DOUBLE_EQ(filter.update(-4.0, 0.5), -4.0);
}

} // namespace
} // namespace flexiv_hardware
