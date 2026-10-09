#include "flexiv_hardware/command_interface_claims.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace flexiv_hardware {
namespace {

const std::vector<std::string> kArmJoints = {"arm1_joint1", "arm1_joint2", "arm1_joint3",
    "arm1_joint4", "arm1_joint5", "arm1_joint6", "arm1_joint7"};

std::vector<std::string> keys_for(
    const std::vector<std::string>& joints, const std::vector<std::string>& interfaces)
{
    std::vector<std::string> keys;
    for (const auto& joint : joints) {
        for (const auto& interface : interfaces) {
            keys.push_back(joint + "/" + interface);
        }
    }
    return keys;
}

TEST(ClaimedInterfaceBits, NamesOnlyTheGivenJoint)
{
    const auto keys = keys_for({"arm1_joint1"}, {"position"});
    EXPECT_EQ(claimed_interface_bits(keys, "arm1_joint1"), kBitPosition);
    EXPECT_EQ(claimed_interface_bits(keys, "arm1_joint2"), 0);
    // A joint whose name is a prefix of another must not match the longer one.
    EXPECT_EQ(claimed_interface_bits(keys_for({"arm1_joint10"}, {"position"}), "arm1_joint1"), 0);
}

TEST(ClaimedInterfaceBits, CombinesEveryInterfaceOfTheJoint)
{
    const auto keys = keys_for({"arm1_joint1"}, {"position", "velocity", "effort"});
    EXPECT_EQ(
        claimed_interface_bits(keys, "arm1_joint1"), kBitPosition | kBitVelocity | kBitEffort);
}

TEST(InterfaceTypeFromBits, SingleInterfacesAreDriven)
{
    EXPECT_EQ(interface_type_from_bits(kBitPosition), kInterfacePosition);
    EXPECT_EQ(interface_type_from_bits(kBitVelocity), kInterfaceVelocity);
    EXPECT_EQ(interface_type_from_bits(kBitEffort), kInterfaceEffort);
}

TEST(InterfaceTypeFromBits, PositionVelocityAccelerationIsATrajectoryClaim)
{
    EXPECT_EQ(interface_type_from_bits(kBitPosition | kBitVelocity | kBitAcceleration),
        kInterfaceTrajectory);
}

TEST(InterfaceTypeFromBits, OtherCombinationsAreNot)
{
    EXPECT_EQ(interface_type_from_bits(0), kInterfaceNone);
    EXPECT_EQ(interface_type_from_bits(kBitPosition | kBitVelocity), kInterfaceNone);
    EXPECT_EQ(interface_type_from_bits(kBitPosition | kBitAcceleration), kInterfaceNone);
    EXPECT_EQ(interface_type_from_bits(kBitAcceleration), kInterfaceNone);
    EXPECT_EQ(interface_type_from_bits(kBitPosition | kBitEffort), kInterfaceNone);
    EXPECT_EQ(interface_type_from_bits(kBitPosition | kBitVelocity | kBitAcceleration | kBitEffort),
        kInterfaceNone);
}

TEST(ResolveGroupClaim, WholeGroupWithTheTrajectoryTriple)
{
    const GroupClaim claim = resolve_group_claim(
        keys_for(kArmJoints, {"position", "velocity", "acceleration"}), kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kNone);
    EXPECT_EQ(claim.type, kInterfaceTrajectory);
    EXPECT_EQ(claim.claimed_joints, kArmJoints.size());
}

TEST(ResolveGroupClaim, TrajectoryTripleOnSomeJointsOnlyIsPartial)
{
    std::vector<std::string> six(kArmJoints.begin(), kArmJoints.end() - 1);
    const GroupClaim claim
        = resolve_group_claim(keys_for(six, {"position", "velocity", "acceleration"}), kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kPartialGroup);
    EXPECT_EQ(claim.type, kInterfaceTrajectory);
}

TEST(ResolveGroupClaim, NoKeysLeavesTheGroupUnclaimed)
{
    const GroupClaim claim = resolve_group_claim({}, kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kNone);
    EXPECT_EQ(claim.type, kInterfaceNone);
    EXPECT_EQ(claim.claimed_joints, 0u);
}

TEST(ResolveGroupClaim, WholeGroupWithOneType)
{
    const GroupClaim claim = resolve_group_claim(keys_for(kArmJoints, {"velocity"}), kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kNone);
    EXPECT_EQ(claim.type, kInterfaceVelocity);
    EXPECT_EQ(claim.claimed_joints, kArmJoints.size());
}

TEST(ResolveGroupClaim, KeysOfOtherGroupsAreIgnored)
{
    const std::vector<std::string> other = {"arm2_joint1", "arm2_joint2"};
    const GroupClaim claim = resolve_group_claim(keys_for(other, {"position"}), kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kNone);
    EXPECT_EQ(claim.type, kInterfaceNone);
}

TEST(ResolveGroupClaim, SixOfSevenIsPartial)
{
    std::vector<std::string> six(kArmJoints.begin(), kArmJoints.end() - 1);
    const GroupClaim claim = resolve_group_claim(keys_for(six, {"position"}), kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kPartialGroup);
    EXPECT_EQ(claim.type, kInterfacePosition);
    EXPECT_EQ(claim.claimed_joints, 6u);
}

TEST(ResolveGroupClaim, TwoTypesAcrossJointsIsMixed)
{
    auto keys = keys_for({kArmJoints[0]}, {"position"});
    const auto more = keys_for({kArmJoints[1]}, {"velocity"});
    keys.insert(keys.end(), more.begin(), more.end());
    const GroupClaim claim = resolve_group_claim(keys, kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kMixedTypes);
}

TEST(ResolveGroupClaim, TwoTypesOnOneJointIsMixed)
{
    const GroupClaim claim
        = resolve_group_claim(keys_for(kArmJoints, {"position", "velocity"}), kArmJoints);
    EXPECT_EQ(claim.error, GroupClaimError::kMixedTypes);
}

TEST(NextClaimedInterface, StartingWins)
{
    EXPECT_EQ(next_claimed_interface(kInterfaceNone, kInterfacePosition, kInterfaceNone),
        kInterfacePosition);
    EXPECT_EQ(next_claimed_interface(kInterfacePosition, kInterfaceVelocity, kInterfacePosition),
        kInterfaceVelocity);
}

TEST(NextClaimedInterface, StoppingReleases)
{
    EXPECT_EQ(next_claimed_interface(kInterfacePosition, kInterfaceNone, kInterfacePosition),
        kInterfaceNone);
}

TEST(NextClaimedInterface, OtherwiseUnchanged)
{
    EXPECT_EQ(
        next_claimed_interface(kInterfaceEffort, kInterfaceNone, kInterfaceNone), kInterfaceEffort);
}

} // namespace
} // namespace flexiv_hardware
