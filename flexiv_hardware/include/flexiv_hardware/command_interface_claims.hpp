#ifndef FLEXIV_HARDWARE__COMMAND_INTERFACE_CLAIMS_HPP_
#define FLEXIV_HARDWARE__COMMAND_INTERFACE_CLAIMS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Maps ros2_control command interface names onto per-joint-group claims, without the RDK.
namespace flexiv_hardware {

/**
 * ROS 2 command interface types that this hardware interface can claim and drive.
 * kInterfaceNone means the group is unclaimed.
 */
enum CommandInterfaceType : uint8_t
{
    kInterfaceNone = 0,
    kInterfacePosition,
    kInterfaceVelocity,
    kInterfaceEffort,
    /** Position, velocity and acceleration together; the latter two stream as feedforward. */
    kInterfaceTrajectory,
};

/** The command interfaces named for one joint, as a bit set. */
enum CommandInterfaceBits : uint8_t
{
    kBitPosition = 1 << 0,
    kBitVelocity = 1 << 1,
    kBitAcceleration = 1 << 2,
    kBitEffort = 1 << 3,
};

/** The command interfaces in `keys` that belong to `joint_name`, as CommandInterfaceBits. */
uint8_t claimed_interface_bits(
    const std::vector<std::string>& keys, const std::string& joint_name);

/**
 * The CommandInterfaceType a joint claimed with exactly `bits` is driven by, or kInterfaceNone
 * when `bits` is empty or a combination this hardware interface does not drive.
 */
CommandInterfaceType interface_type_from_bits(uint8_t bits);

/** Why `keys` cannot claim a joint group. */
enum class GroupClaimError
{
    kNone,
    /** A joint claims an undriven combination of interfaces, or joints differ in type. */
    kMixedTypes,
    /** Some but not all joints of the group are claimed. */
    kPartialGroup,
};

struct GroupClaim
{
    GroupClaimError error = GroupClaimError::kNone;
    /** kInterfaceNone when `keys` names no joint of the group. */
    CommandInterfaceType type = kInterfaceNone;
    /** Joints claimed, counted as far as the resolution got. */
    size_t claimed_joints = 0;
};

/**
 * Resolve the interface type `keys` claim on the joint group `joint_names`, which must be claimed
 * whole with one type.
 */
GroupClaim resolve_group_claim(
    const std::vector<std::string>& keys, const std::vector<std::string>& joint_names);

/** Interface a joint group ends up claimed with, given what a mode switch starts and stops. */
uint8_t next_claimed_interface(uint8_t current, uint8_t starting, uint8_t stopping);

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__COMMAND_INTERFACE_CLAIMS_HPP_ */
