#ifndef FLEXIV_HARDWARE__COMMAND_INTERFACE_CLAIMS_HPP_
#define FLEXIV_HARDWARE__COMMAND_INTERFACE_CLAIMS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

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
};

/** The command interfaces named for one joint, as a bit set. */
enum CommandInterfaceBits : uint8_t
{
    kBitPosition = 1 << 0,
    kBitVelocity = 1 << 1,
    kBitEffort = 1 << 3,
};

/** The command interfaces in `keys` that belong to `joint_name`, as CommandInterfaceBits. */
inline uint8_t claimed_interface_bits(
    const std::vector<std::string>& keys, const std::string& joint_name)
{
    uint8_t bits = 0;
    for (const auto& key : keys) {
        if (key == joint_name + "/" + hardware_interface::HW_IF_POSITION) {
            bits |= kBitPosition;
        } else if (key == joint_name + "/" + hardware_interface::HW_IF_VELOCITY) {
            bits |= kBitVelocity;
        } else if (key == joint_name + "/" + hardware_interface::HW_IF_EFFORT) {
            bits |= kBitEffort;
        }
    }
    return bits;
}

/**
 * The CommandInterfaceType a joint claimed with exactly `bits` is driven by, or kInterfaceNone
 * when `bits` is empty or a combination this hardware interface does not drive.
 */
inline CommandInterfaceType interface_type_from_bits(uint8_t bits)
{
    switch (bits) {
        case kBitPosition:
            return kInterfacePosition;
        case kBitVelocity:
            return kInterfaceVelocity;
        case kBitEffort:
            return kInterfaceEffort;
        default:
            return kInterfaceNone;
    }
}

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
inline GroupClaim resolve_group_claim(
    const std::vector<std::string>& keys, const std::vector<std::string>& joint_names)
{
    GroupClaim claim;
    for (const auto& joint_name : joint_names) {
        const uint8_t bits = claimed_interface_bits(keys, joint_name);
        if (bits == 0) {
            continue;
        }
        const CommandInterfaceType joint_type = interface_type_from_bits(bits);
        if (joint_type == kInterfaceNone
            || (claim.type != kInterfaceNone && joint_type != claim.type)) {
            claim.error = GroupClaimError::kMixedTypes;
            return claim;
        }
        claim.type = joint_type;
        claim.claimed_joints++;
    }
    if (claim.claimed_joints != 0 && claim.claimed_joints != joint_names.size()) {
        claim.error = GroupClaimError::kPartialGroup;
    }
    return claim;
}

/** Interface a joint group ends up claimed with, given what a mode switch starts and stops. */
inline uint8_t next_claimed_interface(uint8_t current, uint8_t starting, uint8_t stopping)
{
    if (starting != kInterfaceNone) {
        return starting;
    }
    if (stopping != kInterfaceNone) {
        return kInterfaceNone;
    }
    return current;
}

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__COMMAND_INTERFACE_CLAIMS_HPP_ */
