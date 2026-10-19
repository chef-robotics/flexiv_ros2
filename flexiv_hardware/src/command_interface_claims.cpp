#include "flexiv_hardware/command_interface_claims.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace flexiv_hardware {

uint8_t claimed_interface_bits(
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

CommandInterfaceType interface_type_from_bits(uint8_t bits)
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

GroupClaim resolve_group_claim(
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

uint8_t next_claimed_interface(uint8_t current, uint8_t starting, uint8_t stopping)
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
