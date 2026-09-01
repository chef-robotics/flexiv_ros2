// Compatibility shims over RDK API differences across supported versions.
//
// TODO: Delete this header once the RDK pin returns to v2.2, and change every
// `flexiv_hardware::compat::` call site back to `flexiv::rdk::`. The shim
// exists only because chef pins v2.1 to match its robot's `v3E.1` software;
// it buys nothing once v2.1 is no longer supported. To revert:
//
//   1. delete this file
//   2. `flexiv_hardware::compat::JointGroupNames()` -> `flexiv::rdk::JointGroupNames()`
//      and `...::OpStatusNames()` -> `flexiv::rdk::OperationalStatusNames()`
//      (call sites in `flexiv_hardware_interface.cpp` and
//      `flexiv_gripper/src/gripper_action_server.cpp`)
//   3. drop the `#include "flexiv_hardware/rdk_compat.hpp"` lines
//   4. return `flexiv_hardware`'s `target_include_directories` to PRIVATE
//
// Chef pins the RDK to the version matching its robot's software (see
// `flexiv.humble.repos`), and that pin moves as robots are upgraded. Where an
// API changed shape but not meaning between those versions, the difference is
// absorbed here rather than at each call site, so the pin can move without
// touching driver logic.

#ifndef FLEXIV_HARDWARE__RDK_COMPAT_HPP_
#define FLEXIV_HARDWARE__RDK_COMPAT_HPP_

#include <map>
#include <string>

#include "flexiv/rdk/data.hpp"

// Whether the installed RDK is v2.2 or newer.
//
// The RDK exposes no version constant, so this keys on `export.hpp`, added in
// v2.2 and absent from v2.1. It has to be the preprocessor: the difference it
// selects between is a *missing namespace member*, which is a hard error rather
// than a SFINAE deduction failure, so an overload trick cannot see it.
//
// `__has_include` keeps the check inside the header, so dependent packages
// (`flexiv_gripper`) resolve the same branch without any build-system plumbing.
#if defined(__has_include)
#if __has_include("flexiv/rdk/export.hpp")
#define FLEXIV_RDK_AT_LEAST_2_2 1
#endif
#endif

namespace flexiv_hardware {
namespace compat {

// The joint-group-to-name map of the installed RDK.
//
// v2.2 replaced the inline `kJointGroupNames` map with a `JointGroupNames()`
// accessor; both hold the same contents.
inline const std::map<flexiv::rdk::JointGroup, std::string>& JointGroupNames()
{
#ifdef FLEXIV_RDK_AT_LEAST_2_2
    return flexiv::rdk::JointGroupNames();
#else
    return flexiv::rdk::kJointGroupNames;
#endif
}

// The operational-status-to-name map of the installed RDK.
//
// v2.1 exposes the inline `kOpStatusNames` map; v2.2 replaced it with an
// `OperationalStatusNames()` accessor -- a different name as well as a different
// shape, so this cannot share the `JointGroupNames` branch.
inline const std::map<flexiv::rdk::OperationalStatus, std::string>& OpStatusNames()
{
#ifdef FLEXIV_RDK_AT_LEAST_2_2
    return flexiv::rdk::OperationalStatusNames();
#else
    return flexiv::rdk::kOpStatusNames;
#endif
}

} /* namespace compat */
} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__RDK_COMPAT_HPP_ */
