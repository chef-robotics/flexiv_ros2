/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <vector>

// ROS
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// ros2_control hardware_interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// Flexiv
#include "flexiv/rdk/robot.hpp"
#include "flexiv/rdk/tool.hpp"

namespace flexiv_hardware {

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT
};

/**
 * Maximum number of RDK-commandable joint groups this interface can drive: an optional external
 * axis group plus up to two single arms (EXT_AXIS, ARM_1, ARM_2).
 */
constexpr size_t kMaxJointGroups = 3;

/** Command interfaces of an arm's `<arm prefix>payload`, in flexiv::rdk::ToolParams order: mass,
 * CoM in the flange frame, then inertia at the CoM. */
constexpr std::array<const char*, 10> kPayloadInterfaceNames
    = {"mass", "com_x", "com_y", "com_z", "ixx", "iyy", "izz", "ixy", "ixz", "iyz"};

using PayloadValues = std::array<double, kPayloadInterfaceNames.size()>;

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

class FlexivHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;

    // RDK control mode for joint position and velocity interfaces
    flexiv::rdk::Mode rdk_control_mode_;

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Reused write-loop buffers to avoid per-cycle allocations.
    std::vector<double> target_pos_buffer_;
    std::vector<double> target_vel_buffer_;
    std::vector<double> target_torque_buffer_;

    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointPositionCmd> rt_joint_position_cmds_;
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointTorqueCmd> rt_joint_torque_cmds_;

    // Robot states exported per active joint group.
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RobotStates> hw_flexiv_robot_states_by_group_;
    std::map<flexiv::rdk::JointGroup, double> hw_flexiv_robot_state_handles_by_group_;

    // Payload commands per arm, parallel to arm_prefixes_; NaN until first commanded.
    std::vector<PayloadValues> hw_commands_payloads_;

    // Payload last sent to the robot per arm, so one the robot rejects is not retried at every
    // rest. NaN until the first attempt.
    std::vector<PayloadValues> requested_payloads_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;

    // Whole-robot health, exported under `robot_health`. A state interface
    // carries only a double, so the status enum is cast here and back downstream.
    double hw_states_operational_status_ = 0.0;
    double hw_states_connected_ = 0.0;
    double hw_states_operational_ = 0.0;
    double hw_states_estop_released_ = 0.0;

    // Joint name prefix of each arm, in RDK order: arm i is joint group ARM_<i+1>.
    std::vector<std::string> arm_prefixes_;

    // Map from RDK joint index to ROS joint index
    // RDK expects: [ext_axis_1, ..., ext_axis_N, arm_joint_1, ..., arm_joint_7]
    std::vector<size_t> rdk_to_ros_map_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static const rclcpp::Logger& getLogger();

    // Clock for the throttled logging macros on error paths.
    rclcpp::Clock log_clock_ {RCL_STEADY_TIME};

    std::unique_ptr<flexiv::rdk::Tool> tool_;

    /** How long every arm must be at rest before a payload update may stop the robot. */
    static constexpr double kPayloadRestSettleTimeSec = 0.5;

    /** Largest gap between a position command and the measured position that a group resuming
     * after a payload update streams; past it the group holds instead. */
    static constexpr double kResumeGuardToleranceRad = 1e-3;

    /** Seconds since the epoch of `write()`'s clock at which the robot came to rest, or NaN
     * when it is not at rest. */
    double rest_start_sec_;

    std::vector<double> previous_commands_joint_positions_;

    /** Stream failures write() tolerates straight after a payload update's SwitchMode(), rather
     * than erroring the component. Believed to clear within a few cycles; unverified. */
    static constexpr size_t kPayloadResumeGraceCycles = 50;

    /** Tolerated stream failures left, reset by the first successful stream. */
    size_t payload_resume_grace_cycles_ {0};

    /** Per joint group, parallel to active_groups_: whether it is holding until its position
     * command rejoins the measured position after a payload update. */
    std::array<bool, kMaxJointGroups> resume_guarded_ {};

    /** Whether any arm's payload command differs from the one last sent to the robot. */
    bool has_pending_payload() const;

    /** Advance rest_start_sec_: at rest while the robot reports every joint group stopped and no
     * position command changes. */
    void update_rest_start(const rclcpp::Time& time);

    /**
     * Stop the robot, send every pending payload, and restore required_mode.
     *
     * Both arms stop, because the RDK accepts tool changes only in IDLE and the control mode is
     * robot-wide; write() calls this only once every arm has been at rest for
     * kPayloadRestSettleTimeSec.
     * @return ERROR when the robot cannot be stopped or put back in required_mode. A payload the
     *         robot rejects is logged and skipped, since the robot then keeps its previous one.
     */
    hardware_interface::return_type apply_pending_payloads(flexiv::rdk::Mode required_mode);

    /**
     * Resolve which joint groups a set of command interface names fully claims, and with which
     * interface type.
     * @param[in] keys Command interface names to resolve.
     * @param[out] claimed Per joint group, the claimed CommandInterfaceType
     * @return False when a joint group would be only partially claimed, or claimed with more than
     *         one interface type at once.
     */
    bool resolve_claimed_groups(
        const std::vector<std::string>& keys, std::array<uint8_t, kMaxJointGroups>& claimed) const;

    /** RDK control mode implied by the interfaces currently claimed on each joint group.
     * Mode::UNKNOWN when no joint group is claimed at all. */
    flexiv::rdk::Mode required_rdk_mode() const;

    // Active RDK joint groups and their DoF, ordered [EXT_AXIS, ARM_1, ARM_2] to match
    // rdk_to_ros_map_.
    std::vector<std::pair<flexiv::rdk::JointGroup, size_t>> active_groups_;

    // CommandInterfaceType currently claimed on each joint group, parallel to active_groups_.
    std::array<uint8_t, kMaxJointGroups> claimed_interfaces_;

    // Control modes
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */
