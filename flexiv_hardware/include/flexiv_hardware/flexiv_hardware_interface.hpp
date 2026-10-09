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

#include "flexiv_hardware/command_interface_claims.hpp"
#include "flexiv_hardware/first_order_low_pass.hpp"

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
    std::vector<double> hw_commands_joint_accelerations_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // The q_d, dq_d and ddq_d last prepared for the robot, exported as state interfaces.
    std::vector<double> hw_states_target_positions_;
    std::vector<double> hw_states_target_velocities_;
    std::vector<double> hw_states_target_accelerations_;

    // Reused write-loop buffers to avoid per-cycle allocations.
    std::vector<double> target_pos_buffer_;
    std::vector<double> target_vel_buffer_;
    std::vector<double> target_acc_buffer_;
    std::vector<double> target_torque_buffer_;

    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointPositionCmd> rt_joint_position_cmds_;
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointTorqueCmd> rt_joint_torque_cmds_;

    /** Low-pass time constant [s] on a trajectory claim's feedforward; hardware parameter
     * `feedforward_time_constant`, 0 disables it. */
    static constexpr double kDefaultFeedforwardTimeConstant = 0.02;
    double feedforward_time_constant_ {kDefaultFeedforwardTimeConstant};
    std::vector<FirstOrderLowPass> feedforward_velocity_filters_;
    std::vector<FirstOrderLowPass> feedforward_acceleration_filters_;

    /** Default for the hardware parameter `feedforward_max_acceleration` [rad/s^2]. */
    static constexpr double kDefaultFeedforwardMaxAcceleration = 15.0;
    // Per-joint bounds on a trajectory claim's feedforward, in RDK order: the robot's dq_max
    // [rad/s] and `feedforward_max_acceleration` [rad/s^2].
    std::vector<double> feedforward_velocity_limits_;
    std::vector<double> feedforward_acceleration_limits_;

    /** SCHED_FIFO priority of the RDK's transport threads; hardware parameter
     * `rdk_thread_priority`, 0 leaves them on the default policy. */
    static constexpr int kDefaultRdkThreadPriority = 49;
    int rdk_thread_priority_ {kDefaultRdkThreadPriority};

    // Robot states exported per active joint group.
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RobotStates> hw_flexiv_robot_states_by_group_;
    std::map<flexiv::rdk::JointGroup, double> hw_flexiv_robot_state_handles_by_group_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;

    // Whole-robot health, exported under `robot_health`. A state interface
    // carries only a double, so the status enum is cast here and back downstream.
    double hw_states_operational_status_ = 0.0;
    double hw_states_connected_ = 0.0;
    double hw_states_operational_ = 0.0;
    double hw_states_estop_released_ = 0.0;

    // Map from RDK joint index to ROS joint index
    // RDK expects: [ext_axis_1, ..., ext_axis_N, arm_joint_1, ..., arm_joint_7]
    std::vector<size_t> rdk_to_ros_map_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static const rclcpp::Logger& getLogger();

    // Clock for the throttled logging macros on error paths.
    rclcpp::Clock log_clock_ {RCL_STEADY_TIME};

    /** Reset by the first write() cycle that streams successfully. */
    size_t consecutive_stream_failures_ {0};

    /** Stream failures tolerated before write() errors the hardware component. */
    static constexpr size_t kMaxConsecutiveStreamFailures = 50;

    /** Reset by the first write() cycle that finds the control mode already correct. */
    size_t consecutive_mode_recoveries_ {0};

    /** Mode-recovery attempts before write() gives up and errors the component. */
    static constexpr size_t kMaxConsecutiveModeRecoveries = 3;

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

    /** Whether joint `ros_idx` has every command its group's `claim` drives it with. */
    bool is_joint_commanded(uint8_t claim, size_t ros_idx) const;

    /** The q_d [rad], dq_d [rad/s] and ddq_d [rad/s^2] streamed for one joint. */
    struct JointTarget
    {
        double position;
        double velocity;
        double acceleration;
    };

    /** Target of RDK joint `rdk_idx` from the commands of its group's position, velocity or
     * trajectory `claim`, advancing the joint's feedforward filters with no feedforward input
     * while the group `is_holding`. */
    JointTarget commanded_target(
        uint8_t claim, size_t rdk_idx, bool is_holding, double feedforward_gain);

    /**
     * Clamp the feedforward of a joint target to the robot's limits, logging when it clamps.
     * @param[in] rdk_idx RDK index of the joint, selecting its limits.
     * @param[in] target The joint's target, with the feedforward to clamp.
     * @return `target` with its velocity and acceleration clamped to the joint's
     *         feedforward_velocity_limits_ and feedforward_acceleration_limits_; the position is
     *         unchanged.
     */
    JointTarget clamp_feedforward(size_t rdk_idx, const JointTarget& target);

    /** Drop every feedforward filter's state, so the next command seeds it. */
    void reset_feedforward_filters();

    /** Put the RDK's transport threads on SCHED_FIFO at rdk_thread_priority_, unless it is 0. */
    void apply_rdk_thread_priority();

    /** Handle a write() cycle whose commands are skipped because the robot is not operational,
     * e.g. E-stopped: log it, and drop the hold targets so they re-seed from the measured position
     * once it is operational again. Digital outputs are deferred until then too. */
    void on_not_operational();

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
