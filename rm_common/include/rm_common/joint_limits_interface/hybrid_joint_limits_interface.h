//
// Created by wk on 2026/1/3.
//

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include <ros/duration.h>

#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>

#include <rm_common/hardware_interface/hybrid_joint_interface.h>

namespace rm_control
{
class HybridJointSaturationHandle
{
public:
  HybridJointSaturationHandle(const rm_control::HybridJointHandle& hjh,
                              const joint_limits_interface::JointLimits& limits)
    : hjh_(hjh), limits_(limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                                                  "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                                                  "'. It has no efforts limits specification.");
    }

    if (limits_.has_position_limits)
    {
      min_pos_limit_ = limits_.min_position;
      max_pos_limit_ = limits_.max_position;
    }
    else
    {
      min_pos_limit_ = -std::numeric_limits<double>::max();
      max_pos_limit_ = std::numeric_limits<double>::max();
    }
  }

  /** \return Joint name. */
  std::string getName() const
  {
    return hjh_.getName();
  }

  /**
   * \brief Enforce position, velocity, and effort limits for a joint that is not subject to soft limits.
   */
  void enforceLimits(const ros::Duration& period)
  {
    using joint_limits_interface::internal::saturate;
    double min_eff = -limits_.max_effort;
    double max_eff = limits_.max_effort;

    if (limits_.has_position_limits)
    {
      const double pos = hjh_.getPosition();
      if (pos < limits_.min_position)
        min_eff = 0.0;
      else if (pos > limits_.max_position)
        max_eff = 0.0;
    }

    const double vel = hjh_.getVelocity();
    if (vel < -limits_.max_velocity)
      min_eff = 0.0;
    else if (vel > limits_.max_velocity)
      max_eff = 0.0;

    if (std::isnan(prev_pos_cmd_))
      prev_pos_cmd_ = hjh_.getPosition();

    double min_pos, max_pos;
    if (limits_.has_velocity_limits)
    {
      const double delta_pos = limits_.max_velocity * period.toSec();
      min_pos = std::max(prev_pos_cmd_ - delta_pos, min_pos_limit_);
      max_pos = std::min(prev_pos_cmd_ + delta_pos, max_pos_limit_);
    }
    else
    {
      min_pos = min_pos_limit_;
      max_pos = max_pos_limit_;
    }

    const double pos_cmd = saturate(hjh_.getPositionDesired(), min_pos, max_pos);
    prev_pos_cmd_ = pos_cmd;

    // Velocity bounds
    double vel_low{};
    double vel_high{};

    if (limits_.has_acceleration_limits)
    {
      assert(period.toSec() > 0.0);
      const double dt = period.toSec();

      vel_low = std::max(prev_vel_cmd_ - limits_.max_acceleration * dt, -limits_.max_velocity);
      vel_high = std::min(prev_vel_cmd_ + limits_.max_acceleration * dt, limits_.max_velocity);
    }
    else
    {
      vel_low = -limits_.max_velocity;
      vel_high = limits_.max_velocity;
    }

    // Saturate velocity command according to limits
    const double vel_cmd = saturate(hjh_.getVelocityDesired(), vel_low, vel_high);
    prev_vel_cmd_ = vel_cmd;

    hjh_.setCommand(pos_cmd, vel_cmd, hjh_.getKp(), hjh_.getKd(), saturate(hjh_.getFeedforward(), min_eff, max_eff));
  }

  /**
   * \brief Reset state, in case of mode switch or e-stop
   */
  void reset()
  {
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

private:
  rm_control::HybridJointHandle hjh_;
  joint_limits_interface::JointLimits limits_;

  double min_pos_limit_, max_pos_limit_;

  double prev_pos_cmd_ = { std::numeric_limits<double>::quiet_NaN() };
  double prev_vel_cmd_ = { std::numeric_limits<double>::quiet_NaN() };
};

class HybridJointSoftLimitsHandle
{
public:
  HybridJointSoftLimitsHandle() = default;

  HybridJointSoftLimitsHandle(const rm_control::HybridJointHandle& hjh,
                              const joint_limits_interface::JointLimits& limits,
                              const joint_limits_interface::SoftJointLimits& soft_limits)
    : hjh_(hjh), limits_(limits), soft_limits_(soft_limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                                                  "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits)
    {
      throw joint_limits_interface::JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                                                  "'. It has no effort limits specification.");
    }
  }

  /** \return Joint name. */
  std::string getName() const
  {
    return hjh_.getName();
  }

  /**
   * \brief Enforce position, velocity and effort limits for a joint subject to soft limits.
   *
   * If the joint has no position limits (eg. a continuous joint), only velocity and effort limits will be enforced.
   */
  void enforceLimits(const ros::Duration& period)
  {
    using joint_limits_interface::internal::saturate;

    // Effort Interface limits
    // Current state
    double pos = hjh_.getPosition();
    double vel = hjh_.getVelocity();

    // Velocity bounds
    double soft_min_vel{};
    double soft_max_vel{};

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position), -limits_.max_velocity,
                              limits_.max_velocity);

      soft_max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position), -limits_.max_velocity,
                              limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel = limits_.max_velocity;
    }

    // Effort bounds depend on the velocity and effort bounds
    const double soft_min_eff =
        saturate(-soft_limits_.k_velocity * (vel - soft_min_vel), -limits_.max_effort, limits_.max_effort);

    const double soft_max_eff =
        saturate(-soft_limits_.k_velocity * (vel - soft_max_vel), -limits_.max_effort, limits_.max_effort);

    // Saturate effort command according to bounds
    const double eff_cmd = saturate(hjh_.getFeedforward(), soft_min_eff, soft_max_eff);

    assert(period.toSec() > 0.0);
    // Current position
    if (std::isnan(prev_pos_cmd_))
    {
      prev_pos_cmd_ = hjh_.getPosition();
    }  // Happens only once at initialization
    pos = prev_pos_cmd_;

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position), -limits_.max_velocity,
                              limits_.max_velocity);

      soft_max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position), -limits_.max_velocity,
                              limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel = limits_.max_velocity;
    }

    // Position Interface limits
    // Position bounds
    const double dt = period.toSec();
    double pos_low = pos + soft_min_vel * dt;
    double pos_high = pos + soft_max_vel * dt;

    if (limits_.has_position_limits)
    {
      // This extra measure safeguards against pathological cases, like when the soft limit lies beyond the hard limit
      pos_low = std::max(pos_low, limits_.min_position);
      pos_high = std::min(pos_high, limits_.max_position);
    }

    // Saturate position command according to bounds
    const double pos_cmd = saturate(hjh_.getPositionDesired(), pos_low, pos_high);

    // Cache variables
    prev_pos_cmd_ = pos_cmd;

    // Velocity Interface limits
    double min_vel{}, max_vel{};
    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit.
      pos = hjh_.getPosition();
      min_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position), -max_vel_limit_, max_vel_limit_);
      max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position), -max_vel_limit_, max_vel_limit_);
    }
    else
    {
      min_vel = -max_vel_limit_;
      max_vel = max_vel_limit_;
    }

    if (limits_.has_acceleration_limits)
    {
      vel = hjh_.getVelocity();
      const double delta_t = period.toSec();
      min_vel = std::max(vel - limits_.max_acceleration * delta_t, min_vel);
      max_vel = std::min(vel + limits_.max_acceleration * delta_t, max_vel);
    }

    hjh_.setCommand(pos_cmd, saturate(hjh_.getVelocityDesired(), min_vel, max_vel), hjh_.getKp(), hjh_.getKd(), eff_cmd);
  }

  /**
   * \brief Reset state, in case of mode switch or e-stop
   */
  void reset()
  {
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

private:
  rm_control::HybridJointHandle hjh_;
  joint_limits_interface::JointLimits limits_;
  joint_limits_interface::SoftJointLimits soft_limits_;

  double prev_pos_cmd_ = { std::numeric_limits<double>::quiet_NaN() };
  double max_vel_limit_{};
};

/** Interface for enforcing limits on an effort-controlled joint through saturation. */
class HybridJointSaturationInterface : public joint_limits_interface::JointLimitsInterface<HybridJointSaturationHandle>
{
public:
  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Reset all managed handles. */
  void reset()
  {
    for (auto&& resource_name_and_handle : this->resource_map_)
    {
      resource_name_and_handle.second.reset();
    }
  }
};
class HybridJointSoftLimitsInterface : public joint_limits_interface::JointLimitsInterface<HybridJointSoftLimitsHandle>
{
public:
  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Reset all managed handles. */
  void reset()
  {
    for (auto&& resource_name_and_handle : this->resource_map_)
    {
      resource_name_and_handle.second.reset();
    }
  }
};
}  // namespace rm_control
