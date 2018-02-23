#pragma once

#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/Trajectory.h>
#include <ros/time.h>

namespace quadrotor_common
{

struct Trajectory
{
  Trajectory();
  Trajectory(const quadrotor_msgs::Trajectory& trajectory_msg);
  virtual ~Trajectory();

  quadrotor_msgs::Trajectory toRosMessage() const;
  quadrotor_common::TrajectoryPoint getStateAtTime(
      const ros::Duration& time_from_start) const;

  ros::Time timestamp;

  enum class TrajectoryType
  {
    UNDEFINED, GENERAL, ACCELERATION, JERK, SNAP
  } trajectory_type;

  std::list<quadrotor_common::TrajectoryPoint> points;
};

} // namespace quadrotor_common
