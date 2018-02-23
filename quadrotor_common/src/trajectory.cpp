#include "quadrotor_common/trajectory.h"

#include <Eigen/Dense>

#include "quadrotor_common/math_common.h"

namespace quadrotor_common
{

Trajectory::Trajectory() :
    timestamp(ros::Time::now()), trajectory_type(TrajectoryType::UNDEFINED), points()
{
}

Trajectory::Trajectory(const quadrotor_msgs::Trajectory& trajectory_msg)
{
  timestamp = trajectory_msg.header.stamp;

  switch (trajectory_msg.type)
  {
    case trajectory_msg.GENERAL:
      trajectory_type = TrajectoryType::GENERAL;
      break;
    case trajectory_msg.ACCELERATION:
      trajectory_type = TrajectoryType::ACCELERATION;
      break;
    case trajectory_msg.JERK:
      trajectory_type = TrajectoryType::JERK;
      break;
    case trajectory_msg.SNAP:
      trajectory_type = TrajectoryType::SNAP;
      break;
    default:
      trajectory_type = TrajectoryType::UNDEFINED;
      break;
  }

  for (int i = 0; i < trajectory_msg.points.size(); i++)
  {
    points.push_back(
        quadrotor_common::TrajectoryPoint(trajectory_msg.points[i]));
  }
}

Trajectory::~Trajectory()
{
}

quadrotor_msgs::Trajectory Trajectory::toRosMessage() const
{
  quadrotor_msgs::Trajectory ros_msg;

  ros_msg.header.stamp = timestamp;

  switch (trajectory_type)
  {
    case TrajectoryType::UNDEFINED:
      ros_msg.type = ros_msg.UNDEFINED;
      break;
    case TrajectoryType::GENERAL:
      ros_msg.type = ros_msg.GENERAL;
      break;
    case TrajectoryType::ACCELERATION:
      ros_msg.type = ros_msg.ACCELERATION;
      break;
    case TrajectoryType::JERK:
      ros_msg.type = ros_msg.JERK;
      break;
    case TrajectoryType::SNAP:
      ros_msg.type = ros_msg.SNAP;
      break;
  }

  std::list<quadrotor_common::TrajectoryPoint>::iterator it;
  for (it = points.begin(); it != points.end(); it++)
  {
    ros_msg.points.push_back(it->toRosMessage());
  }

  return ros_msg;
}

quadrotor_common::TrajectoryPoint Trajectory::getStateAtTime(
    const ros::Duration& time_from_start) const
{
  if (time_from_start <= points.front().time_from_start)
  {
    return points.front();
  }
  if (time_from_start >= points.back().time_from_start)
  {
    return points.back();
  }

  quadrotor_common::TrajectoryPoint trajectory_point;

  // Find points p0 and p1 such that
  // p0.time_from_start <= time_from_start <= p1.time_from_start
  std::list<quadrotor_common::TrajectoryPoint>::iterator p1;
  for (p1 = points.begin(); p1 != points.end(); p1++)
  {
    if (p1->time_from_start > time_from_start)
    {
      break;
    }
  }
  std::list<quadrotor_common::TrajectoryPoint>::iterator p0 = std::prev(p1);
  const double interp_ratio = (time_from_start - p0->time_from_start).toSec()
      / (p1->time_from_start - p0->time_from_start).toSec();

  return interpolate(*p0, *p1, interp_ratio);
}

} // namespace quadrotor_common
