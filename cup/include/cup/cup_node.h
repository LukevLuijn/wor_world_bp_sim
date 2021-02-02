//
// Created by luke on 02-02-21.
//

#ifndef CLIONPROJECT_CUP_NODE_H
#define CLIONPROJECT_CUP_NODE_H
/** CUP REQUIREMENTS
 *
 * TODO random location.
 *  >   pose position arg in main.
 *
 * TODO publish 3D object.
 *  >   maker and some stuff, all done.
 *
 * TODO detect hit fields of gripper.
 *  >   arm publish gripper point, cup -> read.
 *
 * TODO visual effect when gripper holds cup.
 *  >   change color of cup
 *
 * TODO move with gripper (when held).
 *  >   arm publish gripper loc, cup -> read.
 *
 * TODO fall when released.
 *  >   fall straight down with speed.
 *
 * TODO calc location and publish this.
 *  >   easy
 *
 * TODO calc velocity and publish this.
 *  >   hard
 *
 * TODO something with RQT plot.
 *  >   no idea
 *
 */

/// ROS HEADERS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

/// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/// MESSAGES
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace States
{
enum CupState
{
  MOVING = 0,
  FALLING = 1,
  STILL = 2
};
}

class CupNode : public rclcpp::Node
{
  typedef States::CupState State;

  typedef visualization_msgs::msg::Marker Marker_msg;

  typedef geometry_msgs::msg::TransformStamped Trans_stamped_msg;
  typedef geometry_msgs::msg::Quaternion Quaternion_msg;
  typedef geometry_msgs::msg::Twist Twist_msg;
  typedef geometry_msgs::msg::Pose Pose_msg;
  typedef geometry_msgs::msg::Point Point_msg;

  typedef std_msgs::msg::ColorRGBA Color_msg;

public:
  explicit CupNode(std::string frame, Point_msg aPoint);
  ~CupNode() override = default;

private:
  void timerCallback();

  void publishMarker();
  void publishPose();
  void publishTwist();

  void setCupColor(Color_msg &msg);
  void setCupPose(Pose_msg &msg);

private:  /// initial setup
  void initMessages(Point_msg init_point);
  void initPose(Point_msg init_point);
  void initTransform();
  void initMarker();

private:  /// fields
  State current_state_ = State::STILL;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener trans_listener_;
  tf2_ros::TransformBroadcaster trans_broadcaster_;

  rclcpp::Publisher<Marker_msg>::SharedPtr marker_pub_;
  rclcpp::Publisher<Twist_msg>::SharedPtr twist_pub_;
  rclcpp::Publisher<Pose_msg>::SharedPtr pose_pub_;

  Trans_stamped_msg trans_stamped_msg_;
  Marker_msg marker_msg_;
  Twist_msg twist_msg_;
  Pose_msg pose_msg_;

  std::string header_frame_id_ = "n/a";

private:  /// const fields
  const rclcpp::QoS QOS_VAL = 10;
  const std::string MESSAGE_NAMESPACE = "simulation/cup/";
  const std::string FRAME_ID = "cup_link";
  const std::string RESOURCE_PATH = "package://cup/resource/cup.stl";  // TODO
};

#endif  // CLIONPROJECT_CUP_NODE_H
