//
// Created by luke on 02-02-21.
//

#ifndef CLIONPROJECT_CUP_NODE_H
#define CLIONPROJECT_CUP_NODE_H

/** CUP REQUIREMENTS
 *
 * DONE random location.                        > pose position arg in main.
 * DONE publish 3D object.                      > maker and some stuff, all done.
 * TODO detect hit fields of gripper.           > arm publish gripper point, cup -> read.
 * TODO visual effect when gripper holds cup.   > change color of cup
 * TODO move with gripper (when held).          > arm publish gripper loc, cup -> read.
 * TODO fall when released.                     > fall straight down with speed.
 * DONE calc location and publish this.         > easy
 * TODO calc velocity and publish this.         > hard
 * TODO something with RQT plot.                > no idea
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace States
{
enum CupState
{
  MOVING = 0,   /// state when the cup is moving.
  FALLING = 1,  /// state when the cup is falling.
  STILL = 2     /// state when the cup is standing still.
};
}

class CupNode : public rclcpp::Node
{
  typedef States::CupState State;                                  /// typedef cupstate
  typedef visualization_msgs::msg::Marker Marker_msg;              /// typedef maker message
  typedef geometry_msgs::msg::TransformStamped Trans_stamped_msg;  /// typedef transform stamped message
  typedef geometry_msgs::msg::Twist Twist_msg;                     /// typedef twist message
  typedef geometry_msgs::msg::Point Point_msg;                     /// typedef point message
  typedef std_msgs::msg::ColorRGBA Color_msg;                      /// typedef color message
  typedef geometry_msgs::msg::Pose Pose_msg;                       /// typedef pose message

public:
  /**
   * Constructor of the CupNode class.
   *
   * @param frame   name of the global frame.
   * @param aPoint  start location of the cup.
   */
  explicit CupNode(std::string frame, Point_msg aPoint);

  /**
   * Destructor of the CupNode Class.
   */
  ~CupNode() override = default;

private:
  /**
   * Timer callback function. calls publishers and broadcaster every 10ms
   */
  void timerCallback();

  /**
   * broadcaster which broadcasts the transform. TODO
   */
  void broadcastTransform();

  /**
   * updates and publishes the marker message.
   */
  void publishMarker();

  /**
   * updates and publishes the twist message.
   */
  void publishTwist();

  /**
   * updates and publishes the pose message
   */
  void publishPose();

  /**
   * Sets color of the marker according to the current state of the cup.
   *
   * @param msg reference to color attribute of the message.
   */
  void setCupColor(Color_msg &msg);

  /**
   * Sets the pose of the marker.
   *
   * @param msg reference to pose attribute of the message.
   */
  void setCupPose(Pose_msg &msg);

  /**
   * calls the different message initializers.
   *
   * @param init_point class param containing the start location.
   */
  void initMessages(Point_msg init_point);

  /**
   * Initializer of the marker message, sets the base values.
   *
   * @param init_point class param containing the start location.
   */
  void initMarker(Point_msg init_point);

  /**
   * Initializer of the transform stamped message, sets the base values.
   */
  void initTransform();

private:
  State current_state_ = State::STILL;                   /// current state of cup
  rclcpp::TimerBase::SharedPtr timer_;                   /// wall timer (10 ms)
  tf2_ros::Buffer buffer_;                               /// tf buffer
  tf2_ros::TransformListener trans_listener_;            /// transform listener
  tf2_ros::TransformBroadcaster trans_broadcaster_;      /// transform broadcaster
  Trans_stamped_msg trans_stamped_msg_;                  /// transform stamped message
  Marker_msg marker_msg_;                                /// marker message
  Twist_msg twist_msg_;                                  /// twist message
  std::string header_frame_id_ = "n/a";                  /// parent frame id
  rclcpp::Publisher<Marker_msg>::SharedPtr marker_pub_;  /// publisher for marker message
  rclcpp::Publisher<Twist_msg>::SharedPtr twist_pub_;    /// publisher for twist message
  rclcpp::Publisher<Pose_msg>::SharedPtr pose_pub_;      /// publisher for pose message

  const rclcpp::QoS QOS_VAL = 10;                           /// Quality of service value for published messages
  const std::string MESSAGE_NAMESPACE = "simulation/cup/";  /// namespace for marker message
  const std::string FRAME_ID = "cup_link";                  /// frame id for cup
  const std::string RESOURCE_PATH = "package://cup/resource/cup.stl";  /// resource location
};

#endif  // CLIONPROJECT_CUP_NODE_H
