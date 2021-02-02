
#include "cup_node.h"

#include <utility>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  // TODO read from exe agrs.
  geometry_msgs::msg::Point point;
  point.x = 0;  // should be given
  point.y = 0;  // should be given
  point.z = 0;

  std::string frame_id = "world_link";
  // end

  if (argc > 0)  // ros2 run cup cup_node --ros-args -- <arg>
    RCLCPP_INFO(rclcpp::get_logger("test"), argv[3]);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CupNode>(frame_id, point));
  rclcpp::shutdown();
  return 0;
}

CupNode::CupNode(std::string frame, Point_msg aPoint)
  : Node("cup")
  , buffer_(get_clock())
  , trans_listener_(buffer_)
  , trans_broadcaster_(this)
  , header_frame_id_(std::move(frame))
{
  initMessages(aPoint);

  timer_ = create_wall_timer(500ms, [this] { timerCallback(); });

  marker_pub_ = create_publisher<Marker_msg>(MESSAGE_NAMESPACE + "marker", QOS_VAL);
  twist_pub_ = create_publisher<Twist_msg>(MESSAGE_NAMESPACE + "twist", QOS_VAL);
  pose_pub_ = create_publisher<Pose_msg>(MESSAGE_NAMESPACE + "pose", QOS_VAL);
}

void CupNode::timerCallback()
{
  publishMarker();
  publishTwist();
  publishPose();
}

void CupNode::publishMarker()
{
  setCupColor(marker_msg_.color);
  setCupPose(marker_msg_.pose);

  marker_pub_->publish(marker_msg_);
}

void CupNode::publishTwist()
{
}
void CupNode::publishPose()
{
}
void CupNode::setCupColor(CupNode::Color_msg& msg)
{
  const std::vector<std::vector<float>> COLORS{ { 1.0f, 0.0f, 0.0f, 1.0f },
                                                { 1.0f, 0.0f, 0.0f, 1.0f },
                                                { 1.0f, 0.0f, 0.0f, 1.0f } };
  msg.r = COLORS.at(current_state_).at(0);
  msg.g = COLORS.at(current_state_).at(1);
  msg.b = COLORS.at(current_state_).at(2);
  msg.a = COLORS.at(current_state_).at(3);
}
void CupNode::setCupPose(CupNode::Pose_msg& msg)
{
  msg.position.x += 0.001;  // TODO
}

void CupNode::initMessages(CupNode::Point_msg init_point)
{
  initPose(init_point);
  initTransform();
  initMarker();
}

void CupNode::initPose(CupNode::Point_msg init_point)
{
  // create pose message
  pose_msg_ = Pose_msg();

  // set position to given position
  pose_msg_.position = init_point;

  // set orientation
  pose_msg_.orientation.x = 0;
  pose_msg_.orientation.y = 0;
  pose_msg_.orientation.z = 0;
  pose_msg_.orientation.w = 1;
}

void CupNode::initTransform()
{
  // create trans stamped message
  trans_stamped_msg_ = Trans_stamped_msg();

  // set header values
  trans_stamped_msg_.header.stamp = now();
  trans_stamped_msg_.header.frame_id = header_frame_id_;

  // set child frame
  trans_stamped_msg_.child_frame_id = FRAME_ID;

  // set rotation
  trans_stamped_msg_.transform.rotation = pose_msg_.orientation;

  // set translation
  trans_stamped_msg_.transform.translation.x = pose_msg_.position.x;
  trans_stamped_msg_.transform.translation.y = pose_msg_.position.y;
  trans_stamped_msg_.transform.translation.z = pose_msg_.position.z;
}

void CupNode::initMarker()
{
  // create marker message
  marker_msg_ = Marker_msg();

  // set header values
  marker_msg_.header.stamp = now();
  marker_msg_.header.frame_id = FRAME_ID;

  // set resource
  marker_msg_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker_msg_.action = visualization_msgs::msg::Marker::ADD;
  marker_msg_.mesh_resource = RESOURCE_PATH;

  // set scale
  marker_msg_.scale.x = marker_msg_.scale.y = marker_msg_.scale.z = 0.001;

  // set misc
  marker_msg_.ns = "/cup";
  marker_msg_.id = 0;

  // set color and pose
  setCupColor(marker_msg_.color);
  setCupPose(marker_msg_.pose);

  // set lifetime
  marker_msg_.lifetime = builtin_interfaces::msg::Duration();
}
