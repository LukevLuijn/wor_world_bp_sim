
#include "cup_node.h"

#include <utility>

using namespace std::chrono_literals;

CupNode::CupNode(std::string frame, Point_msg aPoint)
  : Node("cup")
  , buffer_(get_clock())
  , trans_listener_(buffer_)
  , trans_broadcaster_(this)
  , header_frame_id_(std::move(frame))
{
  RCLCPP_DEBUG(this->get_logger(), "initialization started..");

  initMessages(aPoint);
  RCLCPP_DEBUG(this->get_logger(), "messages created");

  timer_ = create_wall_timer(10ms, [this] { timerCallback(); });

  marker_pub_ = create_publisher<Marker_msg>(MESSAGE_NAMESPACE + "marker", QOS_VAL);
  twist_pub_ = create_publisher<Twist_msg>(MESSAGE_NAMESPACE + "twist", QOS_VAL);
  pose_pub_ = create_publisher<Pose_msg>(MESSAGE_NAMESPACE + "pose", QOS_VAL);
  RCLCPP_DEBUG(this->get_logger(), "publishers created");

  /// TODO subscribers

  RCLCPP_DEBUG(this->get_logger(), "subscribers created");

  RCLCPP_DEBUG(this->get_logger(), "initialization complete");
}

void CupNode::timerCallback()
{
  broadcastTransform();
  publishMarker();
  publishTwist();
  publishPose();
}

void CupNode::broadcastTransform()
{
  // TODO check things
  try
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = buffer_.lookupTransform(header_frame_id_, FRAME_ID, rclcpp::Time(0));

    trans_stamped_msg_.transform = transformStamped.transform;
    trans_stamped_msg_.header.frame_id = header_frame_id_;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), ex.what());
  }

  trans_broadcaster_.sendTransform(trans_stamped_msg_);
}

void CupNode::publishMarker()
{
  marker_msg_.header.stamp = now();
  marker_msg_.header.frame_id = FRAME_ID;

  setCupColor(marker_msg_.color);
  setCupPose(marker_msg_.pose);
  marker_pub_->publish(marker_msg_);
}

void CupNode::publishTwist()
{
  // TODO
}
void CupNode::publishPose()
{
  setCupPose(marker_msg_.pose);
  pose_pub_->publish(marker_msg_.pose);
}
void CupNode::setCupColor(CupNode::Color_msg& msg)
{
  const std::vector<std::vector<float>> COLORS{ { 1.0f, 0.0f, 0.0f, 1.0f },    // MOVING   [RED]
                                                { 1.0f, 1.0f, 0.0f, 1.0f },    // FALLING  [YELLOW]
                                                { 0.0f, 1.0f, 0.0f, 1.0f } };  // STILL    [GREEN]
  msg.r = COLORS.at(current_state_).at(0);
  msg.g = COLORS.at(current_state_).at(1);
  msg.b = COLORS.at(current_state_).at(2);
  msg.a = COLORS.at(current_state_).at(3);
}
void CupNode::setCupPose(CupNode::Pose_msg& msg)
{
}

void CupNode::initMessages(CupNode::Point_msg init_point)
{
  initMarker(init_point);
  initTransform();
}

void CupNode::initMarker(Point_msg init_point)
{
  // create marker message
  marker_msg_ = Marker_msg();

  // set header values
  marker_msg_.header.stamp = now();
  marker_msg_.header.frame_id = FRAME_ID;

  // set position
  marker_msg_.pose.position = init_point;

  // set orientation
  marker_msg_.pose.orientation.x = 0;
  marker_msg_.pose.orientation.y = 0;
  marker_msg_.pose.orientation.z = 0;
  marker_msg_.pose.orientation.w = 1;

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

  // set lifetime
  marker_msg_.lifetime = builtin_interfaces::msg::Duration();
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
  trans_stamped_msg_.transform.rotation = marker_msg_.pose.orientation;

  // set translation
  trans_stamped_msg_.transform.translation.x = marker_msg_.pose.position.x;
  trans_stamped_msg_.transform.translation.y = marker_msg_.pose.position.y;
  trans_stamped_msg_.transform.translation.z = marker_msg_.pose.position.z;

  // initial broadcast
  trans_broadcaster_.sendTransform(trans_stamped_msg_);
}

////////
////////    MAIN
////////

signed int castToInt(const std::string& arg, const std::string& name)
{
  signed int val = 0;

  try
  {
    val = std::stoi(arg);
    std::max(val, -5);
    std::min(val, 5);
  }
  catch (std::invalid_argument& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("launch"),
                 "invalid arguments for cup location [" + name + "], location set to default.");
  }
  catch (std::logic_error& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("launch"), "logic error for cup location [" + name + "], location set to default.");
  }

  return val;
}

geometry_msgs::msg::Point parseLocation(int argc, char* argv[])
{
  geometry_msgs::msg::Point point;
  point.x = 0;  // should be given
  point.y = 0;  // should be given
  point.z = 0;

  if (argc < 5) return point;

  point.x = castToInt(argv[4], "x");
  point.y = castToInt(argv[5], "y");

  return point;
}

int main(int argc, char* argv[])
{
  std::string frame_id = (argc > 3) ? argv[3] : "world_link";
  geometry_msgs::msg::Point point = parseLocation(argc, argv);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CupNode>(frame_id, point));
  rclcpp::shutdown();
  return 0;
}