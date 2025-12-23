#include <memory>
#include <string>

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class TfSnapshotterNode : public rclcpp::Node
{
public:
  TfSnapshotterNode()
  : Node("tf_snapshotter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "target_frame");
    nav_state_topic_ = this->declare_parameter<std::string>(
      "nav_state_topic", "target_tracker/nav_state");
    detector_state_topic_ = this->declare_parameter<std::string>(
      "detector_state_topic", "state/detector");
    broadcast_rate_hz_ = this->declare_parameter<double>("broadcast_rate_hz", 10.0);

    nav_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      nav_state_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) { nav_state_ = msg->data; });
    detector_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      detector_state_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) { detector_state_ = msg->data; });

    snapshot_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "snapshot",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handle_snapshot(response);
      });

    const auto period = std::chrono::duration<double>(1.0 / broadcast_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->on_timer(); });
  }

private:
  void handle_snapshot(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (home_ready_) {
      response->success = false;
      response->message = "home already captured";
      RCLCPP_WARN(this->get_logger(), "Snapshot ignored: home already captured.");
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(1.0));
      home_transform_ = transform.transform;
      home_ready_ = true;
      response->success = true;
      response->message = "home captured";
      RCLCPP_INFO(
        this->get_logger(),
        "Home captured from %s to %s.", map_frame_.c_str(), base_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      response->success = false;
      response->message = std::string("lookup failed: ") + ex.what();
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to lookup %s->%s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
    }
  }

  void on_timer()
  {
    if (!home_ready_) {
      return;
    }
    if (nav_state_ != "finish" || detector_state_ != "FINISH") {
      return;
    }

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = target_frame_;
    msg.transform = home_transform_;
    tf_broadcaster_.sendTransform(msg);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detector_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr snapshot_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string map_frame_;
  std::string base_frame_;
  std::string target_frame_;
  std::string nav_state_topic_;
  std::string detector_state_topic_;
  std::string nav_state_;
  std::string detector_state_;
  double broadcast_rate_hz_{10.0};

  bool home_ready_{false};
  geometry_msgs::msg::Transform home_transform_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfSnapshotterNode>());
  rclcpp::shutdown();
  return 0;
}
