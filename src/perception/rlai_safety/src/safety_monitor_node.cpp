// Watches the forward LiDAR sector and reports whether the path is blocked.
// Nav2 keeps owning obstacle avoidance; this node only raises a risk flag that
// the search manager can use to pause or cancel the current navigation task.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rlai_interfaces/msg/safety_status.hpp>

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor")
  {
    declare_parameter<double>("safety_distance", 0.35);
    declare_parameter<double>("half_angle_deg", 30.0);
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("status_topic", "/safety_status");

    safety_distance_ = get_parameter("safety_distance").as_double();
    half_angle_rad_ = get_parameter("half_angle_deg").as_double() * M_PI / 180.0;

    const auto scan_topic = get_parameter("scan_topic").as_string();
    const auto status_topic = get_parameter("status_topic").as_string();

    publisher_ = create_publisher<rlai_interfaces::msg::SafetyStatus>(status_topic, 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SafetyMonitorNode::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "safety_monitor started — scan=%s status=%s distance=%.2f m sector=+/-%.0f deg",
      scan_topic.c_str(),
      status_topic.c_str(),
      safety_distance_,
      half_angle_rad_ * 180.0 / M_PI);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float closest = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
      if (std::fabs(angle) > half_angle_rad_) {
        continue;
      }

      const float range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min + 1e-6F) {
        continue;
      }
      if (range > msg->range_max) {
        continue;
      }
      closest = std::min(closest, range);
    }

    rlai_interfaces::msg::SafetyStatus status;
    status.header = msg->header;

    const bool has_reading = std::isfinite(closest);
    status.min_distance = has_reading ? closest : msg->range_max;
    status.blocked = has_reading && closest < static_cast<float>(safety_distance_);

    std::ostringstream reason;
    if (status.blocked) {
      reason.setf(std::ios::fixed);
      reason.precision(2);
      reason << "forward sector " << closest << " m";
    } else {
      reason << "forward sector clear";
    }
    status.reason = reason.str();

    publisher_->publish(status);
  }

  double safety_distance_{0.35};
  double half_angle_rad_{M_PI / 6.0};

  rclcpp::Publisher<rlai_interfaces::msg::SafetyStatus>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
