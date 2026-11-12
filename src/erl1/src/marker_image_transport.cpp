#include <memory>
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

class MarkerImageTransport final : public rclcpp::Node {
public:
  MarkerImageTransport() : Node("marker_image_transport") {
    auto raw_topic = declare_parameter<std::string>("raw_topic", "/marker_image_raw");
    auto output_topic = declare_parameter<std::string>("output_topic", "/marker_image");
    publisher_ = image_transport::create_publisher(this, output_topic);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      // Reached-marker snapshots are one-shot events: request reliable delivery.
      raw_topic, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
          auto cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          publisher_.publish(*cv_image->toImageMsg());
        } catch (const cv_bridge::Exception & error) {
          RCLCPP_WARN(get_logger(), "Could not convert marker image: %s", error.what());
        }
      });
  }
private:
  image_transport::Publisher publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerImageTransport>());
  rclcpp::shutdown();
  return 0;
}
