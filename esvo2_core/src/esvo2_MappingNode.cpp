#include <esvo2_core/esvo2_Mapping.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, "esvo2_Mapping");
  auto node =
      std::make_shared<esvo2_core::esvo2_Mapping>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
