#include <rclcpp/rclcpp.hpp>
#include "hikrobot_camera/camera_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  
  // 【修改点】移除了 automatically_declare_parameters_from_overrides(true)
  // 这样 Launch 传入的参数会作为 override，在 declare_parameter 时自动生效，避免重复声明报错。

  auto node = std::make_shared<hikrobot_camera::CameraNode>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}