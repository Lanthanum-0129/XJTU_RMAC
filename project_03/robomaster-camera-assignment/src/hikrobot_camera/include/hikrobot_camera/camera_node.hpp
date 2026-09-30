#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include "MvCameraControl.h"

#include <string>
#include <thread>
#include <atomic>
#include <vector>

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options);
  ~CameraNode();

private:
  // ROS 2 核心组件
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // MVS SDK 组件
  void * handle_;
  std::thread grab_thread_;
  std::atomic<bool> is_running_;
  std::atomic<bool> is_connected_;

  // 参数变量
  std::string camera_ip_;
  std::string camera_sn_;
  std::string image_topic_;
  double exposure_time_;
  double gain_;
  double frame_rate_;
  std::string pixel_format_;
  bool exposure_auto_;
  bool gain_auto_;

  // 核心方法
  void declare_parameters();
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
  
  bool init_camera();
  bool apply_camera_params();
  void start_grabbing();
  void stop_grabbing();
  void grab_loop();
  void reconnect();
  
  std::string get_ros_encoding(const std::string & mv_pixel_format);
};

} // namespace hikrobot_camera

#endif // HIKROBOT_CAMERA__CAMERA_NODE_HPP_