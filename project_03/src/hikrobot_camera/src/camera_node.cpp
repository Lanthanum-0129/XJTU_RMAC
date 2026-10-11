#include "hikrobot_camera/camera_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera_node", options),
  handle_(nullptr),
  is_running_(false),
  is_connected_(false)
{
  declare_parameters();
  
  if (init_camera()) {
    apply_camera_params();
    start_grabbing();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera. Will attempt to reconnect periodically.");
    reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&CameraNode::reconnect, this)
    );
  }

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CameraNode::parameters_callback, this, std::placeholders::_1));
}

CameraNode::~CameraNode()
{
  is_running_ = false;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  stop_grabbing();
  if (handle_) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  RCLCPP_INFO(this->get_logger(), "Camera node shut down and resources released.");
}

void CameraNode::declare_parameters()
{
  this->declare_parameter<std::string>("camera_ip", "");
  this->declare_parameter<std::string>("camera_sn", "");
  this->declare_parameter<std::string>("image_topic", "/image_raw");
  this->declare_parameter<double>("exposure_time", 10000.0);
  this->declare_parameter<double>("gain", 0.0);
  this->declare_parameter<double>("frame_rate", 30.0);
  this->declare_parameter<std::string>("pixel_format", "Mono8");
  this->declare_parameter<bool>("exposure_auto", false);
  this->declare_parameter<bool>("gain_auto", false);

  camera_ip_ = this->get_parameter("camera_ip").as_string();
  camera_sn_ = this->get_parameter("camera_sn").as_string();
  image_topic_ = this->get_parameter("image_topic").as_string();
  exposure_time_ = this->get_parameter("exposure_time").as_double();
  gain_ = this->get_parameter("gain").as_double();
  frame_rate_ = this->get_parameter("frame_rate").as_double();
  pixel_format_ = this->get_parameter("pixel_format").as_string();
  exposure_auto_ = this->get_parameter("exposure_auto").as_bool();
  gain_auto_ = this->get_parameter("gain_auto").as_bool();

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::QoS(10));
}

rcl_interfaces::msg::SetParametersResult CameraNode::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";

  if (!is_connected_ || !handle_) {
    result.successful = false;
    result.reason = "Camera is not connected";
    return result;
  }

  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      double val = param.as_double();
      if (val < 0.0) { result.successful = false; result.reason = "Exposure time must be >= 0"; break; }
      if (MV_CC_SetFloatValue(handle_, "ExposureTime", val) != MV_OK) {
        result.successful = false; result.reason = "SDK failed to set ExposureTime"; break;
      }
      exposure_time_ = val;
    } 
    else if (param.get_name() == "gain") {
      double val = param.as_double();
      if (val < 0.0) { result.successful = false; result.reason = "Gain must be >= 0"; break; }
      if (MV_CC_SetFloatValue(handle_, "Gain", val) != MV_OK) {
        result.successful = false; result.reason = "SDK failed to set Gain"; break;
      }
      gain_ = val;
    }
    else if (param.get_name() == "frame_rate") {
      double val = param.as_double();
      if (val <= 0.0) { result.successful = false; result.reason = "Frame rate must be > 0"; break; }
      if (MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", val) != MV_OK) {
        result.successful = false; result.reason = "SDK failed to set AcquisitionFrameRate"; break;
      }
      frame_rate_ = val;
    }
    else if (param.get_name() == "pixel_format") {
      std::string val = param.as_string();
      if (MV_CC_SetEnumValueByString(handle_, "PixelFormat", val.c_str()) != MV_OK) {
        result.successful = false; result.reason = "SDK failed to set PixelFormat"; break;
      }
      pixel_format_ = val;
    }
    else if (param.get_name() == "exposure_auto") {
      bool val = param.as_bool();
      const char* mode = val ? "Continuous" : "Off";
      if (MV_CC_SetEnumValueByString(handle_, "ExposureAuto", mode) != MV_OK) {
        result.successful = false; result.reason = "SDK failed to set ExposureAuto"; break;
      }
      exposure_auto_ = val;
    }
    else if (param.get_name() == "gain_auto") {
      bool val = param.as_bool();
      const char* mode = val ? "Continuous" : "Off";
      if (MV_CC_SetEnumValueByString(handle_, "GainAuto", mode) != MV_OK) {
        result.successful = false; result.reason = "SDK failed to set GainAuto"; break;
      }
      gain_auto_ = val;
    }
  }

  if (result.successful) {
    RCLCPP_INFO(this->get_logger(), "Parameters updated successfully.");
  } else {
    RCLCPP_WARN(this->get_logger(), "Parameter update failed: %s", result.reason.c_str());
  }
  return result;
}

bool CameraNode::init_camera()
{
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(), "No camera found.");
    return false;
  }

  int nSelectedIndex = -1;
  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
    if (!pDeviceInfo) continue;

    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
      MV_GIGE_DEVICE_INFO* pGigEInfo = &pDeviceInfo->SpecialInfo.stGigEInfo;
      std::string ip = std::to_string((pGigEInfo->nCurrentIp >> 24) & 0xFF) + "." +
                       std::to_string((pGigEInfo->nCurrentIp >> 16) & 0xFF) + "." +
                       std::to_string((pGigEInfo->nCurrentIp >> 8) & 0xFF) + "." +
                       std::to_string(pGigEInfo->nCurrentIp & 0xFF);
      std::string sn = reinterpret_cast<const char*>(pGigEInfo->chSerialNumber);

      if (!camera_ip_.empty() && ip == camera_ip_) { nSelectedIndex = i; break; }
      if (!camera_sn_.empty() && sn == camera_sn_) { nSelectedIndex = i; break; }
      if (camera_ip_.empty() && camera_sn_.empty() && nSelectedIndex == -1) { nSelectedIndex = i; }
    } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
      std::string sn = reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
      if (!camera_sn_.empty() && sn == camera_sn_) { nSelectedIndex = i; break; }
      if (camera_ip_.empty() && camera_sn_.empty() && nSelectedIndex == -1) { nSelectedIndex = i; }
    }
  }

  if (nSelectedIndex == -1) {
    RCLCPP_ERROR(this->get_logger(), "Specified camera (IP: %s, SN: %s) not found or is occupied.", camera_ip_.c_str(), camera_sn_.c_str());
    return false;
  }

  nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nSelectedIndex]);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Create handle failed: 0x%x", nRet);
    return false;
  }

  nRet = MV_CC_OpenDevice(handle_);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Open device failed: 0x%x. It might be occupied.", nRet);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  is_connected_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera connected successfully.");
  return true;
}

bool CameraNode::apply_camera_params()
{
  if (!handle_ || !is_connected_) return false;

  // 【关键修改 1】强制关闭触发模式，确保相机连续出图
  MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off");
  MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
  
  const char* exp_mode = exposure_auto_ ? "Continuous" : "Off";
  const char* gain_mode = gain_auto_ ? "Continuous" : "Off";
  MV_CC_SetEnumValueByString(handle_, "ExposureAuto", exp_mode);
  MV_CC_SetEnumValueByString(handle_, "GainAuto", gain_mode);

  if (!exposure_auto_) MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time_);
  if (!gain_auto_) MV_CC_SetFloatValue(handle_, "Gain", gain_);
  
  MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate_);
  MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  
  int nRet = MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixel_format_.c_str());
  if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set PixelFormat to %s (Error: 0x%x). Camera might use default format.", pixel_format_.c_str(), nRet);
  }

  RCLCPP_INFO(this->get_logger(), "Camera parameters applied: Format=%s, Exp=%.1fus, Gain=%.1fdB, FPS=%.1f", 
              pixel_format_.c_str(), exposure_time_, gain_, frame_rate_);
  return true;
}

void CameraNode::start_grabbing()
{
  if (!handle_ || !is_connected_) return;
  
  int nRet = MV_CC_StartGrabbing(handle_);
  if (nRet == MV_OK) {
    is_running_ = true;
    grab_thread_ = std::thread(&CameraNode::grab_loop, this);
    RCLCPP_INFO(this->get_logger(), "Started grabbing.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Start grabbing failed: 0x%x", nRet);
  }
}

void CameraNode::stop_grabbing()
{
  if (handle_) {
    MV_CC_StopGrabbing(handle_);
    RCLCPP_INFO(this->get_logger(), "Stopped grabbing.");
  }
}

void CameraNode::grab_loop()
{
  MV_FRAME_OUT stFrameOut;
  memset(&stFrameOut, 0, sizeof(MV_FRAME_OUT));
  int timeout_count = 0;

  while (rclcpp::ok() && is_running_) {
    int nRet = MV_CC_GetImageBuffer(handle_, &stFrameOut, 1000);
    
    if (nRet == MV_OK) {
      timeout_count = 0; // 重置超时计数
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = this->now();
      msg->header.frame_id = "camera_optical_frame";
      msg->height = stFrameOut.stFrameInfo.nHeight;
      msg->width = stFrameOut.stFrameInfo.nWidth;
      msg->encoding = get_ros_encoding(pixel_format_);
      msg->step = stFrameOut.stFrameInfo.nFrameLen / stFrameOut.stFrameInfo.nHeight;
      msg->data.assign(stFrameOut.pBufAddr, stFrameOut.pBufAddr + stFrameOut.stFrameInfo.nFrameLen);

      image_pub_->publish(std::move(msg));
      MV_CC_FreeImageBuffer(handle_, &stFrameOut);
      
    } else if (nRet == (int)MV_E_HANDLE || nRet == (int)MV_E_CALLORDER) {
      RCLCPP_WARN(this->get_logger(), "Camera handle error. Triggering reconnect.");
      is_connected_ = false;
      is_running_ = false;
      reconnect();
      break; 
    } else {
      // 【关键修改 2】打印取流错误，帮助调试
      timeout_count++;
      if (nRet != (int)MV_E_TIMEOUT) {
          RCLCPP_ERROR(this->get_logger(), "GetImageBuffer Error: 0x%x", nRet);
      } else if (timeout_count % 10 == 0) {
          // 每 10 次超时打印一次，避免刷屏
          RCLCPP_WARN(this->get_logger(), "GetImageBuffer Timeout (Camera might not be sending images). Check TriggerMode.");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void CameraNode::reconnect()
{
  if (reconnect_timer_) reconnect_timer_->cancel();
  RCLCPP_WARN(this->get_logger(), "Attempting to reconnect...");
  is_running_ = false;
  if (grab_thread_.joinable()) grab_thread_.join();
  stop_grabbing();
  if (handle_) { MV_CC_DestroyHandle(handle_); handle_ = nullptr; }
  is_connected_ = false;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  if (init_camera()) {
    apply_camera_params();
    start_grabbing();
    if (reconnect_timer_) reconnect_timer_->reset();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Reconnect failed.");
    if (reconnect_timer_) reconnect_timer_->reset();
  }
}

std::string CameraNode::get_ros_encoding(const std::string & mv_pixel_format)
{
  if (mv_pixel_format == "Mono8") return sensor_msgs::image_encodings::MONO8;
  if (mv_pixel_format == "RGB8Packed") return sensor_msgs::image_encodings::RGB8;
  if (mv_pixel_format == "BayerRG8") return sensor_msgs::image_encodings::BAYER_RGGB8;
  if (mv_pixel_format == "BayerGB8") return sensor_msgs::image_encodings::BAYER_GBRG8;
  if (mv_pixel_format == "BayerGR8") return sensor_msgs::image_encodings::BAYER_GRBG8;
  if (mv_pixel_format == "BayerBG8") return sensor_msgs::image_encodings::BAYER_BGGR8;
  
  RCLCPP_WARN(this->get_logger(), "Unknown pixel format: %s, defaulting to mono8", mv_pixel_format.c_str());
  return sensor_msgs::image_encodings::MONO8;
}

} // namespace hikrobot_camera