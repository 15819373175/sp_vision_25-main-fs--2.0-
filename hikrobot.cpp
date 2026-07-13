#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(double exposure_ms, double gain, double gamma, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e3), gain_(gain), gamma_(gamma), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  // ============== [参考代码逻辑移植开始] ==============
  // 显式设置像素格式！这是解决你问题的关键
  // 我们设置为 BayerRG8，因为传输快，且你的代码已经支持它的软解
  ret = MV_CC_SetPixelFormat(handle_, PixelType_Gvsp_BayerRG8);
  if (ret != MV_OK) {
      // 如果设置失败（有的相机不支持RG8），尝试设置 RGB8 作为备选
      tools::logger()->warn("Set BayerRG8 failed: {:#x}, trying RGB8...", ret);
      MV_CC_SetPixelFormat(handle_, PixelType_Gvsp_RGB8_Packed);
  }
  // ============== [参考代码逻辑移植结束] ==============

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  set_float_value("Gamma", gamma_);
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();

      // ============== [鲁棒性补丁开始] ==============
      // 即使前面强制设置了格式，这里依然保留动态判断，防止“漏网之鱼”导致闪退
      int channels = 1;
      if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_YUV422_YUYV_Packed ||
          raw.stFrameInfo.enPixelType == PixelType_Gvsp_YUV422_Packed) {
          channels = 2; // YUV 格式是双通道
      } else if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed ||
                 raw.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
          channels = 3; // RGB/BGR 是三通道
      }
      
      // 动态创建 Mat，彻底杜绝 Assertion failed !fixedType() 错误
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), 
                  CV_MAKETYPE(CV_8U, channels), 
                  raw.pBufAddr);
      // ============== [鲁棒性补丁结束] ==============

      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      
      // 这里的映射表已经包含你之前报错的 YUV 格式
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB},
        {PixelType_Gvsp_YUV422_YUYV_Packed, cv::COLOR_YUV2BGR_YUYV} // 支持 YUV
      };

      if (type_map.count(pixel_type)) {
          cv::cvtColor(img, dst_image, type_map.at(pixel_type));
          img = dst_image;
      } 
      // [新增] 如果相机本身就输出了 RGB/BGR，就不需要 cvtColor 了，直接用
      else if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
          // 已经是 BGR，无需转换
      }
      else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
          cv::cvtColor(img, dst_image, cv::COLOR_RGB2BGR); // 转成 OpenCV 习惯的 BGR
          img = dst_image;
      }
      else {
          tools::logger()->warn("Unknown pixel type: {:#x}", pixel_type);
      }

      queue_.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io