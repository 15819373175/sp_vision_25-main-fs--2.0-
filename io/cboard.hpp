#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <thread>
#include <vector>
#include <deque>
#include "serial/serial.h"  // 确保你的项目中已包含 serial 库
#include "io/command.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// 定义协议常量 (参考 protocol.py)
constexpr uint8_t HEAD_BYTE = 0xFF;
constexpr uint8_t D_ADDR_PC = 0x01;       // 电脑(Mainfold)地址，接收时校验用
constexpr uint8_t D_ADDR_ROBOT = 0x10;    // 步兵/Standard地址，发送时用
constexpr uint8_t ID_GIMBAL = 0x20;       // 云台数据 ID
constexpr uint8_t ID_SHOOTER = 0x40;      // 射击数据 ID

enum Mode {
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

enum ShootMode {
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  // 公共成员变量，用于 main 循环读取
  std::atomic<double> bullet_speed;
  std::atomic<Mode> mode;
  std::atomic<ShootMode> shoot_mode;
  
  CBoard(const std::string & config_path);
  ~CBoard();

  // 获取指定时间点的 IMU 姿态（插值）
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // 发送控制指令
  void send(Command command);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  // 接收状态机状态
  enum RxState {
    WAIT_HEAD,
    WAIT_D_ADDR,
    WAIT_ID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_SUM_CHECK,
    WAIT_ADD_CHECK
  };

  serial::Serial serial_;
  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_;
  IMUData data_behind_;

  // 接收线程相关
  std::thread rx_thread_;
  std::atomic<bool> rx_running_;
  void receive_loop();

  // 协议处理函数
  void process_packet(uint8_t id, const std::vector<uint8_t>& data);
  
  // 辅助函数：读取配置
  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP