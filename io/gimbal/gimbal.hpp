#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// ==========================================
// 通信健康监控统计
// ==========================================
struct GimbalStats
{
  std::atomic<uint64_t> tx_frames{0};         // 发送帧总数
  std::atomic<uint64_t> tx_errors{0};         // 发送失败次数
  std::atomic<uint64_t> tx_dropped{0};        // 因限流丢弃的帧数
  std::atomic<uint64_t> rx_frames{0};         // 成功接收帧总数
  std::atomic<uint64_t> rx_checksum_err{0};   // 校验和错误数
  std::atomic<uint64_t> rx_overflow{0};       // 队列溢出次数
  std::atomic<int64_t> last_tx_time_us{0};    // 最后一次发送时间
  std::atomic<int64_t> last_rx_time_us{0};    // 最后一次接收时间

  // 默认构造
  GimbalStats() = default;

  // 显式复制构造：
  // std::atomic 不能直接复制，需要先 load 普通数值
  GimbalStats(const GimbalStats & other) noexcept
    : tx_frames(other.tx_frames.load(std::memory_order_relaxed)),
      tx_errors(other.tx_errors.load(std::memory_order_relaxed)),
      tx_dropped(other.tx_dropped.load(std::memory_order_relaxed)),
      rx_frames(other.rx_frames.load(std::memory_order_relaxed)),
      rx_checksum_err(other.rx_checksum_err.load(std::memory_order_relaxed)),
      rx_overflow(other.rx_overflow.load(std::memory_order_relaxed)),
      last_tx_time_us(other.last_tx_time_us.load(std::memory_order_relaxed)),
      last_rx_time_us(other.last_rx_time_us.load(std::memory_order_relaxed))
  {
  }

  // 显式复制赋值
  GimbalStats & operator=(const GimbalStats & other) noexcept
  {
    if (this == &other) {
      return *this;
    }

    tx_frames.store(
      other.tx_frames.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    tx_errors.store(
      other.tx_errors.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    tx_dropped.store(
      other.tx_dropped.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    rx_frames.store(
      other.rx_frames.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    rx_checksum_err.store(
      other.rx_checksum_err.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    rx_overflow.store(
      other.rx_overflow.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    last_tx_time_us.store(
      other.last_tx_time_us.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    last_rx_time_us.store(
      other.last_rx_time_us.load(std::memory_order_relaxed),
      std::memory_order_relaxed);

    return *this;
  }
};

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  // 当前反馈协议没有 pitch_vel，保留字段以兼容现有上位机模块。
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // 发送控制指令
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch);

  // 通信健康监控
  GimbalStats stats() const;
  bool is_gimbal_alive() const;

private:
  // ==========================================
  // BCP 方案 B 协议常量
  // ==========================================
  static constexpr uint8_t HEAD_BYTE = 0xFF;
  static constexpr uint8_t DESTINATION_ADDRESS = 0x02;
  static constexpr uint8_t ID_GIMBAL = 0x20;
  // 上位机 -> 下位机：mode + yaw + yaw_vel + yaw_acc + pitch。
  static constexpr uint8_t GIMBAL_COMMAND_PAYLOAD_LENGTH = 17;

  // 下位机 -> 上位机：mode + yaw + pitch + reserved[8] + yaw_vel + yaw_acc。
  static constexpr uint8_t GIMBAL_FEEDBACK_PAYLOAD_LENGTH = 25;
  static constexpr std::size_t FEEDBACK_YAW_OFFSET = 1;
  static constexpr std::size_t FEEDBACK_PITCH_OFFSET = 5;
  static constexpr std::size_t FEEDBACK_YAW_VEL_OFFSET = 17;
  static constexpr std::size_t FEEDBACK_YAW_ACC_OFFSET = 21;

  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  std::atomic<bool> serial_connected_{false};
  int consecutive_tx_failures_ = 0;
  mutable std::mutex mutex_;

  GimbalStats stats_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  
  // 用于插值的四元数队列
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  // ==========================================
  // 接收状态机相关
  // ==========================================
  enum RxState {
    WAIT_HEAD,
    WAIT_D_ADDR,
    WAIT_ID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_SUM_CHECK,
    WAIT_ADD_CHECK
  };

  void read_thread();    // 接收线程函数
  void reconnect();      // 重连逻辑
  void process_packet(uint8_t id, const std::vector<uint8_t>& data); // 解包处理
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
