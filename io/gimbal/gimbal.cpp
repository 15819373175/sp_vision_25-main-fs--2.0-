#include "gimbal.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <cmath>
#include <cstring>
#include <vector>

namespace io
{

// ── 辅助：获取近似的微秒时间戳 ──
static int64_t now_us()
{
  auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(to);
    serial_.open();
    serial_connected_ = true;
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {} — will auto-reconnect", e.what());
    serial_connected_ = false;
  }

  // 初始化状态
  state_ = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 25.0F, 0};

  // 初始化队列防止为空
  auto now = std::chrono::steady_clock::now();
  queue_.push({Eigen::Quaterniond::Identity(), now});

  // 只有串口成功打开时才启动接收线程
  if (serial_connected_) {
    thread_ = std::thread(&Gimbal::read_thread, this);
    tools::logger()->info("[Gimbal] Serial initialized and read thread started.");
  } else {
    tools::logger()->warn("[Gimbal] Read thread NOT started — will retry on first send()");
  }
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (serial_.isOpen()) serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE: return "IDLE";
    case GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF: return "BIG_BUFF";
    default: return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    if (queue_.empty()) return Eigen::Quaterniond::Identity();

    auto [q_a, t_a] = queue_.pop();
    if (queue_.empty()) {
        return q_a;
    }

    auto [q_b, t_b] = queue_.front();

    if (t <= t_a) return q_a;

    if (t > t_a && t <= t_b) {
        auto t_ab = tools::delta_time(t_a, t_b);
        auto t_ac = tools::delta_time(t_a, t);
        double k = (t_ab > 1e-6) ? (t_ac / t_ab) : 0.0;
        return q_a.slerp(k, q_b).normalized();
    }
  }
}
// 发送控制指令，控制云台转动
void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch)
{
  // 串口未连接时尝试自动重连
  if (!serial_.isOpen() || !serial_connected_) {
    reconnect();
    if (!serial_.isOpen()) return;
  }

  // [限流保护]
  static auto last_send_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count() < 10) {
      stats_.tx_dropped++;
      return;
  }
  last_send_time = now;

  std::vector<uint8_t> frame_gimbal;
  frame_gimbal.reserve(4 + GIMBAL_COMMAND_PAYLOAD_LENGTH + 2);

  // 方案 B: 0=不控制，1=控制但不开火，2=控制并请求开火。
  const uint8_t control_mode = control ? (fire ? 2 : 1) : 0;

  // Planner 内部统一使用 rad、rad/s、rad/s^2；BCP 使用对应量的 degree * 1000。
  const auto to_milli_degrees = [](float value) {
    return static_cast<int32_t>(value * 180.0 / M_PI * 1000.0);
  };

  const int32_t yaw_int = to_milli_degrees(yaw);
  const int32_t yaw_vel_int = to_milli_degrees(yaw_vel);
  const int32_t yaw_acc_int = to_milli_degrees(yaw_acc);
  const int32_t pitch_int = to_milli_degrees(pitch);

  // ==========================================
  // 3. 辅助函数
  // ==========================================
  auto push_int32 = [](std::vector<uint8_t>& frame, int32_t val) {
      const auto uval = static_cast<uint32_t>(val);
      frame.push_back(uval & 0xFF);
      frame.push_back((uval >> 8) & 0xFF);
      frame.push_back((uval >> 16) & 0xFF);
      frame.push_back((uval >> 24) & 0xFF);
  };

  // 帧结构：FF 02 20 11 | mode + yaw/yaw_vel/yaw_acc + pitch | SC AC
  frame_gimbal.push_back(HEAD_BYTE);
  frame_gimbal.push_back(DESTINATION_ADDRESS);
  frame_gimbal.push_back(ID_GIMBAL);
  frame_gimbal.push_back(GIMBAL_COMMAND_PAYLOAD_LENGTH);

  frame_gimbal.push_back(control_mode);
  push_int32(frame_gimbal, yaw_int);
  push_int32(frame_gimbal, yaw_vel_int);
  push_int32(frame_gimbal, yaw_acc_int);
  push_int32(frame_gimbal, pitch_int);

  // 校验和 (Sum Check + Add Check)
  uint8_t sum_check = 0, add_check = 0;
  for (uint8_t b : frame_gimbal) { sum_check += b; add_check += sum_check; }
  frame_gimbal.push_back(sum_check);
  frame_gimbal.push_back(add_check);

  try {
      serial_.write(frame_gimbal);
      consecutive_tx_failures_ = 0;
      stats_.tx_frames++;
      stats_.last_tx_time_us = now_us();
  } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
      stats_.tx_errors++;
      consecutive_tx_failures_++;
      // 连续丢 50 帧（约 500ms）触发自动重连
      if (consecutive_tx_failures_ > 50) {
        tools::logger()->error("[Gimbal] {} consecutive tx failures, triggering reconnect", consecutive_tx_failures_);
        serial_connected_ = false;
        reconnect();
        consecutive_tx_failures_ = 0;
      }
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  RxState state = WAIT_HEAD;
  std::vector<uint8_t> rx_data_buffer;
  uint8_t packet_id = 0;
  uint8_t packet_len = 0;

  uint8_t calc_sum = 0;
  uint8_t calc_add = 0;
  uint8_t rx_sum = 0;

  int consecutive_read_errors = 0;

  while (!quit_) {
      try {
          size_t available = serial_.available();
          if (available > 0) {
              std::vector<uint8_t> buffer;
              serial_.read(buffer, available);
              consecutive_read_errors = 0;

              for (uint8_t byte : buffer) {
                  switch (state) {
                      case WAIT_HEAD:
                          if (byte == HEAD_BYTE) {
                              state = WAIT_D_ADDR;
                              calc_sum = byte;
                              calc_add = calc_sum;
                          }
                          break;
                      case WAIT_D_ADDR:
                          if (byte == HEAD_BYTE) {
                              state = WAIT_D_ADDR;
                              calc_sum = byte;
                              calc_add = calc_sum;
                          } else {
                              state = WAIT_ID;
                              calc_sum += byte;
                              calc_add += calc_sum;
                          }
                          break;
                      case WAIT_ID:
                          packet_id = byte;
                          state = WAIT_LEN;
                          calc_sum += byte;
                          calc_add += calc_sum;
                          break;
                      case WAIT_LEN:
                          packet_len = byte;
                          rx_data_buffer.clear();
                          if (packet_len > 100) {
                              state = WAIT_HEAD;
                          } else {
                              rx_data_buffer.reserve(packet_len);
                              state = WAIT_DATA;
                              calc_sum += byte;
                              calc_add += calc_sum;
                          }
                          break;
                      case WAIT_DATA:
                          if (rx_data_buffer.size() < packet_len) {
                              rx_data_buffer.push_back(byte);
                              calc_sum += byte;
                              calc_add += calc_sum;
                          }
                          if (rx_data_buffer.size() >= packet_len) {
                              state = WAIT_SUM_CHECK;
                          }
                          break;
                      case WAIT_SUM_CHECK:
                          rx_sum = byte;
                          state = WAIT_ADD_CHECK;
                          break;
                      case WAIT_ADD_CHECK:
                          if (rx_sum == (calc_sum & 0xFF)) {
                              process_packet(packet_id, rx_data_buffer);
                          } else {
                              stats_.rx_checksum_err++;
                              tools::logger()->debug("[Gimbal] Checksum failed.");
                          }
                          state = WAIT_HEAD;
                          break;
                  }
              }
          } else {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
      } catch (...) {
          consecutive_read_errors++;
          if (consecutive_read_errors > 100) {
            tools::logger()->error("[Gimbal] 100 consecutive read errors, marking disconnected");
            serial_connected_ = false;
            consecutive_read_errors = 0;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
  }
  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::process_packet(uint8_t id, const std::vector<uint8_t>& data) {
    auto get_int32 = [&](size_t offset) -> int32_t {
        if (offset + 4 > data.size()) return 0;
        int32_t val = 0;
        memcpy(&val, &data[offset], 4);
        return val;
    };

    if (id == ID_GIMBAL) {
        // mode + yaw + pitch + reserved[8] + yaw_vel + yaw_acc
        if (data.size() < GIMBAL_FEEDBACK_PAYLOAD_LENGTH) return;

        double yaw_deg = get_int32(FEEDBACK_YAW_OFFSET) / 1000.0;
        double pitch_deg = get_int32(FEEDBACK_PITCH_OFFSET) / 1000.0;
        double yaw_vel_deg = get_int32(FEEDBACK_YAW_VEL_OFFSET) / 1000.0;
        double yaw_acc_deg = get_int32(FEEDBACK_YAW_ACC_OFFSET) / 1000.0;

        if (std::abs(yaw_deg) > 360.0 || std::abs(pitch_deg) > 180.0) return;

        double yaw_rad = yaw_deg * M_PI / 180.0;
        double pitch_rad = -pitch_deg * M_PI / 180.0;
        double yaw_vel_rad = yaw_vel_deg * M_PI / 180.0;
        double yaw_acc_rad = yaw_acc_deg * M_PI / 180.0;
        double roll_rad = 0;

        Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        queue_.push({q, std::chrono::steady_clock::now()});

        std::lock_guard<std::mutex> lock(mutex_);
        state_.yaw = static_cast<float>(yaw_rad);
        state_.yaw_vel = static_cast<float>(yaw_vel_rad);
        state_.yaw_acc = static_cast<float>(yaw_acc_rad);
        state_.pitch = static_cast<float>(pitch_rad);
        state_.pitch_vel = 0.0F;

        switch (data[0]) {
          case 0: mode_ = GimbalMode::IDLE; break;
          case 1: mode_ = GimbalMode::AUTO_AIM; break;
          case 2: mode_ = GimbalMode::SMALL_BUFF; break;
          case 3: mode_ = GimbalMode::BIG_BUFF; break;
          default:
            mode_ = GimbalMode::IDLE;
            tools::logger()->warn("[Gimbal] Invalid feedback mode: {}", data[0]);
            break;
        }

        // 更新接收统计
        stats_.rx_frames++;
        stats_.last_rx_time_us = now_us();
    }
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {}

    try {
      serial_.open();
      serial_connected_ = true;
      consecutive_tx_failures_ = 0;
      // 重连后如果接收线程未启动，则启动它
      if (!thread_.joinable()) {
        thread_ = std::thread(&Gimbal::read_thread, this);
      }
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  tools::logger()->error("[Gimbal] Reconnect failed after {} attempts", max_retry_count);
  serial_connected_ = false;
}

GimbalStats Gimbal::stats() const
{
  return stats_;
}

bool Gimbal::is_gimbal_alive() const
{
  // 超过 100ms 没有收到下位机数据则判定离线
  auto last_rx = stats_.last_rx_time_us.load();
  if (last_rx == 0) return false;  // 从未收到过数据
  auto now = now_us();
  return (now - last_rx) < 100'000;  // 100ms 超时
}

}  // namespace io
