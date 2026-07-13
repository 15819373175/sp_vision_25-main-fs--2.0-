#include "io/cboard.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <vector>
#include <sstream>
#include <thread>

namespace io
{

// ==========================================
// [地址确认] 
// 既然 launch 文件里是 sentry_up，且日志能打印，
// 0x02 是完全正确的！保持 0x02！
static const uint8_t ROBOT_D_ADDR = 0x02; 
// ==========================================

CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(11.6),
  queue_(5000),
  rx_running_(false)
{
  std::string port_name = read_yaml(config_path);

  try {
    serial_.setPort(port_name);
    serial_.setBaudrate(115200); 
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(to);
    serial_.open();
  } catch (serial::IOException& e) {
    tools::logger()->error("Unable to open port {}", port_name);
    exit(-1);
  }

  if (serial_.isOpen()) {
    tools::logger()->info("Serial Port {} initialized.", port_name);
    rx_running_ = true;
    rx_thread_ = std::thread(&CBoard::receive_loop, this);
  } else {
    tools::logger()->error("Serial Port {} failed to open.", port_name);
  }

  auto now = std::chrono::steady_clock::now();
  queue_.push({{1, 0, 0, 0}, now});
  queue_.push({{1, 0, 0, 0}, now});
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
}

CBoard::~CBoard() {
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void CBoard::receive_loop() {
    RxState state = WAIT_HEAD;
    std::vector<uint8_t> rx_data_buffer;
    uint8_t packet_id = 0;
    uint8_t packet_len = 0;
    
    uint8_t calc_sum = 0;
    uint8_t calc_add = 0;
    uint8_t rx_sum = 0;
    uint8_t rx_add = 0;

    while (rx_running_) {
        try {
            size_t available = serial_.available();
            if (available > 0) {
                std::vector<uint8_t> buffer;
                serial_.read(buffer, available);

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
                            rx_add = byte;
                            if (rx_sum == (calc_sum & 0xFF)) {
                                process_packet(packet_id, rx_data_buffer);
                            }
                            state = WAIT_HEAD;
                            break;
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void CBoard::process_packet(uint8_t id, const std::vector<uint8_t>& data) {
    auto get_int32 = [&](size_t offset) -> int32_t {
        if (offset + 4 > data.size()) return 0;
        int32_t val = 0;
        memcpy(&val, &data[offset], 4);
        return val;
    };

    if (id == ID_GIMBAL || id == 0x20) { 
        if (data.size() < 12) return;
        double yaw = get_int32(1) / 1000.0;   
        double pitch = get_int32(5) / 1000.0;
        
        if (std::abs(yaw) > 360.0 || std::abs(pitch) > 180.0) return;

        double yaw_rad = yaw * M_PI / 180.0;
        double pitch_rad = -pitch * M_PI / 180.0; 
        double roll_rad = 0; 

        Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        queue_.push({q, std::chrono::steady_clock::now()});
    }
}

void CBoard::send(Command command) {
    if (!serial_.isOpen()) return;

    // [简单的限流] 防止发送太快堵死串口 (仅允许 ~100Hz)
    static auto last_send_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count() < 10) {
        return; // 如果距离上次发送小于 10ms，直接跳过
    }
    last_send_time = now;

    // 辅助 lambda
    auto push_int32 = [](std::vector<uint8_t>& frame, int32_t val) {
        frame.push_back(val & 0xFF);
        frame.push_back((val >> 8) & 0xFF);
        frame.push_back((val >> 16) & 0xFF);
        frame.push_back((val >> 24) & 0xFF);
    };

    // ==========================================
    // 只发送 ID 0x20 (云台包)
    // 经确认：下位机应该是通过读取 Roll 轴数据(1000)来触发射击的
    // ==========================================
    std::vector<uint8_t> frame_gimbal;
    frame_gimbal.reserve(32);
    frame_gimbal.push_back(HEAD_BYTE);     
    frame_gimbal.push_back(0x02); // Addr: Sentry Up          
    frame_gimbal.push_back(0x20); // ID: Gimbal     
    frame_gimbal.push_back(25);   // Len

    // 这里的 1000 是关键！
    int32_t fire_advice_as_roll = command.shoot ? 1000 : 0.0; 

    uint8_t ctrl_mode = 1;
    frame_gimbal.push_back(ctrl_mode);
    push_int32(frame_gimbal, static_cast<int32_t>(command.yaw * 180.0 / M_PI * 1000));
    push_int32(frame_gimbal, static_cast<int32_t>(command.pitch * 180.0 / M_PI * 1000));
    
    // 发送 1000
    push_int32(frame_gimbal, fire_advice_as_roll); 
    push_int32(frame_gimbal, 1000); 
    push_int32(frame_gimbal, 1000); 
    push_int32(frame_gimbal, 1000); 

    uint8_t sum_check = 0, add_check = 0;
    for (uint8_t b : frame_gimbal) { sum_check += b; add_check += sum_check; }
    frame_gimbal.push_back(sum_check);
    frame_gimbal.push_back(add_check);
    
    try { 
        serial_.write(frame_gimbal); 
        // 只有开火时才打印，且降低打印频率，防止卡顿
        if (command.shoot) {
             // tools::logger()->info("FIRE! Val: 1000 (0x3e8)"); 
        }
    } catch (...) {}

    // [注意] 这里删除了 ID 0x40 的发送代码
    // 既然 Python 主要是靠 0x20 的 fire_advice 工作，
    // 我们先专注于让这一个包稳定传输，避免带宽竞争。
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (queue_.empty()) return {1,0,0,0}; 
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;
  while (true) {
    if(queue_.empty()) break;
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }
  
  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  double t = 0.5;
  auto duration_total = std::chrono::duration<double>(data_behind_.timestamp - data_ahead_.timestamp).count();
  if(duration_total > 1e-6) {
      t = std::chrono::duration<double>(timestamp - data_ahead_.timestamp).count() / duration_total;
  }
  return q_a.slerp(t, q_b).normalized();
}

std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  if (yaml["gimbal"] && yaml["gimbal"]["com_port"]) {
      return yaml["gimbal"]["com_port"].as<std::string>();
  }
  return "/dev/ttyACM0"; 
}

} // namespace io
