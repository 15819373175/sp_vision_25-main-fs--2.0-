#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto aim_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();

      io::Command cmd{false, false, 0, 0};

      if (target.has_value()) {
        cmd = aimer.aim({*target}, std::chrono::steady_clock::now(), gs.bullet_speed, true);
      }

      float yaw = 0, pitch = 0;
      bool fire = false;

      if (cmd.control && target.has_value()) {
        yaw = static_cast<float>(tools::limit_rad(cmd.yaw - gs.yaw));
        pitch = static_cast<float>(cmd.pitch - gs.pitch);

        Eigen::Vector3d gimbal_pos(gs.yaw, gs.pitch, 0);
        fire = shooter.shoot(cmd, aimer, {*target}, gimbal_pos);
      }

      gimbal.send(cmd.control, fire, yaw, 0, 0, pitch);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_yaw_acc"] = gs.yaw_acc;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["plan_yaw"] = yaw;
      data["plan_pitch"] = pitch;

      data["fire"] = fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];
        data["target_vz"] = target->ekf_x()[5];
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  while (!exiter.exit()) {
    auto t0 = std::chrono::steady_clock::now();
    auto [img, armors, t] = detector.debug_pop();
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto targets = tracker.track(armors, t);

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    cv::Mat display_img;
    cv::resize(img, display_img, {}, 0.5, 0.5);

    tools::draw_text(display_img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    auto latency_ms = tools::delta_time(std::chrono::steady_clock::now(), t) * 1000;
    tools::draw_text(display_img, fmt::format("Latence: {:.2f} ms", latency_ms), {10, 60}, {0, 255, 0});

    tools::draw_text(display_img, fmt::format("Armors: {}", armors.size()), {10, 90}, {255, 255, 0});

    cv::Point center(display_img.cols / 2, display_img.rows / 2);
    cv::circle(display_img, center, 5, {0, 0, 255}, 2);

    if (!targets.empty()) {
      auto target = targets.front();

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        std::vector<cv::Point> resized_points;
        for (const auto & pt : image_points) {
          resized_points.emplace_back(pt.x * 0.5, pt.y * 0.5);
        }
        tools::draw_points(display_img, resized_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      std::vector<cv::Point> resized_aim_points;
      for (const auto & pt : image_points) {
        resized_aim_points.emplace_back(pt.x * 0.5, pt.y * 0.5);
      }
      if (aim_point.valid)
        tools::draw_points(display_img, resized_aim_points, {0, 0, 255});
      else
        tools::draw_points(display_img, resized_aim_points, {255, 0, 0});
    }

    cv::imshow("reprojection", display_img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  exiter.exit();
  detect_thread.join();
  aim_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0);

  return 0;
}
