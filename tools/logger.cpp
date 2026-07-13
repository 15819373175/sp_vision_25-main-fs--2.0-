#include "logger.hpp"

#include <ctime>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger()
{
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  #ifdef _WIN32
  localtime_s(&now_tm, &now_c);
  #else
  localtime_r(&now_c, &now_tm);
  #endif
  char time_str[64];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d_%H-%M-%S", &now_tm);
  auto file_name = fmt::format("logs/{}.log", time_str);
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::debug);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
