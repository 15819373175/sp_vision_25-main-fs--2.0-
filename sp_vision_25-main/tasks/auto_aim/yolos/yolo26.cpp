#include "yolo26.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLO26::YOLO26(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolo26_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLO26::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  tmp_img_ = raw_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLO26::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // output layout after transpose: [num_det, 18]
  // cols: 0-3=xywh, 4=conf, 5=cls_id, 6-17=keypoints(x,y,vis)*4
  cv::transpose(output, output);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    // output layout: [xywh(4), conf(1), cls_id(1), kpts(x,y,vis)*4=12] => 18 cols
    float conf   = output.at<float>(r, 4);
    int   cls_id = static_cast<int>(output.at<float>(r, 5));
    auto  one_key_points = output.row(r).colRange(6, 18);

    if (conf < score_threshold_) continue;

    auto x = output.at<float>(r, 0);
    auto y = output.at<float>(r, 1);
    auto w = output.at<float>(r, 2);
    auto h = output.at<float>(r, 3);
    auto left = static_cast<int>((x - 0.5f * w) / scale);
    auto top = static_cast<int>((y - 0.5f * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    std::vector<cv::Point2f> armor_key_points;
    for (int i = 0; i < 4; i++) {
      // stride 3: x, y, visibility
      float kx = one_key_points.at<float>(0, i * 3 + 0) / scale;
      float ky = one_key_points.at<float>(0, i * 3 + 1) / scale;
      armor_key_points.push_back({kx, ky});
    }

    ids.emplace_back(cls_id);
    confidences.emplace_back(conf);
    boxes.emplace_back(left, top, width, height);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  // yolo26 class layout: 4 colors × 9 types, grouped by color
  // id = color*9 + type_idx, type_idx: G=0,1=1,2=2,3=3,4=4,5=5,O=6,Bs=7,Bb=8
  static const std::vector<std::tuple<Color, ArmorName, ArmorType>> yolo26_properties = {
    {blue,       sentry,  small}, {blue,       one,    small}, {blue,       two,    small},
    {blue,       three,   small}, {blue,       four,   small}, {blue,       five,   small},
    {blue,       outpost, small}, {blue,       base,   small}, {blue,       base,   big  },
    {red,        sentry,  small}, {red,        one,    small}, {red,        two,    small},
    {red,        three,   small}, {red,        four,   small}, {red,        five,   small},
    {red,        outpost, small}, {red,        base,   small}, {red,        base,   big  },
    {extinguish, sentry,  small}, {extinguish, one,    small}, {extinguish, two,    small},
    {extinguish, three,   small}, {extinguish, four,   small}, {extinguish, five,   small},
    {extinguish, outpost, small}, {extinguish, base,   small}, {extinguish, base,   big  },
    {purple,     sentry,  small}, {purple,     one,    small}, {purple,     two,    small},
    {purple,     three,   small}, {purple,     four,   small}, {purple,     five,   small},
    {purple,     outpost, small}, {purple,     base,   small}, {purple,     base,   big  },
  };

  std::list<Armor> armors;
  for (const auto & i : indices) {
    sort_keypoints(armors_key_points[i]);
    Armor armor = use_roi_
      ? Armor(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_)
      : Armor(ids[i], confidences[i], boxes[i], armors_key_points[i]);
    // override color/name/type using yolo26-specific mapping
    if (ids[i] >= 0 && ids[i] < static_cast<int>(yolo26_properties.size())) {
      auto [c, n, t] = yolo26_properties[ids[i]];
      armor.color = c;
      armor.name  = n;
      armor.type  = t;
    } else {
      armor.color = blue;
      armor.name  = not_armor;
      armor.type  = small;
    }
    armors.emplace_back(std::move(armor));
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) { it = armors.erase(it); continue; }
    if (!check_type(*it)) { it = armors.erase(it); continue; }
    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLO26::check_name(const Armor & armor) const
{
  return armor.name != ArmorName::not_armor && armor.confidence > min_confidence_;
}

bool YOLO26::check_type(const Armor & armor) const
{
  return (armor.type == ArmorType::small)
    ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
    : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
       armor.name != ArmorName::outpost);
}

cv::Point2f YOLO26::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

void YOLO26::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) return;

  std::sort(keypoints.begin(), keypoints.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });

  std::vector<cv::Point2f> top = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bot = {keypoints[2], keypoints[3]};

  std::sort(top.begin(), top.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });
  std::sort(bot.begin(), bot.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

  keypoints[0] = top[0];  // top-left
  keypoints[1] = top[1];  // top-right
  keypoints[2] = bot[1];  // bottom-right
  keypoints[3] = bot[0];  // bottom-left
}

void YOLO26::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color],
      ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }
  if (use_roi_) cv::rectangle(detection, roi_, cv::Scalar(0, 255, 0), 2);
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("detection", detection);
}

std::list<Armor> YOLO26::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
