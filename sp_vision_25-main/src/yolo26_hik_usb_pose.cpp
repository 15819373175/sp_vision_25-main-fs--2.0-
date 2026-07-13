#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>

struct ArmorDetection {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect box;

    // 顺序: TL, BL, BR, TR
    std::vector<cv::Point2f> keypoints;
    std::vector<float> kpt_scores;
};

// ========================== 参数区 ==========================

static const int INPUT_W = 640;
static const int INPUT_H = 640;

static const int NUM_CLASSES = 36;
static const int NUM_KEYPOINTS = 4;

static const float CONF_THRESHOLD = 0.35f;
static const float NMS_THRESHOLD  = 0.45f;

// 是否显示画面。机器人无显示器时建议 false
static const bool DEFAULT_SHOW_WINDOW = false;

static const std::vector<std::string> CLASS_NAMES = {
    "BG", "B1", "B2", "B3", "B4", "B5", "BO", "BBs", "BBb",
    "RG", "R1", "R2", "R3", "R4", "R5", "RO", "RBs", "RBb",
    "NG", "N1", "N2", "N3", "N4", "N5", "NO", "NBs", "NBb",
    "PG", "P1", "P2", "P3", "P4", "P5", "PO", "PBs", "PBb"
};

// ===========================================================

static float clampFloat(float v, float low, float high) {
    return std::max(low, std::min(v, high));
}

static cv::Mat letterbox(
    const cv::Mat& src,
    int target_w,
    int target_h,
    float& scale,
    int& pad_x,
    int& pad_y
) {
    int src_w = src.cols;
    int src_h = src.rows;

    scale = std::min(
        target_w / static_cast<float>(src_w),
        target_h / static_cast<float>(src_h)
    );

    int new_w = static_cast<int>(std::round(src_w * scale));
    int new_h = static_cast<int>(std::round(src_h * scale));

    pad_x = (target_w - new_w) / 2;
    pad_y = (target_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));

    cv::Mat output(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(output(cv::Rect(pad_x, pad_y, new_w, new_h)));

    return output;
}

static cv::Mat formatOutputToRows(const cv::Mat& raw_output) {
    cv::Mat output;

    if (raw_output.type() != CV_32F) {
        raw_output.convertTo(output, CV_32F);
    } else {
        output = raw_output;
    }

    cv::Mat pred;

    if (output.dims == 3) {
        int dim1 = output.size[1];
        int dim2 = output.size[2];

        cv::Mat temp(dim1, dim2, CV_32F, output.ptr<float>());

        // [1, 52, 8400] -> transpose -> [8400, 52]
        if (dim1 < dim2) {
            cv::transpose(temp, pred);
        }
        // [1, 8400, 52]
        else {
            pred = temp;
        }
    }
    else if (output.dims == 2) {
        pred = output;
    }
    else {
        std::cerr << "[ERROR] Unsupported output dims: " << output.dims << std::endl;
    }

    return pred;
}

static std::vector<ArmorDetection> postprocessPose(
    const cv::Mat& raw_output,
    const cv::Size& original_size,
    float scale,
    int pad_x,
    int pad_y
) {
    std::vector<ArmorDetection> final_detections;

    cv::Mat pred = formatOutputToRows(raw_output);

    if (pred.empty()) {
        return final_detections;
    }

    int rows = pred.rows;
    int cols = pred.cols;

    const int attr_without_obj = 4 + NUM_CLASSES + NUM_KEYPOINTS * 3;      // 52
    const int attr_with_obj    = 4 + 1 + NUM_CLASSES + NUM_KEYPOINTS * 3;  // 53

    bool has_objectness = false;
    int class_start = 4;
    int kpt_start = 4 + NUM_CLASSES;

    if (cols == attr_without_obj) {
        has_objectness = false;
        class_start = 4;
        kpt_start = 4 + NUM_CLASSES;
    }
    else if (cols == attr_with_obj) {
        has_objectness = true;
        class_start = 5;
        kpt_start = 5 + NUM_CLASSES;
    }
    else {
        std::cerr << "[ERROR] Unsupported output cols: " << cols << std::endl;
        std::cerr << "Expected " << attr_without_obj << " or " << attr_with_obj << std::endl;
        std::cerr << "Please check your ONNX output shape." << std::endl;
        return final_detections;
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    std::vector<std::vector<cv::Point2f>> all_keypoints;
    std::vector<std::vector<float>> all_kpt_scores;

    for (int i = 0; i < rows; ++i) {
        const float* data = pred.ptr<float>(i);

        float cx = data[0];
        float cy = data[1];
        float w  = data[2];
        float h  = data[3];

        bool normalized_coord = false;

        if (cx <= 2.0f && cy <= 2.0f && w <= 2.0f && h <= 2.0f) {
            normalized_coord = true;
            cx *= INPUT_W;
            cy *= INPUT_H;
            w  *= INPUT_W;
            h  *= INPUT_H;
        }

        float obj_conf = 1.0f;

        if (has_objectness) {
            obj_conf = data[4];
        }

        int best_class_id = -1;
        float best_class_score = 0.0f;

        for (int c = 0; c < NUM_CLASSES; ++c) {
            float score = data[class_start + c];

            if (score > best_class_score) {
                best_class_score = score;
                best_class_id = c;
            }
        }

        float confidence = obj_conf * best_class_score;

        if (confidence < CONF_THRESHOLD) {
            continue;
        }

        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;

        int left = static_cast<int>((x1 - pad_x) / scale);
        int top  = static_cast<int>((y1 - pad_y) / scale);
        int bw   = static_cast<int>(w / scale);
        int bh   = static_cast<int>(h / scale);

        left = std::max(0, std::min(left, original_size.width - 1));
        top  = std::max(0, std::min(top, original_size.height - 1));
        bw   = std::max(0, std::min(bw, original_size.width - left));
        bh   = std::max(0, std::min(bh, original_size.height - top));

        std::vector<cv::Point2f> kpts;
        std::vector<float> kpt_scores;

        for (int k = 0; k < NUM_KEYPOINTS; ++k) {
            float kx = data[kpt_start + k * 3 + 0];
            float ky = data[kpt_start + k * 3 + 1];
            float ks = data[kpt_start + k * 3 + 2];

            if (normalized_coord) {
                kx *= INPUT_W;
                ky *= INPUT_H;
            }

            float ox = (kx - pad_x) / scale;
            float oy = (ky - pad_y) / scale;

            ox = clampFloat(ox, 0.0f, static_cast<float>(original_size.width - 1));
            oy = clampFloat(oy, 0.0f, static_cast<float>(original_size.height - 1));

            kpts.emplace_back(ox, oy);
            kpt_scores.push_back(ks);
        }

        boxes.emplace_back(left, top, bw, bh);
        confidences.push_back(confidence);
        class_ids.push_back(best_class_id);
        all_keypoints.push_back(kpts);
        all_kpt_scores.push_back(kpt_scores);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(
        boxes,
        confidences,
        CONF_THRESHOLD,
        NMS_THRESHOLD,
        indices
    );

    for (int idx : indices) {
        ArmorDetection det;

        det.class_id = class_ids[idx];
        det.class_name = CLASS_NAMES[det.class_id];
        det.confidence = confidences[idx];
        det.box = boxes[idx];
        det.keypoints = all_keypoints[idx];
        det.kpt_scores = all_kpt_scores[idx];

        final_detections.push_back(det);
    }

    return final_detections;
}

static void printDetections(
    int frame_id,
    const std::vector<ArmorDetection>& detections
) {
    if (detections.empty()) {
        return;
    }

    std::cout << "========== Frame " << frame_id << " ==========" << std::endl;

    for (const auto& det : detections) {
        int center_x = det.box.x + det.box.width / 2;
        int center_y = det.box.y + det.box.height / 2;

        std::cout << std::fixed << std::setprecision(3);

        std::cout << "Class: " << det.class_name
                  << " | ID: " << det.class_id
                  << " | Conf: " << det.confidence
                  << " | BBox: ["
                  << det.box.x << ", "
                  << det.box.y << ", "
                  << det.box.width << ", "
                  << det.box.height << "]"
                  << " | Center: (" << center_x << ", " << center_y << ")"
                  << std::endl;

        const char* kpt_names[4] = {"TL", "BL", "BR", "TR"};

        for (int i = 0; i < 4; ++i) {
            std::cout << "  " << kpt_names[i]
                      << ": (" << det.keypoints[i].x
                      << ", " << det.keypoints[i].y
                      << "), score=" << det.kpt_scores[i]
                      << std::endl;
        }
    }
}

static void drawDetections(
    cv::Mat& frame,
    const std::vector<ArmorDetection>& detections
) {
    for (const auto& det : detections) {
        cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);

        std::string label = det.class_name + " " + cv::format("%.2f", det.confidence);

        cv::putText(
            frame,
            label,
            cv::Point(det.box.x, std::max(0, det.box.y - 5)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2
        );

        // TL, BL, BR, TR
        cv::Scalar colors[4] = {
            cv::Scalar(255, 0, 0),
            cv::Scalar(0, 255, 255),
            cv::Scalar(0, 0, 255),
            cv::Scalar(255, 0, 255)
        };

        for (int i = 0; i < 4; ++i) {
            cv::circle(frame, det.keypoints[i], 4, colors[i], -1);
        }

        // 按 TL -> BL -> BR -> TR 连线
        if (det.keypoints.size() == 4) {
            cv::line(frame, det.keypoints[0], det.keypoints[1], cv::Scalar(255, 255, 0), 2);
            cv::line(frame, det.keypoints[1], det.keypoints[2], cv::Scalar(255, 255, 0), 2);
            cv::line(frame, det.keypoints[2], det.keypoints[3], cv::Scalar(255, 255, 0), 2);
            cv::line(frame, det.keypoints[3], det.keypoints[0], cv::Scalar(255, 255, 0), 2);
        }
    }
}

static void printOutputShapeOnce(const cv::Mat& output) {
    static bool printed = false;

    if (printed) {
        return;
    }

    printed = true;

    std::cout << "[INFO] ONNX output dims: " << output.dims << std::endl;

    std::cout << "[INFO] ONNX output shape: [";
    for (int i = 0; i < output.dims; ++i) {
        std::cout << output.size[i];
        if (i + 1 < output.dims) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

int main(int argc, char** argv) {
    std::string model_path = "assets/yolo26.onnx";
    int camera_id = 0;
    bool show_window = DEFAULT_SHOW_WINDOW;

    if (argc >= 2) {
        model_path = argv[1];
    }

    if (argc >= 3) {
        camera_id = std::stoi(argv[2]);
    }

    if (argc >= 4) {
        show_window = std::stoi(argv[3]) != 0;
    }

    std::cout << "[INFO] Model path: " << model_path << std::endl;
    std::cout << "[INFO] Camera ID: " << camera_id << std::endl;
    std::cout << "[INFO] Show window: " << (show_window ? "true" : "false") << std::endl;

    cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);

    if (net.empty()) {
        std::cerr << "[ERROR] Failed to load ONNX model." << std::endl;
        return -1;
    }

    // 通用 CPU 推理
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    cv::VideoCapture cap;

#ifdef _WIN32
    // Windows 下 USB 摄像头建议先用 DSHOW
    cap.open(camera_id, cv::CAP_DSHOW);
#else
    // Linux 下通常对应 /dev/video0, /dev/video1 ...
    cap.open(camera_id);
#endif

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Failed to open camera: " << camera_id << std::endl;
        std::cerr << "[HINT] 如果这是海康工业相机且不是 UVC 模式，需要用海康 MVS SDK 采图。" << std::endl;
        return -1;
    }

    // 根据你们相机实际能力调
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 60);

    std::cout << "[INFO] Camera opened." << std::endl;

    int frame_id = 0;

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "[WARN] Empty frame." << std::endl;
            continue;
        }

        frame_id++;

        float scale = 1.0f;
        int pad_x = 0;
        int pad_y = 0;

        cv::Mat input_img = letterbox(
            frame,
            INPUT_W,
            INPUT_H,
            scale,
            pad_x,
            pad_y
        );

        cv::Mat blob = cv::dnn::blobFromImage(
            input_img,
            1.0 / 255.0,
            cv::Size(INPUT_W, INPUT_H),
            cv::Scalar(),
            true,
            false
        );

        net.setInput(blob);

        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());

        if (outputs.empty()) {
            std::cerr << "[ERROR] Model has no output." << std::endl;
            continue;
        }

        printOutputShapeOnce(outputs[0]);

        std::vector<ArmorDetection> detections = postprocessPose(
            outputs[0],
            frame.size(),
            scale,
            pad_x,
            pad_y
        );

        printDetections(frame_id, detections);

        if (show_window) {
            drawDetections(frame, detections);

            cv::imshow("YOLO26 Armor Pose Detection", frame);

            int key = cv::waitKey(1);
            if (key == 27) {
                break;
            }
        }
    }

    cap.release();

    if (show_window) {
        cv::destroyAllWindows();
    }

    return 0;
}