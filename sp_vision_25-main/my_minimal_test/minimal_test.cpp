// Minimal armor detection test: OpenCV + OpenVINO only
// Model output: [N, 18] = [x, y, w, h, conf, cls, kp0x, kp0y, kp0v, kp1x, kp1y, kp1v, kp2x, kp2y, kp2v, kp3x, kp3y, kp3v]
// Keypoint order: TL, BL, BR, TR

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// ---- config ----
static const std::string MODEL_PATH  = "assets/yolo26.onnx";
static const int   INPUT_W    = 640;
static const int   INPUT_H    = 640;
static const float CONF_THRESH = 0.45f;
static const float NMS_THRESH  = 0.45f;

// 36 class names matching data.yaml
static const std::vector<std::string> CLASS_NAMES = {
    "BG","B1","B2","B3","B4","B5","BO","BBs","BBb",
    "RG","R1","R2","R3","R4","R5","RO","RBs","RBb",
    "NG","N1","N2","N3","N4","N5","NO","NBs","NBb",
    "PG","P1","P2","P3","P4","P5","PO","PBs","PBb"
};

struct Detection {
    cv::Rect2f  box;       // x,y,w,h in original image coords
    float       conf;
    int         cls;
    cv::Point2f kpts[4];   // TL, BL, BR, TR in original image coords
};

// letterbox resize: keep aspect ratio, pad to square
static cv::Mat letterbox(const cv::Mat& src, int target_w, int target_h,
                         float& scale, int& pad_x, int& pad_y)
{
    float sx = (float)target_w / src.cols;
    float sy = (float)target_h / src.rows;
    scale = std::min(sx, sy);

    int new_w = (int)std::round(src.cols * scale);
    int new_h = (int)std::round(src.rows * scale);
    pad_x = (target_w - new_w) / 2;
    pad_y = (target_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, {new_w, new_h});

    cv::Mat out(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return out;
}

// map a point from letterbox space back to original image space
static cv::Point2f unscale_pt(float px, float py, float scale, int pad_x, int pad_y)
{
    return { (px - pad_x) / scale, (py - pad_y) / scale };
}

static std::vector<Detection> postprocess(const float* data, int rows,
                                          float scale, int pad_x, int pad_y,
                                          int orig_w, int orig_h)
{
    // data layout per row: cx cy w h conf cls kp0x kp0y kp0v kp1x kp1y kp1v kp2x kp2y kp2v kp3x kp3y kp3v
    std::vector<cv::Rect2f> boxes;
    std::vector<float>      scores;
    std::vector<int>        class_ids;
    std::vector<std::array<cv::Point2f, 4>> kpts_list;

    for (int i = 0; i < rows; ++i) {
        const float* row = data + i * 18;
        float conf = row[4];
        if (conf < CONF_THRESH) continue;

        float cx = row[0], cy = row[1], w = row[2], h = row[3];
        int   cls = (int)row[5];

        // convert from letterbox coords to original image coords
        cv::Point2f tl_orig = unscale_pt(cx - w * 0.5f, cy - h * 0.5f, scale, pad_x, pad_y);
        float w_orig = w / scale;
        float h_orig = h / scale;

        // clamp
        tl_orig.x = std::max(0.f, std::min(tl_orig.x, (float)orig_w));
        tl_orig.y = std::max(0.f, std::min(tl_orig.y, (float)orig_h));
        w_orig = std::min(w_orig, orig_w - tl_orig.x);
        h_orig = std::min(h_orig, orig_h - tl_orig.y);

        boxes.push_back({tl_orig.x, tl_orig.y, w_orig, h_orig});
        scores.push_back(conf);
        class_ids.push_back(cls);

        std::array<cv::Point2f, 4> kp;
        for (int k = 0; k < 4; ++k) {
            float kx = row[6 + k * 3];
            float ky = row[6 + k * 3 + 1];
            kp[k] = unscale_pt(kx, ky, scale, pad_x, pad_y);
        }
        kpts_list.push_back(kp);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, CONF_THRESH, NMS_THRESH, indices，1.0f,0);

    std::vector<Detection> result;
    result.reserve(indices.size());
    for (int idx : indices) {
        Detection d;
        d.box  = boxes[idx];
        d.conf = scores[idx];
        d.cls  = class_ids[idx];
        for (int k = 0; k < 4; ++k) d.kpts[k] = kpts_list[idx][k];
        result.push_back(d);
    }
    return result;
}

static void draw(cv::Mat& img, const std::vector<Detection>& dets)
{
    // color by team: B=blue, R=red, N=gray, P=purple
    auto team_color = [](int cls) -> cv::Scalar {
        if (cls < 9)  return {255, 100, 0};   // Blue team
        if (cls < 18) return {0, 0, 255};     // Red team
        if (cls < 27) return {160, 160, 160}; // Neutral
        return {200, 0, 200};                  // Purple
    };

    // keypoint order: TL(0), BL(1), BR(2), TR(3)
    static const int KPT_PAIRS[4][2] = {{0,1},{1,2},{2,3},{3,0}};

    for (const auto& d : dets) {
        cv::Scalar color = team_color(d.cls);

        // bounding box
        cv::rectangle(img, d.box, color, 2);

        // label
        std::string label = (d.cls < (int)CLASS_NAMES.size() ? CLASS_NAMES[d.cls] : "?")
                          + " " + cv::format("%.2f", d.conf);
        int baseline = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point label_pt((int)d.box.x, (int)d.box.y - 4);
        cv::rectangle(img,
            {label_pt.x, label_pt.y - ts.height - baseline},
            {label_pt.x + ts.width, label_pt.y + baseline},
            color, cv::FILLED);
        cv::putText(img, label, label_pt,
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,255,255}, 1, cv::LINE_AA);

        // keypoint edges
        for (auto& pair : KPT_PAIRS) {
            cv::line(img,
                cv::Point2i(d.kpts[pair[0]]),
                cv::Point2i(d.kpts[pair[1]]),
                color, 2, cv::LINE_AA);
        }

        // keypoint dots + index labels
        static const cv::Scalar KPT_COLORS[4] = {
            {0,255,255}, {0,255,0}, {255,255,0}, {255,0,255}
        };
        static const char* KPT_LABELS[4] = {"TL","BL","BR","TR"};
        for (int k = 0; k < 4; ++k) {
            cv::circle(img, cv::Point2i(d.kpts[k]), 4, KPT_COLORS[k], -1, cv::LINE_AA);
            cv::putText(img, KPT_LABELS[k], cv::Point2i(d.kpts[k]) + cv::Point(4, -4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, KPT_COLORS[k], 1, cv::LINE_AA);
        }
    }
}

int main(int argc, char** argv)
{
    // ---- load model ----
    ov::Core core;
    std::shared_ptr<ov::Model> model;
    try {
        model = core.read_model(MODEL_PATH);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << e.what() << "\n";
        return 1;
    }

    // dynamic batch → fix to 1
    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input().tensor()
        .set_element_type(ov::element::f32)
        .set_layout("NCHW")
        .set_shape({1, 3, (size_t)INPUT_H, (size_t)INPUT_W});
    ppp.input().model().set_layout("NCHW");
    model = ppp.build();

    ov::CompiledModel compiled = core.compile_model(model, "CPU");
    ov::InferRequest  infer    = compiled.create_infer_request();

    // ---- open camera ----
    int cam_id = (argc > 1) ? std::stoi(argv[1]) : 0;
    cv::VideoCapture cap(cam_id);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Cannot open camera " << cam_id << "\n";
        return 1;
    }
    std::cout << "[INFO] Camera " << cam_id << " opened. Press 'q' to quit.\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // ---- preprocess ----
        float scale; int pad_x, pad_y;
        cv::Mat lb = letterbox(frame, INPUT_W, INPUT_H, scale, pad_x, pad_y);

        cv::Mat blob;
        cv::dnn::blobFromImage(lb, blob, 1.0 / 255.0, {INPUT_W, INPUT_H},
                               cv::Scalar(), true, false, CV_32F);

        // ---- infer ----
        ov::Tensor input_tensor(ov::element::f32,
                                {1, 3, (size_t)INPUT_H, (size_t)INPUT_W},
                                blob.ptr<float>());
        infer.set_input_tensor(input_tensor);
        infer.infer();

        // ---- get output ----
        // expected shape: [1, N, 18]
        auto output = infer.get_output_tensor(0);
        auto shape  = output.get_shape();

        int rows = 0;
        const float* out_data = output.data<float>();

        if (shape.size() == 3) {
            // [1, N, 18]
            rows = (int)shape[1];
        } else if (shape.size() == 2) {
            // [N, 18]
            rows = (int)shape[0];
        } else {
            std::cerr << "[WARN] Unexpected output shape rank: " << shape.size() << "\n";
        }

        // ---- postprocess ----
        auto dets = postprocess(out_data, rows, scale, pad_x, pad_y,
                                frame.cols, frame.rows);

        // ---- visualize ----
        draw(frame, dets);

        cv::putText(frame, cv::format("Detections: %d", (int)dets.size()),
                    {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);

        cv::imshow("minimal_test", frame);
        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
