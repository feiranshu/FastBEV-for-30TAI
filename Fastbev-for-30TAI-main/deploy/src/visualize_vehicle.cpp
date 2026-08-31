// visualize_vehicle.cpp
// ---------------------------------------------------------------------------
// CPU-only C++ port of the vehicle sandbox FastBEV visualizer.
// Reads vehicle detections + fixed camera parameters + 6 camera images, writes
// one composite PNG: 3 front cameras on top, BEV in the middle, and 3 rear
// cameras on the bottom with the rear row flipped horizontally for driver POV.
//
// Dependencies: OpenCV >= 4 (core, imgproc, imgcodecs). No CUDA, no Torch.
//
// Usage:
//   ./visualize_vehicle <camera_params.txt> <result.txt> <output.png>
//
// result.txt format:
//   x y z length width height yaw class_id score
//
// camera_params.txt is produced by the vehicle sandbox generate_camera_params.py.
// ---------------------------------------------------------------------------

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fastbev_vehicle_vis {

struct CameraInfo {
    std::string name;
    std::string img_path;
    cv::Matx44f extrinsic;       // 4x4 lidar -> image/camera projection space
    cv::Matx33f intrinsic;       // 3x3 intrinsic, often identity in sandbox bundle
    cv::Matx33f post_aug_inv;    // inverse of image-space augmentation
};

struct Detection {
    float x, y, z;        // box center in lidar frame; z is bottom
    float length, width, height;
    float yaw;            // rotation around z
    int cls;              // vehicle sandbox currently visualizes class 0 only
    float score;          // confidence
};

static std::vector<Detection> load_detections(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open detections file: " + path);

    std::vector<Detection> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        Detection d;
        float cls_f;
        if (iss >> d.x >> d.y >> d.z >> d.length >> d.width >> d.height
                >> d.yaw >> cls_f >> d.score) {
            d.cls = static_cast<int>(cls_f);
            out.push_back(d);
        }
    }
    return out;
}

static bool next_line(std::istream& f, std::string& out)
{
    while (std::getline(f, out)) {
        const auto hash = out.find('#');
        if (hash != std::string::npos) out.erase(hash);
        while (!out.empty() && std::isspace(static_cast<unsigned char>(out.back()))) {
            out.pop_back();
        }

        bool blank = true;
        for (char c : out) {
            if (!std::isspace(static_cast<unsigned char>(c))) {
                blank = false;
                break;
            }
        }
        if (!blank) return true;
    }
    return false;
}

static void parse_floats(const std::string& line, float* dst, int n)
{
    std::istringstream iss(line);
    for (int i = 0; i < n; ++i) {
        if (!(iss >> dst[i])) {
            throw std::runtime_error("Expected " + std::to_string(n) +
                                     " floats, got fewer in line: " + line);
        }
    }
}

static std::unordered_map<std::string, CameraInfo> load_cameras(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open camera params file: " + path);

    std::string line;
    if (!next_line(f, line)) throw std::runtime_error("Empty camera params file");
    const int n = std::stoi(line);

    std::unordered_map<std::string, CameraInfo> out;
    for (int i = 0; i < n; ++i) {
        CameraInfo c;
        if (!next_line(f, line)) throw std::runtime_error("Missing camera name");
        c.name = line;
        if (!next_line(f, line)) throw std::runtime_error("Missing image path");
        c.img_path = line;

        float buf16[16], buf9[9], buf3[3];

        if (!next_line(f, line)) throw std::runtime_error("Missing extrinsic");
        parse_floats(line, buf16, 16);
        for (int r = 0; r < 4; ++r) {
            for (int cc = 0; cc < 4; ++cc) c.extrinsic(r, cc) = buf16[r * 4 + cc];
        }

        if (!next_line(f, line)) throw std::runtime_error("Missing intrinsic");
        parse_floats(line, buf9, 9);
        for (int r = 0; r < 3; ++r) {
            for (int cc = 0; cc < 3; ++cc) c.intrinsic(r, cc) = buf9[r * 3 + cc];
        }

        cv::Matx33f post_rot;
        if (!next_line(f, line)) throw std::runtime_error("Missing post_rot");
        parse_floats(line, buf9, 9);
        for (int r = 0; r < 3; ++r) {
            for (int cc = 0; cc < 3; ++cc) post_rot(r, cc) = buf9[r * 3 + cc];
        }

        if (!next_line(f, line)) throw std::runtime_error("Missing post_tran");
        parse_floats(line, buf3, 3);

        cv::Matx33f post_aug = cv::Matx33f::eye();
        post_aug(0, 0) = post_rot(0, 0);
        post_aug(0, 1) = post_rot(0, 1);
        post_aug(0, 2) = buf3[0];
        post_aug(1, 0) = post_rot(1, 0);
        post_aug(1, 1) = post_rot(1, 1);
        post_aug(1, 2) = buf3[1];
        c.post_aug_inv = post_aug.inv();

        out.emplace(c.name, std::move(c));
    }
    return out;
}

// Exact order used by task/FastBEV4.0_sandbox/visualize/visualize_fastbev_custom.cpp.
// z is bottom, and BEV bottom face uses corners [0, 3, 7, 4].
static const int kCornerSigns[8][3] = {
    {-1, -1, 0}, {-1, -1, 1}, {-1,  1, 1}, {-1,  1, 0},
    { 1, -1, 0}, { 1, -1, 1}, { 1,  1, 1}, { 1,  1, 0},
};

static void compute_corners(const Detection& d, cv::Vec3f corners[8])
{
    const float cy = std::cos(d.yaw);
    const float sy = std::sin(d.yaw);
    for (int i = 0; i < 8; ++i) {
        const float lx = kCornerSigns[i][0] * d.length * 0.5f;
        const float ly = kCornerSigns[i][1] * d.width * 0.5f;
        const float lz = kCornerSigns[i][2] * d.height;
        const float rx = lx * cy - ly * sy;
        const float ry = lx * sy + ly * cy;
        corners[i][0] = d.x + rx;
        corners[i][1] = d.y + ry;
        corners[i][2] = d.z + lz;
    }
}

static void project(const std::vector<cv::Vec3f>& pts,
                    const CameraInfo& cam,
                    std::vector<cv::Point2f>& out_pts,
                    std::vector<unsigned char>& valid)
{
    const auto& E = cam.extrinsic;
    const auto& K = cam.intrinsic;
    const auto& Pi = cam.post_aug_inv;

    out_pts.resize(pts.size());
    valid.assign(pts.size(), 0);

    for (size_t i = 0; i < pts.size(); ++i) {
        const float px = pts[i][0], py = pts[i][1], pz = pts[i][2];

        float cx = E(0, 0) * px + E(0, 1) * py + E(0, 2) * pz + E(0, 3);
        float cy = E(1, 0) * px + E(1, 1) * py + E(1, 2) * pz + E(1, 3);
        float cz = E(2, 0) * px + E(2, 1) * py + E(2, 2) * pz + E(2, 3);

        valid[i] = (cz > 0.05f) ? 1u : 0u;
        if (std::fabs(cz) < 1e-6f) cz = (cz < 0 ? -1e-6f : 1e-6f);

        const float nx = cx / cz;
        const float ny = cy / cz;
        const float nz = 1.0f;

        const float ix = K(0, 0) * nx + K(0, 1) * ny + K(0, 2) * nz;
        const float iy = K(1, 0) * nx + K(1, 1) * ny + K(1, 2) * nz;
        const float iz = K(2, 0) * nx + K(2, 1) * ny + K(2, 2) * nz;

        const float ox = Pi(0, 0) * ix + Pi(0, 1) * iy + Pi(0, 2) * iz;
        const float oy = Pi(1, 0) * ix + Pi(1, 1) * iy + Pi(1, 2) * iz;
        out_pts[i] = cv::Point2f(ox, oy);
    }
}

static cv::Scalar score_color(float score)
{
    const float strength = std::max(0.0f, std::min(1.0f, score));
    return cv::Scalar(0, static_cast<int>(180 + 75 * strength),
                      static_cast<int>(80 + 175 * strength));
}

static const int kEdges3D[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7},
};
static const int kEdgesBev[4][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
};

static const cv::Scalar kBoxColor(0, 255, 255);

static const std::array<const char*, 6> kViews = {
    "CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",  "CAM_BACK",  "CAM_BACK_RIGHT",
};

static int run(const std::string& cams_path,
               const std::string& result_path,
               const std::string& out_path)
{
    constexpr int kCanvas = 960;
    constexpr float kShowRange = 72.0f;
    constexpr float kScoreThresh = 0.6f;
    constexpr int kImgW = 640;
    constexpr int kImgH = 480;
    constexpr float kWorldScale = 24.0f;

    auto cams = load_cameras(cams_path);
    auto dets = load_detections(result_path);
    std::printf("Loaded %zu detections, %zu cameras.\n", dets.size(), cams.size());

    std::vector<cv::Vec3f> corners_flat;
    corners_flat.reserve(dets.size() * 8);
    for (const auto& d : dets) {
        cv::Vec3f c8[8];
        compute_corners(d, c8);
        for (int k = 0; k < 8; ++k) corners_flat.push_back(c8[k]);
    }

    std::vector<cv::Mat> rendered;
    rendered.reserve(kViews.size());
    for (const auto* view_name : kViews) {
        auto it = cams.find(view_name);
        if (it == cams.end()) {
            std::cerr << "Missing camera: " << view_name << "\n";
            return 1;
        }
        const CameraInfo& cam = it->second;

        cv::Mat img = cv::imread(cam.img_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::fprintf(stderr,
                         "[warn] cannot open %s -- substituting blank frame\n",
                         cam.img_path.c_str());
            img = cv::Mat::zeros(kImgH, kImgW, CV_8UC3);
        }

        std::vector<cv::Point2f> pix;
        std::vector<unsigned char> valid;
        project(corners_flat, cam, pix, valid);

        for (size_t a = 0; a < dets.size(); ++a) {
            if (dets[a].score < kScoreThresh || dets[a].cls != 0) continue;
            const cv::Scalar color = score_color(dets[a].score);
            for (const auto& e : kEdges3D) {
                const size_t i0 = a * 8 + e[0];
                const size_t i1 = a * 8 + e[1];
                if (valid[i0] && valid[i1]) {
                    const cv::Point p0(static_cast<int>(pix[i0].x),
                                       static_cast<int>(pix[i0].y));
                    const cv::Point p1(static_cast<int>(pix[i1].x),
                                       static_cast<int>(pix[i1].y));
                    cv::line(img, p0, p1, color, 2, cv::LINE_AA);
                }
            }
            std::vector<cv::Point> valid_pixels;
            valid_pixels.reserve(8);
            for (int i = 0; i < 8; ++i) {
                const size_t idx = a * 8 + i;
                if (valid[idx]) {
                    valid_pixels.emplace_back(static_cast<int>(pix[idx].x),
                                              static_cast<int>(pix[idx].y));
                }
            }
            if (!valid_pixels.empty()) {
                auto anchor_it = std::min_element(
                    valid_pixels.begin(), valid_pixels.end(),
                    [](const cv::Point& lhs, const cv::Point& rhs) {
                        return lhs.y < rhs.y;
                    });
                cv::Point anchor = *anchor_it;
                anchor.x = std::max(5, std::min(anchor.x, img.cols - 150));
                anchor.y = std::max(18, std::min(anchor.y - 4, img.rows - 5));
                char label[64];
                std::snprintf(label, sizeof(label), "car %.2f", dets[a].score);
                cv::putText(img, label, anchor, cv::FONT_HERSHEY_SIMPLEX,
                            0.48, color, 1, cv::LINE_AA);
            }
        }
        cv::rectangle(img, cv::Rect(0, 0, 250, 34), cv::Scalar(0, 0, 0), -1);
        cv::putText(img, view_name, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        rendered.push_back(std::move(img));
    }

    cv::Mat canvas(kCanvas, kCanvas, CV_8UC3, cv::Scalar(18, 18, 18));
    const int center = kCanvas / 2;
    for (float coordinate = -kShowRange; coordinate <= kShowRange + 0.1f;
         coordinate += kWorldScale) {
        const int pixel = static_cast<int>(std::lround(
            (coordinate + kShowRange) / (2.0f * kShowRange) * kCanvas));
        cv::line(canvas, cv::Point(pixel, 0), cv::Point(pixel, kCanvas - 1),
                 cv::Scalar(38, 38, 38), 1);
        cv::line(canvas, cv::Point(0, kCanvas - pixel), cv::Point(kCanvas - 1, kCanvas - pixel),
                 cv::Scalar(38, 38, 38), 1);
    }
    for (int physical_radius = 1; physical_radius <= 3; ++physical_radius) {
        const int radius = static_cast<int>(std::lround(
            physical_radius * kWorldScale / (2.0f * kShowRange) * kCanvas));
        cv::circle(canvas, cv::Point(center, center), radius,
                   cv::Scalar(75, 75, 75), 1, cv::LINE_AA);
        cv::putText(canvas, std::to_string(physical_radius) + "m",
                    cv::Point(center + radius + 4, center - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    }
    cv::arrowedLine(canvas, cv::Point(center, center), cv::Point(center, center - 70),
                    cv::Scalar(70, 70, 255), 2);
    cv::arrowedLine(canvas, cv::Point(center, center), cv::Point(center - 70, center),
                    cv::Scalar(70, 255, 70), 2);
    cv::putText(canvas, "+X", cv::Point(center + 4, center - 74),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(70, 70, 255), 1);
    cv::putText(canvas, "+Y", cv::Point(center - 94, center + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(70, 255, 70), 1);

    std::vector<size_t> order(dets.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) { return dets[a].score < dets[b].score; });

    auto world_to_canvas_round = [&](float wx, float wy) -> cv::Point {
        const long cx = std::lround((-wy + kShowRange) / (2.0f * kShowRange) * kCanvas);
        const long cy = std::lround((-wx + kShowRange) / (2.0f * kShowRange) * kCanvas);
        return cv::Point(static_cast<int>(cx), static_cast<int>(cy));
    };
    auto world_to_canvas_trunc = [&](float wx, float wy) -> cv::Point {
        const int cx = static_cast<int>((-wy + kShowRange) / (2.0f * kShowRange) * kCanvas);
        const int cy = static_cast<int>((-wx + kShowRange) / (2.0f * kShowRange) * kCanvas);
        return cv::Point(cx, cy);
    };

    for (size_t rid : order) {
        const float s = dets[rid].score;
        if (s < kScoreThresh || dets[rid].cls != 0) continue;
        const cv::Scalar col = score_color(s);

        cv::Vec3f c8[8];
        compute_corners(dets[rid], c8);
        const int bidx[4] = {0, 3, 7, 4};
        cv::Point bot[4];
        float cx_w = 0.0f;
        float cy_w = 0.0f;
        for (int k = 0; k < 4; ++k) {
            const float wx = c8[bidx[k]][0];
            const float wy = c8[bidx[k]][1];
            bot[k] = world_to_canvas_round(wx, wy);
            cx_w += wx;
            cy_w += wy;
        }
        cx_w /= 4.0f;
        cy_w /= 4.0f;

        for (const auto& e : kEdgesBev) {
            cv::line(canvas, bot[e[0]], bot[e[1]], col, 3, cv::LINE_AA);
        }
        const cv::Point centre = world_to_canvas_trunc(cx_w, cy_w);
        const cv::Point head = world_to_canvas_trunc(
            (c8[0][0] + c8[4][0]) * 0.5f,
            (c8[0][1] + c8[4][1]) * 0.5f);
        cv::arrowedLine(canvas, centre, head, col, 2, cv::LINE_AA);
        auto text_it = std::min_element(
            std::begin(bot), std::end(bot),
            [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.y < rhs.y; });
        char score_text[32];
        std::snprintf(score_text, sizeof(score_text), "%.2f", s);
        cv::putText(canvas, score_text, *text_it, cv::FONT_HERSHEY_SIMPLEX,
                    0.48, col, 1, cv::LINE_AA);
    }
    const size_t drawn_count = std::count_if(
        dets.begin(), dets.end(),
        [](const Detection& d) { return d.score >= kScoreThresh && d.cls == 0; });
    char det_text[96];
    std::snprintf(det_text, sizeof(det_text), "detections=%zu  scale=%gx",
                  drawn_count, static_cast<double>(kWorldScale));
    cv::putText(canvas, det_text, cv::Point(14, 28), cv::FONT_HERSHEY_SIMPLEX,
                0.62, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    const int H = kImgH * 2 + kCanvas;
    const int W = kImgW * 3;
    cv::Mat big = cv::Mat::zeros(H, W, CV_8UC3);

    for (int k = 0; k < 3; ++k) {
        cv::Mat dst = big(cv::Rect(k * kImgW, 0, kImgW, kImgH));
        rendered[k].copyTo(dst);
    }
    for (int k = 0; k < 3; ++k) {
        cv::Mat flipped;
        cv::flip(rendered[3 + k], flipped, 1);
        cv::Mat dst = big(cv::Rect(k * kImgW, H - kImgH, kImgW, kImgH));
        flipped.copyTo(dst);
    }

    const int w_begin = (W - kCanvas) / 2;
    const int y_begin = kImgH;
    cv::Mat dst = big(cv::Rect(w_begin, y_begin, kCanvas, kCanvas));
    canvas.copyTo(dst);

    if (!cv::imwrite(out_path, big)) {
        std::cerr << "Failed to write " << out_path << "\n";
        return 1;
    }
    std::printf("Saved %s  (%dx%d)\n", out_path.c_str(), W, H);
    return 0;
}

}  // namespace fastbev_vehicle_vis

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::fprintf(stderr,
                     "Usage: %s <camera_params.txt> <result.txt> <output.png>\n",
                     argv[0]);
        return 1;
    }
    try {
        return fastbev_vehicle_vis::run(argv[1], argv[2], argv[3]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
