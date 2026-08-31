// visualize.cpp
// ---------------------------------------------------------------------------
// CPU-only C++ port of CUDA-FastBEV / draw.py post-processing visualiser.
// Reads detections + camera parameters + 6 camera images, writes one
// composite PNG (3 front cams on top, BEV in the middle, 3 back cams on
// the bottom -- the back row is flipped horizontally to match driver POV).
//
// Dependencies: OpenCV >= 4 (core, imgproc, imgcodecs).  No CUDA, no Torch.
//
// Build (Linux, with system OpenCV):
//   g++ -std=c++17 -O2 visualize.cpp -o visualize `pkg-config --cflags --libs opencv4`
// Or with the supplied CMakeLists.txt.
//
// Usage:
//   ./visualize <camera_params.txt> <result.txt> <output.png> [--bev-matrixvt] [--matrixvt-rotate-all]
//
// camera_params.txt is produced by extract_params.py (see README).
// ---------------------------------------------------------------------------

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fastbev_vis {

// ---------------------------------------------------------------------------
// Plain data containers
// ---------------------------------------------------------------------------
struct CameraInfo {
    std::string name;
    std::string img_path;
    cv::Matx44f extrinsic;       // 4x4 lidar -> (intrinsic-applied) image space
    cv::Matx33f intrinsic;       // 3x3 (often identity in CUDA-FastBEV bundle)
    cv::Matx33f post_aug_inv;    // inverse of the 3x3 image-space augmentation
};

struct Detection {
    float x, y, z;        // box centre (lidar frame, bottom of box at z)
    float w, l, h;        // size
    float yaw;            // rotation around z
    int   cls;            // class id (unused for colour right now -- always yellow)
    float score;          // confidence
};

// ---------------------------------------------------------------------------
// I/O
// ---------------------------------------------------------------------------
static std::vector<Detection> load_detections(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open detections file: " + path);

    std::vector<Detection> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::vector<float> values;
        float value = 0.0f;
        while (iss >> value) values.push_back(value);

        // Stable pipeline rows have 9 fields. SA rows add vx/vy before class.
        if (values.size() != 9 && values.size() != 11) continue;
        Detection d;
        d.x = values[0];
        d.y = values[1];
        d.z = values[2];
        d.w = values[3];
        d.l = values[4];
        d.h = values[5];
        d.yaw = values[6];
        const size_t class_index = values.size() == 11 ? 9 : 7;
        d.cls = static_cast<int>(values[class_index]);
        d.score = values[class_index + 1];
        out.push_back(d);
    }
    return out;
}

// Skip whitespace-only and '#'-prefixed lines.  Returns the next "real" line.
static bool next_line(std::istream& f, std::string& out)
{
    while (std::getline(f, out)) {
        auto hash = out.find('#');
        if (hash != std::string::npos) out.erase(hash);
        // trim right
        while (!out.empty() && std::isspace(static_cast<unsigned char>(out.back())))
            out.pop_back();
        // check non-blank
        bool blank = true;
        for (char c : out) if (!std::isspace(static_cast<unsigned char>(c))) { blank = false; break; }
        if (!blank) return true;
    }
    return false;
}

static void parse_floats(const std::string& line, float* dst, int n)
{
    std::istringstream iss(line);
    for (int i = 0; i < n; ++i) {
        if (!(iss >> dst[i]))
            throw std::runtime_error("Expected " + std::to_string(n) +
                                     " floats, got fewer in line: " + line);
    }
}

static std::unordered_map<std::string, CameraInfo>
load_cameras(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open camera params file: " + path);

    std::string line;
    if (!next_line(f, line)) throw std::runtime_error("Empty camera params file");
    int n = std::stoi(line);

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
        for (int r = 0; r < 4; ++r)
            for (int cc = 0; cc < 4; ++cc) c.extrinsic(r, cc) = buf16[r * 4 + cc];

        if (!next_line(f, line)) throw std::runtime_error("Missing intrinsic");
        parse_floats(line, buf9, 9);
        for (int r = 0; r < 3; ++r)
            for (int cc = 0; cc < 3; ++cc) c.intrinsic(r, cc) = buf9[r * 3 + cc];

        cv::Matx33f post_rot;
        if (!next_line(f, line)) throw std::runtime_error("Missing post_rot");
        parse_floats(line, buf9, 9);
        for (int r = 0; r < 3; ++r)
            for (int cc = 0; cc < 3; ++cc) post_rot(r, cc) = buf9[r * 3 + cc];

        if (!next_line(f, line)) throw std::runtime_error("Missing post_tran");
        parse_floats(line, buf3, 3);

        // Build the 3x3 image-space augmentation matrix:
        //   [ post_rot[0:2,0:2]   post_tran[0:2] ]
        //   [        0  0                  1     ]
        cv::Matx33f post_aug = cv::Matx33f::eye();
        post_aug(0, 0) = post_rot(0, 0); post_aug(0, 1) = post_rot(0, 1); post_aug(0, 2) = buf3[0];
        post_aug(1, 0) = post_rot(1, 0); post_aug(1, 1) = post_rot(1, 1); post_aug(1, 2) = buf3[1];
        c.post_aug_inv = post_aug.inv();

        out.emplace(c.name, std::move(c));
    }
    return out;
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------
// Local-frame corner signs that match
// mmdet3d.LiDARInstance3DBoxes(origin=(0.5, 0.5, 0)).corners:
//    0:(-,-, 0)  1:(-,-, h)  2:(-, +, h)  3:(-, +, 0)
//    4:(+,-, 0)  5:(+,-, h)  6:(+, +, h)  7:(+, +, 0)
static const int kCornerSigns[8][3] = {
    {-1, -1, 0}, {-1, -1, 1}, {-1,  1, 1}, {-1,  1, 0},
    { 1, -1, 0}, { 1, -1, 1}, { 1,  1, 1}, { 1,  1, 0},
};

// Returns 8 corners in lidar frame for a single box.
static void compute_corners(const Detection& d, cv::Vec3f corners[8])
{
    const float cy = std::cos(d.yaw);
    const float sy = std::sin(d.yaw);
    for (int i = 0; i < 8; ++i) {
        const float lx = kCornerSigns[i][0] * d.w * 0.5f;
        const float ly = kCornerSigns[i][1] * d.l * 0.5f;
        const float lz = kCornerSigns[i][2] * d.h;
        const float rx = lx * cy - ly * sy;
        const float ry = lx * sy + ly * cy;
        corners[i][0] = d.x + rx;
        corners[i][1] = d.y + ry;
        corners[i][2] = d.z + lz;
    }
}

static cv::Vec3f rotate_matrixvt_point_for_display(const cv::Vec3f& p)
{
    return cv::Vec3f(-p[1], p[0], p[2]);
}

// Project lidar points to image plane following the exact same chain as
// draw.py:lidar2img -- namely:
//   1. p_cam = extrinsic * (p_lidar; 1)
//   2. valid := (p_cam.z > 0.5)
//   3. p_norm = p_cam / p_cam.z
//   4. p_img  = intrinsic * p_norm
//   5. p_out  = inv(post_aug) * p_img      (and we keep the first two coords)
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

        valid[i] = (cz > 0.5f) ? 1u : 0u;
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

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
static cv::Scalar depth_to_color(float depth)
{
    float gray = std::max(0.0f, std::min((depth + 2.5f) / 3.0f, 1.0f));
    constexpr float L = 200.0f;
    // R, G, B (we'll convert to BGR at the end)
    static const float palette[6][3] = {
        {L,  0,  L},
        {L,  0,  0},
        {L,  L,  0},
        {0,  L,  0},
        {0,  L,  L},
        {0,  0,  L},
    };
    float r, g, b;
    if (gray >= 1.0f) {
        r = palette[5][0]; g = palette[5][1]; b = palette[5][2];
    } else {
        const int n = 5;
        const int rank = static_cast<int>(std::floor(gray * n));
        const float diff = (gray - static_cast<float>(rank) / n) * n;
        r = palette[rank][0] + (palette[rank + 1][0] - palette[rank][0]) * diff;
        g = palette[rank][1] + (palette[rank + 1][1] - palette[rank][1]) * diff;
        b = palette[rank][2] + (palette[rank + 1][2] - palette[rank][2]) * diff;
    }
    return cv::Scalar(b, g, r);  // OpenCV is BGR
}

// Edge tables.
static const int kEdges3D[12][2] = {
    {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
};
static const int kEdgesBev[4][2] = { {0,1},{1,2},{2,3},{3,0} };

// In draw.py: color_map = {0:(255,255,0), 1:(0,255,255)} and pred_flag is
// always 1, so every box uses BGR(0,255,255) -- yellow.
static const cv::Scalar kBoxColor(0, 255, 255);

// Camera names in display order.
static const std::array<const char*, 6> kViews = {
    "CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",  "CAM_BACK",  "CAM_BACK_RIGHT"
};

// ---------------------------------------------------------------------------
// Main render
// ---------------------------------------------------------------------------
static int run(const std::string& cams_path,
               const std::string& result_path,
               const std::string& out_path,
               bool matrixvt_bev,
               bool matrixvt_rotate_all)
{
    constexpr int   kCanvas      = 1000;
    constexpr float kShowRange   = 50.0f;
    constexpr int   kScaleFactor = 4;
    constexpr int   kImgW        = 1600;
    constexpr int   kImgH        = 900;

    auto cams = load_cameras(cams_path);
    auto dets = load_detections(result_path);
    std::printf("Loaded %zu detections, %zu cameras.\n", dets.size(), cams.size());

    // Pre-compute all corners (Nx8 in lidar frame).
    std::vector<cv::Vec3f> corners_flat;
    corners_flat.reserve(dets.size() * 8);
    for (const auto& d : dets) {
        cv::Vec3f c8[8];
        compute_corners(d, c8);
        for (int k = 0; k < 8; ++k) {
            corners_flat.push_back(matrixvt_rotate_all ? rotate_matrixvt_point_for_display(c8[k])
                                                       : c8[k]);
        }
    }

    // ---- per-camera image rendering ----
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
            for (auto& e : kEdges3D) {
                const size_t i0 = a * 8 + e[0];
                const size_t i1 = a * 8 + e[1];
                if (valid[i0] && valid[i1]) {
                    const cv::Point p0(static_cast<int>(pix[i0].x),
                                       static_cast<int>(pix[i0].y));
                    const cv::Point p1(static_cast<int>(pix[i1].x),
                                       static_cast<int>(pix[i1].y));
                    cv::line(img, p0, p1, kBoxColor, kScaleFactor);
                }
            }
        }
        rendered.push_back(std::move(img));
    }

    // ---- BEV canvas ----
    cv::Mat canvas = cv::Mat::zeros(kCanvas, kCanvas, CV_8UC3);
    const int cc = static_cast<int>((0.0f + kShowRange) / kShowRange / 2.0f * kCanvas);
    cv::circle(canvas, cv::Point(cc, cc), 1, cv::Scalar(255, 255, 255), 0);
    for (int r = 10; r < 100; r += 10) {
        const int rc = static_cast<int>(r / kShowRange / 2.0f * kCanvas);
        cv::circle(canvas, cv::Point(cc, cc), rc, depth_to_color(r), 1);
    }

    // result.txt already contains thresholded and NMS-filtered detections.
    // Draw the same set in BEV as in the six camera views; applying another
    // score threshold here made the two regions disagree for extended SA rows.
    std::vector<size_t> order(dets.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) { return dets[a].score < dets[b].score; });

    auto bev_display_xy = [&](const cv::Vec3f& p) -> cv::Point2f {
        if (matrixvt_bev) {
            // MatrixVT follows nuScenes lidar axes: x forward, y left. The
            // FastBEV canvas draws horizontal right and vertical forward-up,
            // so remap only the BEV display while camera projection stays in
            // the native lidar frame.
            return cv::Point2f(-p[1], -p[0]);
        }
        return cv::Point2f(p[0], -p[1]);
    };

    auto world_to_canvas_round = [&](float wx, float wy_flipped) -> cv::Point {
        // draw.py applies np.round before astype(int32) for bottom corners.
        const long cx = std::lround((wx         + kShowRange) / kShowRange / 2.0f * kCanvas);
        const long cy = std::lround((wy_flipped + kShowRange) / kShowRange / 2.0f * kCanvas);
        return cv::Point(static_cast<int>(cx), static_cast<int>(cy));
    };
    auto world_to_canvas_trunc = [&](float wx, float wy_flipped) -> cv::Point {
        // draw.py uses .astype(int32) (truncation toward zero) for centre/head.
        const int cx = static_cast<int>((wx         + kShowRange) / kShowRange / 2.0f * kCanvas);
        const int cy = static_cast<int>((wy_flipped + kShowRange) / kShowRange / 2.0f * kCanvas);
        return cv::Point(cx, cy);
    };

    size_t bev_drawn = 0;
    for (size_t rid : order) {
        // Bottom 4 corners with Y flipped (indices 0,3,7,4 in lidar order).
        cv::Vec3f c8[8];
        compute_corners(dets[rid], c8);
        if (matrixvt_rotate_all) {
            for (auto& p : c8) p = rotate_matrixvt_point_for_display(p);
        }
        const int bidx[4] = {0, 3, 7, 4};
        cv::Point bot[4];
        float cx_w = 0, cy_w = 0;
        for (int k = 0; k < 4; ++k) {
            const cv::Point2f bev = bev_display_xy(c8[bidx[k]]);
            bot[k] = world_to_canvas_round(bev.x, bev.y);
            cx_w += bev.x;
            cy_w += bev.y;
        }
        cx_w /= 4.0f; cy_w /= 4.0f;

        const cv::Point centre = world_to_canvas_trunc(cx_w, cy_w);
        const cv::Point2f h0 = bev_display_xy(c8[0]);
        const cv::Point2f h4 = bev_display_xy(c8[4]);
        const cv::Point head = world_to_canvas_trunc(
            (h0.x + h4.x) * 0.5f,
            (h0.y + h4.y) * 0.5f);

        for (auto& e : kEdgesBev)
            cv::line(canvas, bot[e[0]], bot[e[1]], kBoxColor, 1);
        cv::line(canvas, centre, head, kBoxColor, 1, cv::LINE_8);
        ++bev_drawn;
    }
    std::printf("BEV drew %zu/%zu detections (no secondary score filter).\n",
                bev_drawn, dets.size());

    // ---- compose ----
    const int H = kImgH * 2 + kCanvas * kScaleFactor;
    const int W = kImgW * 3;
    cv::Mat big = cv::Mat::zeros(H, W, CV_8UC3);

    // Top row: front cameras at full resolution.
    for (int k = 0; k < 3; ++k) {
        cv::Mat dst = big(cv::Rect(k * kImgW, 0, kImgW, kImgH));
        rendered[k].copyTo(dst);
    }
    // Bottom row: back cameras flipped horizontally.
    for (int k = 0; k < 3; ++k) {
        cv::Mat flipped;
        cv::flip(rendered[3 + k], flipped, /*flipCode=*/1);
        cv::Mat dst = big(cv::Rect(k * kImgW, H - kImgH, kImgW, kImgH));
        flipped.copyTo(dst);
    }

    cv::Mat resized;
    const int out_w = kImgW / kScaleFactor * 3;             // 1200
    const int out_h = kImgH / kScaleFactor * 2 + kCanvas;   // 1450
    cv::resize(big, resized, cv::Size(out_w, out_h));

    const int w_begin = (kImgW * 3 / kScaleFactor - kCanvas) / 2;  // 100
    const int y_begin = kImgH / kScaleFactor;                       // 225
    cv::Mat dst = resized(cv::Rect(w_begin, y_begin, kCanvas, kCanvas));
    canvas.copyTo(dst);

    if (!cv::imwrite(out_path, resized)) {
        std::cerr << "Failed to write " << out_path << "\n";
        return 1;
    }
    std::printf("Saved %s  (%dx%d)\n", out_path.c_str(), out_w, out_h);
    return 0;
}

}  // namespace fastbev_vis

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::fprintf(stderr,
                     "Usage: %s <camera_params.txt> <result.txt> <output.png> [--bev-matrixvt] [--matrixvt-rotate-all]\n",
                     argv[0]);
        return 1;
    }
    bool matrixvt_bev = false;
    bool matrixvt_rotate_all = false;
    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--bev-matrixvt") {
            matrixvt_bev = true;
        } else if (arg == "--matrixvt-rotate-all") {
            matrixvt_bev = true;
            matrixvt_rotate_all = true;
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        }
    }
    try {
        return fastbev_vis::run(argv[1], argv[2], argv[3], matrixvt_bev, matrixvt_rotate_all);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
