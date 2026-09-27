// task3_windmill: 真实能量机关多目标识别与稳定跟踪（防中心发散+极坐标预测+长寿命INACTIVE）
// 核心修复：1. 废除速度外推，单目标无R标时冻结中心；2. 极坐标预测匹配；3. 放宽INACTIVE匹配门限。
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;
using namespace cv;

inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// ---------------- 调色板 ----------------
struct Palette { Scalar line_lo, line_hi, bar_lo, bar_hi; };
static const Palette kPalOrange{ Scalar(5, 120, 80),   Scalar(20, 255, 255),
                                 Scalar(20, 120, 150), Scalar(45, 255, 255) };
static const Palette kPalBlue  { Scalar(100, 120, 80), Scalar(135, 255, 255),
                                 Scalar(80, 120, 150), Scalar(100, 255, 255) };

// ---------------- 判参 ----------------
static const double kMinR = 18.0, kMaxR = 55.0;     
static const double kCoverMin = 0.65;               
static const int    kInnerReticle = 50;             
static const int    kLostToleranceActive = 60;      // ACTIVE 目标丢失容忍度
static const int    kLostToleranceInactive = 150;   // INACTIVE 目标丢失容忍度（大幅延长）
static const int    kProbeFrames = 60;

// R 标检测参数
static const int    kRAreaLo = 40, kRAreaHi = 350;  
static const double kRSearchRadius = 100.0;         // 放宽搜索半径
static const double kRMaxStep = 80.0;               // R 标帧间最大移动距离

struct Candidate {
    Point2d c; double r = 0; int type = 0;  
    int inner_pixels = 0;
};

enum class TrackState { ACTIVE, INACTIVE, LOST };

struct TrackedObject {
    int id = -1;
    Point2d center;
    double radius = 0;
    double theta = 0;
    double orbit_radius = 0; // 轨道半径（到旋转中心的距离）
    double last_omega = 0.0; // 历史角速度
    int type = 0; 
    TrackState state = TrackState::LOST;
    int lost_count = 0;
    bool is_permanent_inactive = false; // 标记为永久失效
};

// ---------------- 预处理器 ----------------
class Preprocessor {
public:
    void init() { std::cout << "[INFO] backend: CPU\n"; }
    void probe(const Mat& bgr, double& warm_px, double& blue_px) {
        warm_px = blue_px = 0;
        Mat hsv, a, b;
        cvtColor(bgr, hsv, COLOR_BGR2HSV);
        inRange(hsv, kPalOrange.line_lo, kPalOrange.line_hi, a);
        inRange(hsv, kPalOrange.bar_lo,  kPalOrange.bar_hi,  b);
        bitwise_or(a, b, a); warm_px = countNonZero(a);
        inRange(hsv, kPalBlue.line_lo, kPalBlue.line_hi, a);
        inRange(hsv, kPalBlue.bar_lo,  kPalBlue.bar_hi,  b);
        bitwise_or(a, b, a); blue_px = countNonZero(a);
    }
    void masks(const Mat& bgr, const Palette& p, Mat& line_m, Mat& bar_m) {
        Mat hsv;
        cvtColor(bgr, hsv, COLOR_BGR2HSV);
        inRange(hsv, p.line_lo, p.line_hi, line_m);
        inRange(hsv, p.bar_lo,  p.bar_hi,  bar_m);
    }
};

// ---------------- 工具 ----------------
static double ringCoverage(const Mat& m, const Point2d& c, double r) {
    const int N = 36; int hit = 0; 
    for (int i = 0; i < N; ++i) {
        double a = 2 * CV_PI * i / N;
        bool ok = false;
        for (int dr = -2; dr <= 2 && !ok; ++dr) {
            int x = (int)std::round(c.x + (r + dr) * std::cos(a));
            int y = (int)std::round(c.y + (r + dr) * std::sin(a));
            if (x >= 0 && y >= 0 && x < m.cols && y < m.rows && m.at<uchar>(y, x)) ok = true;
        }
        if (ok) ++hit;
    }
    return (double)hit / N;
}

static int innerCount(const Mat& m, const Point2d& c, double r) {
    int R = (int)std::ceil(r);
    Rect roi((int)c.x - R, (int)c.y - R, 2 * R + 1, 2 * R + 1);
    roi &= Rect(0, 0, m.cols, m.rows);
    if (roi.area() <= 0) return 0;
    Mat disk = Mat::zeros(roi.size(), CV_8UC1);
    circle(disk, Point((int)c.x - roi.x, (int)c.y - roi.y), (int)(r * 0.5), Scalar(255), -1);
    Mat inter; bitwise_and(m(roi), disk, inter);
    return countNonZero(inter);
}

// ---------------- 主程序 ----------------
int main(int argc, char** argv) {
    std::string input, outbase = "result/task3_windmill";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--input" && i + 1 < argc) input = argv[++i];
        else if (input.empty()) input = a;
    }
    if (input.empty()) { std::cerr << "usage: --input <mp4>\n"; return 1; }
    
    std::string tag = fs::path(input).stem().string();
    std::string outdir = outbase + "/" + tag;
    fs::create_directories(outdir);
    
    VideoCapture cap(input);
    if (!cap.isOpened()) { std::cerr << "Cannot open " << input << "\n"; return 1; }
    
    const double fps = cap.get(CAP_PROP_FPS) > 0 ? cap.get(CAP_PROP_FPS) : 60.0;
    const int n_frames = (int)cap.get(CAP_PROP_FRAME_COUNT);
    const Size vsize((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));
    
    Preprocessor pre; pre.init();
    
    VideoWriter w_ov(outdir + "/recognition_overlay.mp4",
                     VideoWriter::fourcc('m', 'p', '4', 'v'), fps, vsize);
    
    Palette pal = kPalOrange; bool pal_fixed = false, pal_orange = true;
    double warm_sum = 0, blue_sum = 0;
    
    Point2d center(vsize.width / 2.0, vsize.height / 2.0);
    Point2d r_mark_pos(-1, -1);
    
    int next_id = 0;
    std::vector<TrackedObject> tracks;
    
    Mat frame, m_line, m_bar, proc;
    int idx = 0;
    
    while (cap.read(frame)) {
        if (!pal_fixed) {
            double w, b; pre.probe(frame, w, b);
            warm_sum += w; blue_sum += b;
            if (idx == kProbeFrames - 1) {
                pal_orange = (warm_sum >= blue_sum);
                pal = pal_orange ? kPalOrange : kPalBlue;
                pal_fixed = true;
            }
        }
        
        pre.masks(frame, pal, m_line, m_bar);
        morphologyEx(m_line, proc, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        
        // ---- 1. 候选检测 ----
        std::vector<Candidate> cands;
        {
            std::vector<Vec3f> circles;
            HoughCircles(proc, circles, HOUGH_GRADIENT, 1.0, kMinR * 0.8, 60, 20, (int)kMinR, (int)kMaxR);
            for (auto& v : circles) {
                Point2d c(v[0], v[1]); double r = v[2];
                if (ringCoverage(proc, c, r) < kCoverMin) continue;
                int inner = innerCount(proc, c, r);
                Candidate k; k.c = c; k.r = r; k.inner_pixels = inner;
                k.type = (inner >= kInnerReticle) ? 1 : 0; 
                cands.push_back(k);
            }
        }
        if ((int)cands.size() < 3) {
            std::vector<std::vector<Point>> contours;
            findContours(proc.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (auto& cnt : contours) {
                if (contourArea(cnt) < 80) continue;
                Point2f cc; float rr;
                minEnclosingCircle(cnt, cc, rr);
                if (rr < kMinR || rr > kMaxR) continue;
                if (ringCoverage(proc, cc, rr) < kCoverMin * 0.6) continue;
                bool dup = false;
                for (auto& k : cands) if (std::hypot(k.c.x - cc.x, k.c.y - cc.y) < 10.0) { dup = true; break; }
                if (dup) continue;
                Candidate k; k.c = cc; k.r = rr;
                k.inner_pixels = innerCount(proc, cc, rr);
                k.type = (k.inner_pixels >= kInnerReticle) ? 1 : 0;
                cands.push_back(k);
            }
        }
        
        // NMS 去重
        std::vector<Candidate> nms_cands;
        std::vector<bool> used(cands.size(), false);
        for (size_t i = 0; i < cands.size(); ++i) {
            if (used[i]) continue;
            Candidate best = cands[i]; used[i] = true;
            for (size_t j = i + 1; j < cands.size(); ++j) {
                if (used[j]) continue;
                if (std::hypot(cands[i].c.x - cands[j].c.x, cands[i].c.y - cands[j].c.y) < 0.6 * cands[i].r) {
                    used[j] = true;
                    if (cands[j].inner_pixels > best.inner_pixels) best = cands[j];
                }
            }
            nms_cands.push_back(best);
        }
        cands = nms_cands;
        
        // ---- 2. 中心估计（防发散策略） ----
        r_mark_pos = Point2d(-1, -1);
        {
            std::vector<std::vector<Point>> cs; std::vector<Vec4i> h;
            findContours(proc.clone(), cs, h, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            double best_score = 1e18;
            for (size_t ci = 0; ci < cs.size(); ++ci) {
                if (h[ci][3] != -1) continue;
                double a = contourArea(cs[ci]);
                if (a < kRAreaLo || a > kRAreaHi) continue;
                double p = arcLength(cs[ci], true);
                if (p <= 0) continue;
                if (4 * CV_PI * a / (p * p) > 0.65) continue;
                Rect bb = boundingRect(cs[ci]);
                double asp = (bb.height > 0) ? (double)bb.width / bb.height : 0;
                if (asp < 0.3 || asp > 3.0) continue;
                
                Moments m = moments(cs[ci]);
                Point2d pt(m.m10 / m.m00, m.m01 / m.m00);
                
                double dist_to_last = std::hypot(pt.x - center.x, pt.y - center.y);
                if (dist_to_last > kRMaxStep) continue;
                
                bool is_blade = false;
                for(const auto& c : cands) if (std::hypot(c.c.x - pt.x, c.c.y - pt.y) < c.r * 0.8) { is_blade = true; break; }
                if (is_blade) continue;
                
                if (dist_to_last < best_score) { best_score = dist_to_last; r_mark_pos = pt; }
            }
        }
        
        // 【核心修复】中心更新逻辑：R标 > 多目标几何中心 > 冻结
        if (r_mark_pos.x > 0) {
            // 检测到 R 标：平滑更新
            center = center * 0.8 + r_mark_pos * 0.2;
        } else if (cands.size() >= 2) {
            // 多目标：几何中心非常稳定
            Point2d geo(0, 0);
            for (const auto& c : cands) geo += c.c;
            geo /= static_cast<double>(cands.size());
            center = center * 0.8 + geo * 0.2;
        } else {
            // 【关键】单目标且无 R 标：冻结中心！绝不外推，防止飘出画面。
            // center 保持不变
        }
        
        // ---- 3. 多目标跟踪（极坐标预测 + 长寿命 INACTIVE） ----
        std::vector<bool> cand_matched(cands.size(), false);
        
        // 3.1 匹配现有 Track
        for (size_t t = 0; t < tracks.size(); ++t) {
            if (tracks[t].state == TrackState::LOST) continue;
            
            double best_cost = 1e18;
            int best_i = -1;
            
            // 预测下一帧位置（极坐标）
            double pred_theta = tracks[t].theta + tracks[t].last_omega;
            Point2d pred_pos(center.x + tracks[t].orbit_radius * std::cos(pred_theta),
                             center.y + tracks[t].orbit_radius * std::sin(pred_theta));
            
            // 根据状态调整门限
            double pos_gate = tracks[t].is_permanent_inactive ? 250.0 : 150.0;
            
            for (size_t c = 0; c < cands.size(); ++c) {
                if (cand_matched[c]) continue;
                
                double dp = std::hypot(cands[c].c.x - pred_pos.x, cands[c].c.y - pred_pos.y);
                double dr = std::abs(cands[c].r - tracks[t].radius);
                double d_orbit = std::abs(std::hypot(cands[c].c.x - center.x, cands[c].c.y - center.y) - tracks[t].orbit_radius);
                
                double cost = dp * 2.0 + dr * 5.0 + d_orbit * 3.0;
                if (cost < best_cost && dp < pos_gate) { 
                    best_cost = cost;
                    best_i = (int)c;
                }
            }
            
            if (best_i >= 0) {
                cand_matched[best_i] = true;
                tracks[t].center = cands[best_i].c;
                tracks[t].radius = cands[best_i].r;
                tracks[t].type = cands[best_i].type;
                tracks[t].lost_count = 0;
                
                // 更新角速度
                double new_theta = std::atan2(cands[best_i].c.y - center.y, cands[best_i].c.x - center.x);
                tracks[t].last_omega = wrapPi(new_theta - tracks[t].theta);
                tracks[t].theta = new_theta;
                tracks[t].orbit_radius = std::hypot(cands[best_i].c.x - center.x, cands[best_i].c.y - center.y);
                
                tracks[t].state = (cands[best_i].type == 1) ? TrackState::ACTIVE : TrackState::INACTIVE;
                if (tracks[t].state == TrackState::INACTIVE) tracks[t].is_permanent_inactive = true;
            } else {
                tracks[t].lost_count++;
                int tolerance = tracks[t].is_permanent_inactive ? kLostToleranceInactive : kLostToleranceActive;
                if (tracks[t].lost_count > tolerance) tracks[t].state = TrackState::LOST;
            }
        }
        
        // 3.2 初始化新 Track（严格单 ACTIVE 约束）
        bool has_active = false;
        for (const auto& t : tracks) if (t.state == TrackState::ACTIVE) { has_active = true; break; }
        
        for (size_t c = 0; c < cands.size(); ++c) {
            if (cand_matched[c]) continue;
            if (cands[c].type != 1) continue;
            if (has_active) continue;
            
            TrackedObject new_track;
            new_track.id = next_id++;
            new_track.center = cands[c].c;
            new_track.radius = cands[c].r;
            new_track.type = 1;
            new_track.state = TrackState::ACTIVE;
            new_track.theta = std::atan2(cands[c].c.y - center.y, cands[c].c.x - center.x);
            new_track.orbit_radius = std::hypot(cands[c].c.x - center.x, cands[c].c.y - center.y);
            tracks.push_back(new_track);
            cand_matched[c] = true;
            has_active = true;
        }
        
        // 清理超时 Track
        tracks.erase(std::remove_if(tracks.begin(), tracks.end(), 
            [](const TrackedObject& t) { return t.state == TrackState::LOST && t.lost_count > 200; }), tracks.end());
        
        // ---- 4. 可视化 ----
        if (r_mark_pos.x > 0) {
            drawMarker(frame, Point((int)r_mark_pos.x, (int)r_mark_pos.y), Scalar(255, 0, 255), MARKER_CROSS, 12, 2);
            putText(frame, "R", Point((int)r_mark_pos.x + 8, (int)r_mark_pos.y - 8),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);
        }
        
        drawMarker(frame, Point((int)center.x, (int)center.y), Scalar(0, 255, 255), MARKER_CROSS, 10, 1);
        
        for (const auto& t : tracks) {
            if (t.state == TrackState::LOST) continue;
            
            Scalar color = (t.state == TrackState::ACTIVE) ? Scalar(0, 255, 0) : Scalar(255, 255, 0);
            std::string status = (t.state == TrackState::ACTIVE) ? "ACTIVE" : "INACTIVE";
            
            circle(frame, Point((int)t.center.x, (int)t.center.y), (int)t.radius, color, 2);
            circle(frame, Point((int)t.center.x, (int)t.center.y), 3, color, -1);
            
            // 绘制连线（限制在画面内）
            if (center.x >= 0 && center.x < vsize.width && center.y >= 0 && center.y < vsize.height) {
                line(frame, Point((int)center.x, (int)center.y), Point((int)t.center.x, (int)t.center.y), color, 1);
            }
            
            putText(frame, "ID:" + std::to_string(t.id) + " " + status, 
                    Point((int)t.center.x + 8, (int)t.center.y - 8),
                    FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
        
        putText(frame, "frame " + std::to_string(idx) + "/" + std::to_string(n_frames - 1),
                Point(15, 28), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, "center=(" + format("%.1f,%.1f", center.x, center.y) + ")",
                Point(15, 54), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, "Tracks: " + std::to_string(tracks.size()),
                Point(15, 80), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
                
        w_ov.write(frame);
        ++idx;
    }
    
    cap.release(); w_ov.release();
    std::cout << "[OK] " << outdir << "/recognition_overlay.mp4\n";
    return 0;
}