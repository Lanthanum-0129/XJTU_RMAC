// task3_windmill: 真实能量机关视频识别与稳定跟踪（GPU 预处理 + 运动预测优化）
// 目标形态：靶标形（同心圆+十字）→ 单圆形，全程同一身份跟踪至熄灭；
// 中心：R 字母逐帧时序跟踪 + 被选目标历史 Kasa 兜底（无模板匹配）。
#include  <opencv2/opencv.hpp>
#ifdef ENABLE_CUDA_ACCEL
#include  <opencv2/core/cuda.hpp>
#include  <opencv2/cudaarithm.hpp>
#include  <opencv2/cudaimgproc.hpp>
#endif
#include  <Eigen/Dense>
#include  <iostream>
#include  <fstream>
#include  <sstream>
#include  <iomanip>
#include  <vector>
#include  <deque>
#include  <string>
#include  <cmath>
#include  <chrono>
#include  <algorithm>
#include  <filesystem>
namespace fs = std::filesystem;
using namespace cv;

inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// ---------------- 调色板：line=橙/蓝描边, bar=黄/青灯条 ----------------
struct Palette { Scalar line_lo, line_hi, bar_lo, bar_hi; };
static const Palette kPalOrange{ Scalar(5, 120, 80),   Scalar(20, 255, 255),
                                 Scalar(20, 120, 150), Scalar(45, 255, 255) };
static const Palette kPalBlue  { Scalar(100, 120, 80), Scalar(135, 255, 255),
                                 Scalar(80, 120, 150), Scalar(100, 255, 255) };

// ---------------- 判参 ----------------
static const double kMinR = 10.0, kMaxR = 100.0;     // 适当放宽目标圆半径范围
static const double kCoverMin = 0.75;               // 环覆盖率下限
static const int    kInnerReticle = 50;             // 内部像素≥此值判为靶标形
static const int    kRAreaLo = 120, kRAreaHi = 800; // R 字母面积范围
static const double kRStepMax = 150.0;              // R 帧间移动上限
static const double kRInitMax = 400.0;              // R 初始化距画面中心上限
static const double kRCandMax = 350.0;              // R 与最近候选距离上限
static const double kAngGate = 0.5, kRadGate = 50.0, kPosGate = 200.0; // 放宽关联门限
static const int    kLostTolerance = 60;            // 增加丢失容忍度
static const int    kKasaWindow = 150;
static const double kKasaMinArcDeg = 60.0;
static const int    kProbeFrames = 60;
static const int    kConfirmFrames = 3;

struct Candidate {
    Point2d c; double r = 0; int type = 0;  // type: 1=靶标形, 0=单圆形
    double theta = 0;
};

// ---------------- GPU/CPU 预处理器 ----------------
class Preprocessor {
public:
    void init(const std::string & req) {
        bool ok = false;
#ifdef ENABLE_CUDA_ACCEL
        ok = cuda::getCudaEnabledDeviceCount() > 0;
        if (ok) cuda::printShortCudaDeviceInfo(cuda::getDevice());
#endif
        if (req == "gpu" && !ok) std::cerr << "[WARN] GPU unavailable, fallback CPU.\n";
        use_gpu_ = (req == "gpu" || req == "auto") && ok;
        std::cout << "[INFO] backend: " << name() << "\n";
    }
    const char* name() const { return use_gpu_ ? "GPU (OpenCV CUDA)" : "CPU"; }
    void probe(const Mat& bgr, double& warm_px, double& blue_px) {
        warm_px = blue_px = 0;
#ifdef ENABLE_CUDA_ACCEL
        if (use_gpu_) {
            g_bgr_.upload(bgr);
            cuda::cvtColor(g_bgr_, g_hsv_, COLOR_BGR2HSV);
            cuda::inRange(g_hsv_, kPalOrange.line_lo, kPalOrange.line_hi, g_m1_);
            cuda::inRange(g_hsv_, kPalOrange.bar_lo,  kPalOrange.bar_hi,  g_m2_);
            cuda::bitwise_or(g_m1_, g_m2_, g_m1_);
            warm_px = cuda::countNonZero(g_m1_);
            cuda::inRange(g_hsv_, kPalBlue.line_lo, kPalBlue.line_hi, g_m1_);
            cuda::inRange(g_hsv_, kPalBlue.bar_lo,  kPalBlue.bar_hi,  g_m2_);
            cuda::bitwise_or(g_m1_, g_m2_, g_m1_);
            blue_px = cuda::countNonZero(g_m1_);
            return;
        }
#endif
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
#ifdef ENABLE_CUDA_ACCEL
        if (use_gpu_) {
            g_bgr_.upload(bgr);
            cuda::cvtColor(g_bgr_, g_hsv_, COLOR_BGR2HSV);
            cuda::inRange(g_hsv_, p.line_lo, p.line_hi, g_m1_);
            cuda::inRange(g_hsv_, p.bar_lo,  p.bar_hi,  g_m2_);
            g_m1_.download(line_m);
            g_m2_.download(bar_m);
            return;
        }
#endif
        Mat hsv;
        cvtColor(bgr, hsv, COLOR_BGR2HSV);
        inRange(hsv, p.line_lo, p.line_hi, line_m);
        inRange(hsv, p.bar_lo,  p.bar_hi,  bar_m);
    }
private:
    bool use_gpu_ = false;
#ifdef ENABLE_CUDA_ACCEL
    cuda::GpuMat g_bgr_, g_hsv_, g_m1_, g_m2_;
#endif
};

// ---------------- 工具 ----------------
static double ringCoverage(const Mat& m, const Point2d& c, double r) {
    const int N = 48; int hit = 0;
    for (int i = 0; i < N; ++i) {
        double a = 2 * CV_PI * i / N;
        bool ok = false;
        for (int dr = -3; dr <= 3 && !ok; ++dr) {
            double rr = r + dr;
            int x = (int)std::round(c.x + rr * std::cos(a));
            int y = (int)std::round(c.y + rr * std::sin(a));
            if (x < 0 || y < 0 || x >= m.cols || y >= m.rows) continue;
            if (m.at<uchar>(y, x)) ok = true;
        }
        if (ok) ++hit;
    }
    return (double)hit / N;
}

static int innerCount(const Mat& m, const Point2d& c, double r) {
    int R = (int)std::ceil(r);
    Rect roi((int)c.x - R, (int)c.y - R, 2 * R + 1, 2 * R + 1);
    roi &= Rect(0, 0, m.cols, m.rows);
    if (roi.width <= 0 || roi.height <= 0) return 0;
    Mat disk = Mat::zeros(roi.size(), CV_8UC1);
    circle(disk, Point((int)c.x - roi.x, (int)c.y - roi.y), (int)r, Scalar(255), -1);
    Mat inter; bitwise_and(m(roi), disk, inter);
    return countNonZero(inter);
}

static bool kasaCenter(const std::deque<Point2d>& pts, const Point2d& ref, Point2d& out) {
    if ((int)pts.size() < 25) return false;
    std::vector<double> angs;
    for (auto& p : pts) angs.push_back(std::atan2(p.y - ref.y, p.x - ref.x));
    std::sort(angs.begin(), angs.end());
    double maxgap = angs.front() + 2 * CV_PI - angs.back();
    for (size_t i = 1; i < angs.size(); ++i) maxgap = std::max(maxgap, angs[i] - angs[i - 1]);
    if (2 * CV_PI - maxgap < kKasaMinArcDeg * CV_PI / 180.0) return false;
    int n = (int)pts.size();
    Eigen::MatrixXd A(n, 3); Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
        double x = pts[i].x, y = pts[i].y;
        A(i, 0) = x; A(i, 1) = y; A(i, 2) = 1.0;
        b(i) = -(x * x + y * y);
    }
    Eigen::Vector3d s = A.colPivHouseholderQr().solve(b);
    Point2d c(-s(0) / 2, -s(1) / 2);
    if (std::hypot(c.x - ref.x, c.y - ref.y) > 200.0) return false;
    out = c;
    return true;
}

static void drawDashedCircle(Mat& img, Point c, int r, const Scalar& col) {
    for (int a = 0; a < 360; a += 30)
        ellipse(img, c, Size(r, r), 0, a, a + 18, col, 2);
}

// ---------------- 主程序 ----------------
int main(int argc, char** argv) {
    std::string input, outbase = "result/task3_windmill", accel = "gpu";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--input" && i + 1 < argc) input = argv[++i];
        else if (a == "--accel" && i + 1 < argc) accel = argv[++i];
        else if (input.empty()) input = a;
    }
    if (input.empty()) {
        std::cerr << "usage: task3_windmill --input <mp4> [--accel gpu|cpu|auto]\n";
        return 1;
    }
    std::string tag = fs::path(input).stem().string();
    std::string outdir = outbase + "/" + tag;
    fs::create_directories(outdir);
    VideoCapture cap(input);
    if (!cap.isOpened()) { std::cerr << "Cannot open " << input << "\n"; return 1; }
    const double fps = cap.get(CAP_PROP_FPS) > 0 ? cap.get(CAP_PROP_FPS) : 60.0;
    const int n_frames = (int)cap.get(CAP_PROP_FRAME_COUNT);
    const Size vsize((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));
    const Point2d img_center(vsize.width / 2.0, vsize.height / 2.0);
    std::cout << "[INFO] " << tag << ": " << vsize.width << "x" << vsize.height
              << ", fps=" << fps << ", frames=" << n_frames << "\n";

    Preprocessor pre; pre.init(accel);
    VideoWriter w_ov(outdir + "/recognition_overlay.mp4",
                     VideoWriter::fourcc('m', 'p', '4', 'v'), fps, vsize);
    VideoWriter w_bin(outdir + "/binary_process.mp4",
                      VideoWriter::fourcc('m', 'p', '4', 'v'), fps, vsize);
    if (!w_ov.isOpened() || !w_bin.isOpened()) { std::cerr << "writer fail\n"; return 1; }

    Palette pal = kPalOrange; bool pal_fixed = false, pal_orange = true;
    double warm_sum = 0, blue_sum = 0;
    Point2d center = img_center;
    bool center_ready = false;
    Point2d r_last; bool r_have = false;
    int track_id = -1, lost = 0;
    int confirm_cnt = 0; Point2d pend_pos(0, 0);
    bool have_target = false;
    Point2d last_pos(0, 0); double last_theta = 0, last_r = 0; int last_type = 0;
    
    // 新增：运动预测状态变量
    Point2d pred_pos(0, 0);       
    Point2d last_vel(0, 0);       
    bool have_vel = false;

    std::deque<Point2d> locked_hist;
    std::vector<std::string> events;
    int detected_cnt = 0, lost_cnt = 0, max_lost = 0;
    double ms_sum = 0; int ms_n = 0;
    Mat frame, m_line, m_bar, proc;
    int idx = 0;

    while (cap.read(frame)) {
        auto t0 = std::chrono::steady_clock::now();
        if (!pal_fixed) {
            double w, b; pre.probe(frame, w, b);
            warm_sum += w; blue_sum += b;
            if (idx == kProbeFrames - 1) {
                pal_orange = (warm_sum >= blue_sum);
                pal = pal_orange ? kPalOrange : kPalBlue;
                pal_fixed = true;
                std::cout << "[INFO] palette: " << (pal_orange ? "orange/yellow" : "blue")
                          << " (warm=" << warm_sum << ", blue=" << blue_sum << ")\n";
            }
        }
        pre.masks(frame, pal, m_line, m_bar);
        morphologyEx(m_line, proc, MORPH_CLOSE,
                     getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        auto t1 = std::chrono::steady_clock::now();
        ms_sum += std::chrono::duration<double, std::milli>(t1 - t0).count(); ++ms_n;

        // ---- 候选：Hough 圆环 + 环覆盖率 + 内部密度分型（靶标形/单圆形） ----
        std::vector<Candidate> cands;
        {
            std::vector<Vec3f> circles;
            // 降低阈值，允许更多候选，后续再筛选
            HoughCircles(proc, circles, HOUGH_GRADIENT, 1.0, 30.0, 80, 20,
                         (int)kMinR, (int)kMaxR);
            for (auto& v : circles) {
                Point2d c(v[0], v[1]); double r = v[2];
                bool dup = false;
                for (auto& k : cands)
                    if (std::hypot(k.c.x - c.x, k.c.y - c.y) < 30.0) { dup = true; break; }
                if (dup) continue;
                if (ringCoverage(proc, c, r) < kCoverMin) continue;
                int inner = innerCount(proc, c, 0.65 * r);
                double disk_area = CV_PI * 0.65 * r * 0.65 * r;
                if (inner > 0.5 * disk_area) continue;
                Candidate k; k.c = c; k.r = r;
                k.type = (inner >= kInnerReticle) ? 1 : 0;
                k.theta = std::atan2(center.y - c.y, c.x - center.x);
                cands.push_back(k);
                if ((int)cands.size() >= 8) break;
            }
        }

        // 补充检测：当 Hough 候选不足时，使用轮廓最小外接圆补充（应对运动模糊）
        if ((int)cands.size() < 3) {
            std::vector<std::vector<Point>> contours;
            std::vector<Vec4i> hierarchy;
            findContours(proc.clone(), contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (auto& cnt : contours) {
                if (contourArea(cnt) < 100) continue;
                Point2f center_cnt; float radius;
                minEnclosingCircle(cnt, center_cnt, radius);
                if (radius < kMinR || radius > kMaxR) continue;
                if (ringCoverage(proc, center_cnt, radius) < kCoverMin * 0.8) continue; // 模糊时放宽覆盖率
                bool dup = false;
                for (auto& k : cands)
                    if (std::hypot(k.c.x - center_cnt.x, k.c.y - center_cnt.y) < 20.0) { dup = true; break; }
                if (dup) continue;
                Candidate k; k.c = center_cnt; k.r = radius;
                k.type = (innerCount(proc, center_cnt, 0.65 * radius) >= kInnerReticle) ? 1 : 0;
                k.theta = std::atan2(center.y - center_cnt.y, center_cnt.x - center.x);
                cands.push_back(k);
            }
        }
        // ---- R 字母候选（轮毂中心标记） ----
        std::vector<Point2d> r_cands;
        {
            std::vector<std::vector<Point>> cs; std::vector<Vec4i> h;
            findContours(proc.clone(), cs, h, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            for (size_t ci = 0; ci < cs.size(); ++ci) {
                if (h[ci][3] != -1) continue;
                double a = contourArea(cs[ci]);
                if (a < kRAreaLo || a > kRAreaHi) continue;
                double p = arcLength(cs[ci], true);
                if (p <= 0) continue;
                double circ = 4 * CV_PI * a / (p * p);
                if (circ >= 0.6) continue;
                Rect bb = boundingRect(cs[ci]);
                double asp = (bb.height > 0) ? (double)bb.width / bb.height : 0;
                if (asp < 0.6 || asp > 1.6) continue;
                Moments m = moments(cs[ci]);
                r_cands.push_back(Point2d(m.m10 / m.m00, m.m01 / m.m00));
            }
        }

        // ---- 中心：R 时序跟踪，Kasa(被选目标历史) 兜底 ----
        std::string csrc = "prev";
        Point2d r_cur; bool r_ok = false;
        if (r_have) {
            double bd = 1e18;
            for (auto& p : r_cands) {
                double d = std::hypot(p.x - r_last.x, p.y - r_last.y);
                if (d < kRStepMax && d < bd) { bd = d; r_cur = p; r_ok = true; }
            }
        } else if (!cands.empty()) {
            double bd = 1e18;
            for (auto& p : r_cands) {
                double d = std::hypot(p.x - img_center.x, p.y - img_center.y);
                if (d < kRInitMax && d < bd) { bd = d; r_cur = p; r_ok = true; }
            }
        }
        if (r_ok) {
            double dn = 1e18;
            for (auto& k : cands)
                dn = std::min(dn, std::hypot(k.c.x - r_cur.x, k.c.y - r_cur.y));
            if (cands.empty() || dn > kRCandMax) r_ok = false;
        }
        Point2d c_kasa; bool ka_ok = false;
        if (have_target) ka_ok = kasaCenter(locked_hist, center, c_kasa);

        if (r_ok) {
            r_have = true; r_last = r_cur;
            // 优化：限制 R 字母引起的中心跳变
            double r_dist = std::hypot(r_cur.x - center.x, r_cur.y - center.y);
            if (r_dist > 150.0 && center_ready) {
                center = Point2d(0.9 * center.x + 0.1 * r_cur.x, 0.9 * center.y + 0.1 * r_cur.y);
            } else {
                center = center_ready ? Point2d(0.5 * (center.x + r_cur.x), 0.5 * (center.y + r_cur.y)) : r_cur;
            }
            csrc = "R";
        } else if (ka_ok) {
            // 优化：Kasa 兜底时同样限制跳变
            double k_dist = std::hypot(c_kasa.x - center.x, c_kasa.y - center.y);
            if (k_dist < 100.0) {
                center = c_kasa; 
            }
            csrc = "kasa";
        }
        if (!center_ready && r_ok) {
            center_ready = true;
            events.push_back("frame " + std::to_string(idx) + ": center initialized by R-letter");
        }
        // ---- 过滤误检的R字母候选 ----
        // R字母位于机关中心，不应作为外围扇叶目标被跟踪。
        // 剔除与R字母候选位置重合且半径较小的圆，防止R被误认为目标。
        std::vector<Candidate> valid_cands;
        for (const auto& cand : cands) {
            bool is_r_letter = false;
            for (const auto& r_pt : r_cands) {
                // 如果候选圆中心距离R字母中心很近（<20像素），且半径较小（<40像素，R字母通常比扇叶小）
                if (std::hypot(cand.c.x - r_pt.x, cand.c.y - r_pt.y) < 20.0 && cand.r < 40.0) {
                    is_r_letter = true;
                    break;
                }
            }
            if (!is_r_letter) {
                valid_cands.push_back(cand);
            }
        }   
        cands = valid_cands; // 更新候选列表

        // ---- 关联 / 锁定 / 丢失 / 重选 (引入运动预测) ----
        std::string status = "LOST";
        int best_i = -1;

        // 计算预测位置
        if (have_target && have_vel) {
            pred_pos = Point2d(last_pos.x + last_vel.x, last_pos.y + last_vel.y);
        } else {
            pred_pos = last_pos;
        }

        if (have_target) {
            double best_cost = 1e18;
            for (size_t i = 0; i < cands.size(); ++i) {
                double dp = std::hypot(cands[i].c.x - pred_pos.x, cands[i].c.y - pred_pos.y);
                if (dp > kPosGate * 1.5) continue; // 快速移动时适当放宽门限

                double dth = wrapPi(cands[i].theta - last_theta);
                double dr  = cands[i].r - last_r;
                
                // 综合代价：位置权重最高，角度和半径次之
                double cost = dp * 1.0 + std::abs(dth / kAngGate) * 50.0 + std::abs(dr / kRadGate) * 20.0;
                
                if (cost < best_cost) { 
                    best_cost = cost; 
                    best_i = (int)i; 
                }
            }
            
            if (best_i >= 0) {
                if (lost > 0)
                    events.push_back("frame " + std::to_string(idx) + ": recovery of ID=" +
                        std::to_string(track_id) + " after " + std::to_string(lost) + " lost");
                lost = 0; status = "DETECTED";
            } else if (++lost > kLostTolerance && !cands.empty()) {
                // 重构重选逻辑：基于运动趋势与距离评分
                int ri = -1; double best_score = -1e18;
                
                // 计算历史平均角速度
                double avg_omega = 0;
                if (locked_hist.size() >= 5) {
                    Point2d p1 = locked_hist.front();
                    Point2d p2 = locked_hist.back();
                    double th1 = std::atan2(center.y - p1.y, p1.x - center.x);
                    double th2 = std::atan2(center.y - p2.y, p2.x - center.x);
                    avg_omega = wrapPi(th2 - th1) / std::max(1.0, (double)locked_hist.size());
                }

                for (size_t i = 0; i < cands.size(); ++i) {
                    double dp = std::hypot(cands[i].c.x - pred_pos.x, cands[i].c.y - pred_pos.y);
                    
                    double th_cand = std::atan2(center.y - cands[i].c.y, cands[i].c.x - center.x);
                    double omega_cand = wrapPi(th_cand - last_theta) / std::max(1.0, (double)(lost + 1));
                    
                    double dist_score = std::exp(-dp / 100.0); 
                    double omega_diff = std::abs(omega_cand - avg_omega);
                    double omega_score = std::exp(-omega_diff * 5.0);
                    double r_score = (cands[i].r > kMinR && cands[i].r < kMaxR) ? 1.0 : 0.5;
                    
                    double score = dist_score * 0.5 + omega_score * 0.3 + r_score * 0.2;
                    
                    if (score > best_score) {
                        best_score = score;
                        ri = (int)i;
                    }
                }

                if (ri >= 0) {
                    events.push_back("frame " + std::to_string(idx) + ": reselect ID=" +
                        std::to_string(track_id + 1) + " after " + std::to_string(lost) +
                        " lost; basis=motion & proximity");
                    ++track_id; lost = 0; best_i = ri; status = "DETECTED";
                    locked_hist.clear();
                    have_vel = false; // 重选后重置速度
                }
            }
        } else if (!cands.empty()) {
            // 首次锁定逻辑
            int ri = -1; double ra = 0;
            for (size_t i = 0; i < cands.size(); ++i)
                if (cands[i].r > ra) { ra = cands[i].r; ri = (int)i; }
            const Point2d& pc = cands[ri].c;
            if (confirm_cnt > 0 &&
                std::hypot(pc.x - pend_pos.x, pc.y - pend_pos.y) < 40.0)
                ++confirm_cnt;
            else { confirm_cnt = 1; pend_pos = pc; }
            if (confirm_cnt >= kConfirmFrames) {
                track_id = 0; best_i = ri; lost = 0;
                status = "DETECTED"; have_target = true;
                events.push_back("frame " + std::to_string(idx) +
                    ": first lock ID=0 (type=" +
                    std::to_string(cands[ri].type) + "), confirmed over " +
                    std::to_string(kConfirmFrames) + " frames");
            }
        } else {
            confirm_cnt = 0;
        }

        if (best_i >= 0) {
            const Candidate& k = cands[best_i];
            
            // 更新速度
            if (have_target && lost == 0) {
                last_vel = Point2d(k.c.x - last_pos.x, k.c.y - last_pos.y);
                have_vel = true;
            }
            
            last_pos = k.c; last_theta = k.theta; last_r = k.r; last_type = k.type;
            locked_hist.push_back(k.c);
            if ((int)locked_hist.size() > kKasaWindow) locked_hist.pop_front();
            ++detected_cnt;
        } else {
            ++lost_cnt; max_lost = std::max(max_lost, lost);
        }

        // 1. 先绘制未选中的候选
        for (size_t i = 0; i < cands.size(); ++i) {
            if ((int)i == best_i) continue;
            circle(frame, Point((int)cands[i].c.x, (int)cands[i].c.y),
                (int)cands[i].r, Scalar(0, 255, 255), 1);
            putText(frame, cands[i].type ? "cand-ret" : "cand",
                    Point((int)cands[i].c.x + 8, (int)cands[i].c.y - 8),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
        }

        // 2. 绘制被选中的目标（绿色圆圈）
        if (best_i >= 0) {
            const Candidate& k = cands[best_i];
            circle(frame, Point((int)k.c.x, (int)k.c.y), (int)k.r, Scalar(0, 255, 0), 2);
            circle(frame, Point((int)k.c.x, (int)k.c.y), 4, Scalar(0, 255, 255), -1);
            if (center_ready)
                line(frame, Point((int)center.x, (int)center.y),
                    Point((int)k.c.x, (int)k.c.y), Scalar(0, 255, 255), 2);
            putText(frame, "ID=" + std::to_string(track_id),
                    Point((int)k.c.x + 10, (int)k.c.y + 20),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        } else if (have_target) {
            drawDashedCircle(frame, Point((int)last_pos.x, (int)last_pos.y),
                            (int)last_r, Scalar(0, 0, 255));
            putText(frame, "LOST", Point((int)last_pos.x + 10, (int)last_pos.y),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }

        // 3. 最后绘制中心R标记（确保不被遮挡）
        if (center_ready && r_have) {
            drawMarker(frame, Point((int)center.x, (int)center.y),
                    Scalar(255, 0, 255), MARKER_CROSS, 20, 2);
            putText(frame, "R", Point((int)center.x + 14, (int)center.y - 12),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2);
        }
        for (size_t i = 0; i < cands.size(); ++i) {
            if ((int)i == best_i) continue;
            circle(frame, Point((int)cands[i].c.x, (int)cands[i].c.y),
                   (int)cands[i].r, Scalar(0, 255, 255), 1);
            putText(frame, cands[i].type ? "cand-ret" : "cand",
                    Point((int)cands[i].c.x + 8, (int)cands[i].c.y - 8),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
        }
        if (best_i >= 0) {
            const Candidate& k = cands[best_i];
            circle(frame, Point((int)k.c.x, (int)k.c.y), (int)k.r, Scalar(0, 255, 0), 2);
            circle(frame, Point((int)k.c.x, (int)k.c.y), 4, Scalar(0, 255, 255), -1);
            if (center_ready)
                line(frame, Point((int)center.x, (int)center.y),
                     Point((int)k.c.x, (int)k.c.y), Scalar(0, 255, 255), 2);
            putText(frame, "ID=" + std::to_string(track_id),
                    Point((int)k.c.x + 10, (int)k.c.y + 20),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        } else if (have_target) {
            drawDashedCircle(frame, Point((int)last_pos.x, (int)last_pos.y),
                             (int)last_r, Scalar(0, 0, 255));
            putText(frame, "LOST", Point((int)last_pos.x + 10, (int)last_pos.y),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }
        putText(frame, "frame " + std::to_string(idx) + "/" + std::to_string(n_frames - 1),
                Point(15, 28), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, "center=(" + format("%.1f,%.1f", center.x, center.y) + ") src=" + csrc,
                Point(15, 54), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, have_target ? ("ID=" + std::to_string(track_id) + " status=" + status +
                                      " lost=" + std::to_string(lost))
                                   : "status=SEARCHING",
                Point(15, 80), FONT_HERSHEY_SIMPLEX, 0.65,
                status == "DETECTED" ? Scalar(0, 255, 0) : Scalar(0, 0, 255), 2);
        if (have_target)
            putText(frame, format("theta=%.3f rad r=%.1f px type=%s cands=%d",
                                  last_theta, last_r, last_type ? "reticle" : "circle",
                                  (int)cands.size()),
                    Point(15, 106), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        w_ov.write(frame);
        Mat uni; bitwise_or(proc, m_bar, uni);
        Mat bgr_bin; cvtColor(uni, bgr_bin, COLOR_GRAY2BGR);
        w_bin.write(bgr_bin);
        ++idx;
    }
    cap.release(); w_ov.release(); w_bin.release();
    const double avg_ms = ms_n ? ms_sum / ms_n : 0;
    std::cout << "[INFO] frames=" << idx << ", detected=" << detected_cnt
              << ", lost=" << lost_cnt << ", max_lost=" << max_lost
              << ", avg preprocess=" << avg_ms << " ms (" << pre.name() << ")\n";
    std::cout << "[OK] " << outdir << "/recognition_overlay.mp4\n";
    std::cout << "[OK] " << outdir << "/binary_process.mp4\n";

    // ---- 单视频小结 ----
    {
        std::ofstream md(outdir + "/summary.md");
        md << "# " << tag << " 跟踪小结\n\n"
           << "- 视频参数：" << vsize.width << "x" << vsize.height << "，" << fps
           << " FPS，读入 " << idx << " 帧\n"
           << "- 预处理后端：" << pre.name() << "（--accel " << accel
           << "），平均预处理耗时 " << avg_ms << " ms/帧\n"
           << "- 调色板：" << (pal_orange ? "orange/yellow" : "blue") << "\n"
           << "- 检出帧 " << detected_cnt << "，丢失帧 " << lost_cnt
           << "，最长连续丢失 " << max_lost << " 帧\n"
           << "- 目标形态：靶标形（同心圆+十字，内部密度≥50）与单圆形均为有效目标，"
              "同一身份从亮起跟踪至熄灭；候选由 Hough 圆环 + 环覆盖率≥0.75 验证，"
              "横幅、科技核心方形、实心发光块被覆盖率/密度判据拒绝；运动模糊时启用轮廓外接圆补充\n"
           << "- 中心：R 字母逐帧时序跟踪（增加跳变抑制），"
              "R 缺失时被选目标自身历史 Kasa 圆拟合兜底；角度相对当帧中心计算\n"
           << "- 锁定：引入匀速运动预测与综合代价关联（位置/角度/半径）；首锁连续 "
           << kConfirmFrames << " 帧确认；丢失容忍 " << kLostTolerance
           << " 帧后按运动趋势与距离评分重选并递增 ID\n"
           << "- 事件表：\n\n| 帧 | 事件 |\n|---|---|\n";
        for (auto& e : events) {
            size_t sp = e.find(':');
            md << "| " << e.substr(0, sp) << " | " << e.substr(sp + 2) << " |\n";
        }
        md << "\n- 视频：recognition_overlay.mp4、binary_process.mp4（本目录）\n";
    }

    // ---- 合并总说明 ----
    {
        std::ifstream f3(outbase + "/task_3/summary.md"), f4(outbase + "/task_4/summary.md");
        std::stringstream s3, s4;
        if (f3.is_open()) s3 << f3.rdbuf();
        if (f4.is_open()) s4 << f4.rdbuf();
        std::ofstream md(outbase + "/../task3_tracking_result.md");
        md << "# 任务 3 跟踪结果说明\n\n## 1. 场景对应\n\n"
           << "- task_3.mp4：小能量机关（同时最多亮起 1 个目标）\n"
           << "- task_4.mp4：大能量机关（同时最多亮起 2 个目标）\n\n"
           << "## 2. 检测方法\n\n"
           << "- GPU 逐帧 HSV 双掩膜（描边+灯条），前 60 帧自动选择橙黄/蓝调色板；\n"
           << "- 目标=扇叶圆环：Hough 圆检测 + 环覆盖率≥0.75 验证；内部像素密度区分"
              "靶标形（亮起初期）与单圆形（成熟期），二者为同一目标的生命周期形态，全程同一身份跟踪至熄灭；\n"
           << "- 误识别防护：横幅字母与科技核心方形环覆盖率不足、实心发光块内部密度超限，均被拒绝；\n"
           << "- 补充检测：当 Hough 召回率因运动模糊下降时，自动启用轮廓最小外接圆补充；\n"
           << "- 中心：R 字母逐帧时序跟踪（增加跳变抑制），"
              "R 缺失时被选目标自身轨道 Kasa 圆拟合兜底；角度相对当帧中心计算。\n\n"
           << "## 3. 锁定与重选规则\n\n"
           << "- 首锁选最大半径候选并连续 3 帧确认，ID=0；引入匀速运动预测，使用综合代价（位置/角度/半径）进行关联；\n"
           << "- 被选目标持续可见时保持身份（含靶标→单圆形态转换），其余目标仅标 cand；\n"
           << "- 丢失容忍 " << kLostTolerance << " 帧：容忍内恢复保持原 ID；超限后废除最大半径法，"
              "改用基于运动趋势（角速度）与距离预测位置的综合评分重选并递增 ID；\n"
           << "- 状态显示 DETECTED / LOST(lost=k) / SEARCHING；丢失帧保留并以红色虚线圆标记上次位置。\n\n"
           << "## 4. 已知失败情况\n\n"
           << "- 极端运动模糊使圆环覆盖率短暂下降时漏检（由丢失容忍和轮廓补充吸收）；\n"
           << "- 两目标重叠时 Hough 可能只输出一个圆，恢复后按门限重新关联；\n"
           << "- R 字母被遮挡的连续帧内中心由 Kasa/上一帧维持，src 字段可查。\n\n"
           << "## 5. 分视频结果\n\n"
           << (s3.str().empty() ? std::string("_task_3 小结待生成_\n") : s3.str()) << "\n"
           << (s4.str().empty() ? std::string("_task_4 小结待生成_\n") : s4.str()) << "\n"
           << "## 6. 视频链接\n\n"
           << "- result/task3_windmill/task_3/recognition_overlay.mp4\n"
           << "- result/task3_windmill/task_4/recognition_overlay.mp4\n"
           << "- 二值化过程：同目录 binary_process.mp4\n";
    }
    std::cout << "[DONE] Task 3 finished.\n";
    return 0;
}