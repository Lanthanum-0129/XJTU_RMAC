// task2_fit: 合成旋转视频参数拟合（CPU/GPU 双预处理方案）
#include <opencv2/opencv.hpp>

#ifdef ENABLE_CUDA_ACCEL
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;
using namespace cv;

// ---------------------------------------------------------------
// 常量与工具
// ---------------------------------------------------------------
static const Scalar kCyanLo(80, 100, 100);    // 青色 HSV 下界
static const Scalar kCyanHi(100, 255, 255);   // 青色 HSV 上界
static const Scalar kWhiteLo(0, 0, 200);      // 白色 HSV 下界
static const Scalar kWhiteHi(179, 50, 255);   // 白色 HSV 上界
static const Point2d kKnownCenter(480.0, 360.0);

inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct FrameInfo {
    int    idx = -1;
    bool   tracked = false;
    double x = 0, y = 0;
    double theta_wrapped = 0, theta_unwrapped = 0;
};

// ---------------------------------------------------------------
// CPU/GPU 双方案预处理器
// ---------------------------------------------------------------
class Preprocessor {
public:
    void init(const std::string& request) {
        bool cuda_ok = false;
#ifdef ENABLE_CUDA_ACCEL
        cuda_ok = cuda::getCudaEnabledDeviceCount() > 0;
        if (cuda_ok) cuda::printShortCudaDeviceInfo(cuda::getDevice());
#endif
        if (request == "gpu") {
            if (cuda_ok) { use_gpu_ = true; }
            else { std::cerr << "[WARN] GPU requested but unavailable, fallback to CPU.\n"; }
        } else if (request == "auto") {
            use_gpu_ = cuda_ok;
        } else {
            use_gpu_ = false;
        }
        std::cout << "[INFO] Preprocess backend: " << name() << "\n";
    }

    const char* name() const { return use_gpu_ ? "GPU (OpenCV CUDA)" : "CPU"; }

    void masks(const Mat& bgr, Mat& cyan, Mat& white) const {
#ifdef ENABLE_CUDA_ACCEL
        if (use_gpu_) {
            g_bgr_.upload(bgr);
            cuda::cvtColor(g_bgr_, g_hsv_, COLOR_BGR2HSV);
            cuda::inRange(g_hsv_, kCyanLo, kCyanHi, g_c_);
            cuda::inRange(g_hsv_, kWhiteLo, kWhiteHi, g_w_);
            g_c_.download(cyan);
            g_w_.download(white);
            return;
        }
#endif
        Mat hsv;
        cvtColor(bgr, hsv, COLOR_BGR2HSV);
        inRange(hsv, kCyanLo, kCyanHi, cyan);
        inRange(hsv, kWhiteLo, kWhiteHi, white);
    }

private:
    bool use_gpu_ = false;
#ifdef ENABLE_CUDA_ACCEL
    mutable cuda::GpuMat g_bgr_, g_hsv_, g_c_, g_w_;
#endif
};

// ---------------------------------------------------------------
// 连通域工具
// ---------------------------------------------------------------
static bool largestBlob(const Mat& mask, std::vector<Point>& cont,
                        Point2d& center, double& area) {
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hier;
    findContours(mask.clone(), contours, hier, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    size_t best = 0; double best_a = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double a = contourArea(contours[i]);
        if (a > best_a) { best_a = a; best = i; }
    }
    if (best_a < 20.0) return false;
    Moments m = moments(contours[best]);
    if (m.m00 <= 0) return false;
    center = Point2d(m.m10 / m.m00, m.m01 / m.m00);
    cont = contours[best];
    area = best_a;
    return true;
}

// ---------------------------------------------------------------
// Ceres 残差：角度模式，参数 p = [theta0, b_extra, A, Omega, phi]
// b = b_extra + A  =>  自动满足 b > A
// ---------------------------------------------------------------
struct AngleResidual {
    AngleResidual(double t, double obs) : t_(t), obs_(obs) {}
    template <typename T>
    bool operator()(const T* const p, T* residual) const {
        using std::cos;
        const T b = p[1] + p[2];
        const T theta = p[0] + b * T(t_)
                      - (p[2] / p[3]) * (cos(p[3] * T(t_) + p[4]) - cos(p[4]));
        residual[0] = T(obs_) - theta;
        return true;
    }
    double t_, obs_;
};

// Ceres 残差：角速度模式，参数 p = [b_extra, A, Omega, phi]
struct OmegaResidual {
    OmegaResidual(double t, double obs) : t_(t), obs_(obs) {}
    template <typename T>
    bool operator()(const T* const p, T* residual) const {
        using std::sin;
        const T b = p[0] + p[1];
        residual[0] = T(obs_) - (b + p[1] * sin(p[2] * T(t_) + p[3]));
        return true;
    }
    double t_, obs_;
};

// ---------------------------------------------------------------
// 简易绘图工具
// ---------------------------------------------------------------
struct PlotSeries {
    std::string name;
    Scalar color;
    std::vector<Point2d> pts;
    int style;  // 0 = 散点, 1 = 折线
};

static void savePlot(const std::string& path, const std::string& title,
                     const std::string& xlab, const std::string& ylab,
                     const std::vector<PlotSeries>& series, bool zero_line) {
    const int W = 1100, H = 650, L = 95, R = 260, T = 60, B = 70;
    Mat canvas(H, W, CV_8UC3, Scalar(255, 255, 255));

    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (const auto& s : series) for (const auto& p : s.pts) {
        xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    }
    if (xmin >= xmax) { xmin -= 1; xmax += 1; }
    if (ymin >= ymax) { ymin -= 1; ymax += 1; }
    double px = 0.03 * (xmax - xmin), py = 0.08 * (ymax - ymin);
    xmin -= px; xmax += px; ymin -= py; ymax += py;

    auto MX = [&](double x) { return L + (x - xmin) / (xmax - xmin) * (W - L - R); };
    auto MY = [&](double y) { return H - B - (y - ymin) / (ymax - ymin) * (H - T - B); };

    line(canvas, Point(L, T), Point(L, H - B), Scalar(0, 0, 0), 2);
    line(canvas, Point(L, H - B), Point(W - R, H - B), Scalar(0, 0, 0), 2);
    for (int k = 0; k <= 4; ++k) {
        double vx = xmin + (xmax - xmin) * k / 4.0;
        double vy = ymin + (ymax - ymin) * k / 4.0;
        int ux = (int)MX(vx), uy = (int)MY(vy);
        line(canvas, Point(ux, H - B), Point(ux, H - B + 6), Scalar(0, 0, 0), 1);
        line(canvas, Point(L - 6, uy), Point(L, uy), Scalar(0, 0, 0), 1);
        putText(canvas, format("%.2f", vx), Point(ux - 20, H - B + 24),
                FONT_HERSHEY_SIMPLEX, 0.45, Scalar(0, 0, 0), 1);
        putText(canvas, format("%.2f", vy), Point(L - 60, uy + 5),
                FONT_HERSHEY_SIMPLEX, 0.45, Scalar(0, 0, 0), 1);
    }
    if (zero_line && ymin < 0 && ymax > 0)
        line(canvas, Point(L, (int)MY(0)), Point(W - R, (int)MY(0)), Scalar(160, 160, 160), 1);

    putText(canvas, title, Point(L, 32), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
    putText(canvas, xlab, Point(W - R - 120, H - 18), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 0, 0), 1);
    putText(canvas, ylab, Point(12, T - 12), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 0, 0), 1);

    for (const auto& s : series) {
        if (s.style == 0) {
            for (const auto& p : s.pts)
                circle(canvas, Point((int)MX(p.x), (int)MY(p.y)), 2, s.color, -1);
        } else {
            for (size_t i = 1; i < s.pts.size(); ++i)
                line(canvas, Point((int)MX(s.pts[i - 1].x), (int)MY(s.pts[i - 1].y)),
                     Point((int)MX(s.pts[i].x), (int)MY(s.pts[i].y)), s.color, 2);
        }
    }
    int ly = T + 10;
    for (const auto& s : series) {
        line(canvas, Point(W - R + 20, ly), Point(W - R + 55, ly), s.color, 3);
        putText(canvas, s.name, Point(W - R + 62, ly + 5),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        ly += 28;
    }
    imwrite(path, canvas);
    std::cout << "[OK]   " << path << "\n";
}

// ---------------------------------------------------------------
// 主程序
// ---------------------------------------------------------------
int main(int argc, char** argv) {
    std::string input = "resources/task_2.mp4";
    std::string outdir = "result/task2_fit";
    std::string accel = "auto";     // cpu | gpu | auto
    std::string fitmode = "angle";  // angle | omega

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--accel" && i + 1 < argc) accel = argv[++i];
        else if (a == "--fit" && i + 1 < argc) fitmode = argv[++i];
        else if (a == "--input" && i + 1 < argc) input = argv[++i];
        else if (a == "--outdir" && i + 1 < argc) outdir = argv[++i];
    }
    fs::create_directories(outdir);

    // ---------- 打开视频 ----------
    VideoCapture cap(input);
    if (!cap.isOpened()) { std::cerr << "Cannot open video: " << input << "\n"; return 1; }
    const double fps = cap.get(CAP_PROP_FPS) > 0 ? cap.get(CAP_PROP_FPS) : 60.0;
    const int n_frames = (int)cap.get(CAP_PROP_FRAME_COUNT);
    const Size vsize((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));
    std::cout << "[INFO] video " << vsize.width << "x" << vsize.height
              << ", fps=" << fps << ", frames=" << n_frames << "\n";

    Preprocessor pre;
    pre.init(accel);

    // ---------- 第一遍扫描：检测与角度展开 ----------
    Point2d center = kKnownCenter;
    bool center_detected = false;
    std::vector<FrameInfo> frames;
    Mat frame, cyan, white;
    int i = 0;
    double prev_wrapped = 0, prev_unwrapped = 0;

    while (cap.read(frame)) {
        FrameInfo fi; fi.idx = i;
        pre.masks(frame, cyan, white);

        if (i == 0) {  // 仅首帧检测白色旋转中心并与已知值校验
            std::vector<Point> wc; Point2d wcen; double wa;
            if (largestBlob(white, wc, wcen, wa)) {
                center_detected = true; center = wcen;
                std::cout << "[INFO] detected center=(" << center.x << ", " << center.y
                          << "), known=(" << kKnownCenter.x << ", " << kKnownCenter.y << ")\n";
            } else {
                std::cout << "[WARN] white center not found, use known (480,360).\n";
            }
        }

        std::vector<Point> cc; Point2d tcen; double ca;
        if (largestBlob(cyan, cc, tcen, ca)) {
            fi.tracked = true;
            fi.x = tcen.x; fi.y = tcen.y;
            fi.theta_wrapped = std::atan2(center.y - tcen.y, tcen.x - center.x);
            if (frames.empty() || !frames.back().tracked) {
                fi.theta_unwrapped = fi.theta_wrapped;
            } else {
                double d = wrapPi(fi.theta_wrapped - prev_wrapped);
                fi.theta_unwrapped = prev_unwrapped + d;
            }
            prev_wrapped = fi.theta_wrapped;
            prev_unwrapped = fi.theta_unwrapped;
        }
        frames.push_back(fi);
        ++i;
    }
    cap.release();
    const int n_read = i;

    std::vector<double> vt, vth; std::vector<int> vidx;
    for (const auto& f : frames) if (f.tracked) {
        vt.push_back(f.idx / fps);
        vth.push_back(f.theta_unwrapped);
        vidx.push_back(f.idx);
    }
    if (vt.size() < 50) { std::cerr << "Too few tracked frames.\n"; return 1; }
    std::cout << "[INFO] tracked " << vt.size() << "/" << n_read
              << " frames, range [" << vidx.front() << ", " << vidx.back() << "]\n";

    // ---------- 角速度观测（有限差分，用于初始化与角速度模式） ----------
    std::vector<double> wt, ww;
    for (size_t k = 1; k < vidx.size(); ++k) {
        if (vidx[k] == vidx[k - 1] + 1) {
            wt.push_back(0.5 * (vt[k] + vt[k - 1]));
            ww.push_back((vth[k] - vth[k - 1]) * fps);
        }
    }

    // ---------- 初始化：Ω 网格 + Eigen 线性最小二乘 ----------
    double b0 = 0, A0 = 0, Om0 = 1, ph0 = 0, best_sse = 1e18;
    const int N = (int)wt.size();
    for (double Om = 0.05; Om <= 8.0; Om += 0.01) {
        Eigen::MatrixXd M(N, 3); Eigen::VectorXd y(N);
        for (int r = 0; r < N; ++r) {
            M(r, 0) = 1.0;
            M(r, 1) = std::sin(Om * wt[r]);
            M(r, 2) = std::cos(Om * wt[r]);
            y(r) = ww[r];
        }
        Eigen::Vector3d x = M.colPivHouseholderQr().solve(y);
        double sse = (M * x - y).squaredNorm();
        if (sse < best_sse) {
            best_sse = sse;
            b0 = x(0);
            A0 = std::hypot(x(1), x(2));
            Om0 = Om;
            ph0 = std::atan2(x(2), x(1));
        }
    }
    std::cout << "[INFO] init: b=" << b0 << ", A=" << A0
              << ", Omega=" << Om0 << ", phi=" << ph0 << "\n";

    // ---------- Ceres 优化 ----------
    double p[5];
    ceres::Problem problem;
    ceres::Solver::Summary summary;

    if (fitmode == "angle") {
        p[0] = vth.front();
        p[1] = std::max(b0 - A0, 0.05);   // b_extra
        p[2] = std::max(A0, 0.05);        // A
        p[3] = Om0;                       // Omega
        p[4] = ph0;                       // phi
        for (size_t k = 0; k < vt.size(); ++k)
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<AngleResidual, 1, 5>(
                    new AngleResidual(vt[k], vth[k])), nullptr, p);
        problem.SetParameterLowerBound(p, 1, 1e-3);
        problem.SetParameterLowerBound(p, 2, 1e-3);
        problem.SetParameterLowerBound(p, 3, 1e-2);
        problem.SetParameterUpperBound(p, 3, 50.0);
    } else {
        p[0] = std::max(b0 - A0, 0.05);
        p[1] = std::max(A0, 0.05);
        p[2] = Om0;
        p[3] = ph0;
        for (size_t k = 0; k < wt.size(); ++k)
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<OmegaResidual, 1, 4>(
                    new OmegaResidual(wt[k], ww[k])), nullptr, p);
        problem.SetParameterLowerBound(p, 0, 1e-3);
        problem.SetParameterLowerBound(p, 1, 1e-3);
        problem.SetParameterLowerBound(p, 2, 1e-2);
        problem.SetParameterUpperBound(p, 2, 50.0);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 200;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    // ---------- 导出参数 ----------
    double theta0, b_val, A_val, Om_val, phi_val;
    if (fitmode == "angle") {
        theta0 = p[0]; A_val = p[2]; Om_val = p[3]; phi_val = p[4];
        b_val = p[1] + p[2];
    } else {
        theta0 = vth.front(); A_val = p[1]; Om_val = p[2]; phi_val = p[3];
        b_val = p[0] + p[1];
    }
    const double phi_wrapped = wrapPi(phi_val);
    const double T_period = 2.0 * M_PI / Om_val;

    auto omegaModel = [&](double t) { return b_val + A_val * std::sin(Om_val * t + phi_val); };
    auto thetaModel = [&](double t) {
        return theta0 + b_val * t
             - (A_val / Om_val) * (std::cos(Om_val * t + phi_val) - std::cos(phi_val));
    };

    // ---------- 误差指标 ----------
    double sse_th = 0; for (size_t k = 0; k < vt.size(); ++k) {
        double r = vth[k] - thetaModel(vt[k]); sse_th += r * r; }
    const double rmse_angle = std::sqrt(sse_th / vt.size());

    double sse_w = 0; for (size_t k = 0; k < wt.size(); ++k) {
        double r = ww[k] - omegaModel(wt[k]); sse_w += r * r; }
    const double rmse_omega = std::sqrt(sse_w / wt.size());

    std::cout << "[INFO] A=" << A_val << " rad/s, b=" << b_val << " rad/s, Omega=" << Om_val
              << " rad/s, phi=" << phi_wrapped << " rad";
    if (fitmode == "angle") std::cout << ", theta0=" << theta0 << " rad";
    std::cout << "\n[INFO] RMSE angle=" << rmse_angle << " rad, RMSE omega=" << rmse_omega
              << " rad/s\n";

    // ---------- 图 1：拟合对比 ----------
    {
        std::vector<PlotSeries> ss;
        if (fitmode == "angle") {
            PlotSeries obs{"obs theta", Scalar(255, 0, 0), {}, 0};
            for (size_t k = 0; k < vt.size(); ++k) obs.pts.push_back({vt[k], vth[k]});
            PlotSeries fit{"fit theta", Scalar(0, 0, 255), {}, 1};
            for (int s = 0; s <= 600; ++s) {
                double t = vt.front() + (vt.back() - vt.front()) * s / 600.0;
                fit.pts.push_back({t, thetaModel(t)});
            }
            ss = {obs, fit};
            savePlot(outdir + "/fit_comparison.png",
                     "Unwrapped angle: observation vs fit", "t (s)", "theta (rad)", ss, false);
        } else {
            PlotSeries obs{"obs omega", Scalar(255, 0, 0), {}, 0};
            for (size_t k = 0; k < wt.size(); ++k) obs.pts.push_back({wt[k], ww[k]});
            PlotSeries fit{"fit omega", Scalar(0, 0, 255), {}, 1};
            for (int s = 0; s <= 600; ++s) {
                double t = vt.front() + (vt.back() - vt.front()) * s / 600.0;
                fit.pts.push_back({t, omegaModel(t)});
            }
            ss = {obs, fit};
            savePlot(outdir + "/fit_comparison.png",
                     "Angular velocity: observation vs fit", "t (s)", "omega (rad/s)", ss, false);
        }
    }

    // ---------- 图 2：角速度曲线 ----------
    {
        PlotSeries obs{"fd omega", Scalar(255, 0, 0), {}, 0};
        for (size_t k = 0; k < wt.size(); ++k) obs.pts.push_back({wt[k], ww[k]});
        PlotSeries fit{"model omega", Scalar(0, 0, 255), {}, 1};
        for (int s = 0; s <= 600; ++s) {
            double t = vt.front() + (vt.back() - vt.front()) * s / 600.0;
            fit.pts.push_back({t, omegaModel(t)});
        }
        savePlot(outdir + "/angular_velocity.png",
                 "Estimated angular velocity", "t (s)", "omega (rad/s)", {obs, fit}, true);
    }

    // ---------- 图 3：残差 ----------
    {
        PlotSeries res{"residual", Scalar(0, 140, 0), {}, 1};
        if (fitmode == "angle")
            for (size_t k = 0; k < vt.size(); ++k)
                res.pts.push_back({vt[k], vth[k] - thetaModel(vt[k])});
        else
            for (size_t k = 0; k < wt.size(); ++k)
                res.pts.push_back({wt[k], ww[k] - omegaModel(wt[k])});
        savePlot(outdir + "/residuals.png",
                 std::string("Residuals (") + fitmode + " fit)", "t (s)",
                 fitmode == "angle" ? "res (rad)" : "res (rad/s)", {res}, true);
    }

    // ---------- 第二遍扫描：叠加标注视频 ----------
    VideoCapture cap2(input);
    VideoWriter writer(outdir + "/tracking_overlay.mp4",
                       VideoWriter::fourcc('m', 'p', '4', 'v'), fps, vsize);
    if (!writer.isOpened()) { std::cerr << "Cannot open writer.\n"; return 1; }

    int k = 0;
    while (cap2.read(frame)) {
        const FrameInfo& fi = frames[k];
        pre.masks(frame, cyan, white);
        std::vector<Point> cc; Point2d tcen; double ca;
        bool ok = largestBlob(cyan, cc, tcen, ca);

        // 旋转中心标记
        drawMarker(frame, Point((int)center.x, (int)center.y),
                   Scalar(255, 0, 255), MARKER_CROSS, 18, 2);
        putText(frame, "R", Point((int)center.x + 12, (int)center.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);

        if (ok) {
            drawContours(frame, cc, -1, Scalar(0, 255, 0), 2);
            circle(frame, Point((int)tcen.x, (int)tcen.y), 4, Scalar(0, 255, 255), -1);
            line(frame, Point((int)center.x, (int)center.y),
                 Point((int)tcen.x, (int)tcen.y), Scalar(0, 255, 255), 2);
        }
        const double t = k / fps;
        putText(frame, "frame " + std::to_string(k) + "/" + std::to_string(n_read - 1),
                Point(15, 28), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, format("t=%.3fs  theta=%.3f rad", t, fi.theta_unwrapped),
                Point(15, 54), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, format("omega_model=%.3f rad/s", omegaModel(t)),
                Point(15, 80), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(frame, ok ? "ID=0  status=TRACKED" : "ID=0  status=LOST",
                Point(15, 106), FONT_HERSHEY_SIMPLEX, 0.65,
                ok ? Scalar(0, 255, 0) : Scalar(0, 0, 255), 2);
        writer.write(frame);
        ++k;
    }
    cap2.release(); writer.release();
    std::cout << "[OK]   " << outdir << "/tracking_overlay.mp4 (" << k << " frames)\n";

    // ---------- 生成 result/task2_fit_result.md ----------
    {
        std::ofstream md(outdir + "/../task2_fit_result.md");
        md << std::setprecision(10);
        md << "# 任务 2 拟合结果说明\n\n";
        md << "## 1. 视频与检测\n\n";
        md << "- 输入：" << input << "\n";
        md << "- 分辨率：" << vsize.width << "x" << vsize.height
           << "，帧率：" << fps << " FPS，读入帧数：" << n_read << "\n";
        md << "- 旋转中心：已知 (480, 360)；白色点检测值 ("
           << center.x << ", " << center.y << ")，"
           << (center_detected ? "已用于角度计算" : "检测失败，使用已知值") << "\n";
        md << "- 青色 HSV 阈值：H[80,100] S[100,255] V[100,255]\n";
        md << "- 预处理后端：" << pre.name()
           << "（运行参数 --accel " << accel << "）\n";
        md << "- 有效样本数：" << vt.size() << "，帧范围 ["
           << vidx.front() << ", " << vidx.back() << "]\n\n";
        md << "## 2. 模型与方法\n\n";
        md << "- 模型：omega(t) = b + A sin(Omega t + phi)，时间单位 s，角度单位 rad\n";
        md << "- 拟合方式：" << (fitmode == "angle"
               ? "角度拟合（theta(t) 为 omega 的积分，附加估计 theta0）"
               : "角速度拟合（有限差分观测，不估计 theta0）") << "\n";
        md << "- 求解器：Ceres AutoDiff + DENSE_QR；终止状态："
           << ceres::TerminationTypeToString(summary.termination_type)
           << "，解可用：" << (summary.IsSolutionUsable() ? "是" : "否") << "\n";
        md << "- 初始代价 " << summary.initial_cost << " → 最终代价 "
           << summary.final_cost << "\n";
        md << "- 初值获取：Omega 网格扫描（0.05–8.0，步长 0.01），每个 Omega 下以 "
              "Eigen colPivHouseholderQr 解线性最小二乘，取 SSE 最小者\n";
        md << "- 约束：A>0、Omega>0 由参数下界保证；b>A 由重参数化 b = b_extra + A"
              "（b_extra>=1e-3）保证\n\n";
        md << "## 3. 估计参数\n\n";
        md << "| 参数 | 值 | 单位 |\n|---|---|---|\n";
        md << "| A | " << A_val << " | rad/s |\n";
        md << "| b | " << b_val << " | rad/s |\n";
        md << "| Omega | " << Om_val << " | rad/s |\n";
        md << "| phi（统一到 [-pi, pi)） | " << phi_wrapped << " | rad |\n";
        if (fitmode == "angle")
            md << "| theta0 | " << theta0 << " | rad |\n";
        md << "| 速度变化周期 T=2pi/Omega | " << T_period << " | s |\n\n";
        md << "## 4. 误差指标\n\n";
        md << "- 主指标（" << (fitmode == "angle" ? "角度" : "角速度") << "）RMSE = "
           << (fitmode == "angle" ? rmse_angle : rmse_omega)
           << (fitmode == "angle" ? " rad" : " rad/s")
           << "，有效样本 " << (fitmode == "angle" ? vt.size() : wt.size())
           << "，帧范围 [" << vidx.front() << ", " << vidx.back() << "]\n";
        md << "- 参考指标：角度 RMSE = " << rmse_angle << " rad；角速度 RMSE = "
           << rmse_omega << " rad/s\n\n";
        md << "## 5. 输出文件\n\n";
        md << "- tracking_overlay.mp4：叠加中心、目标轮廓、中心连线、ID 与状态\n";
        md << "- fit_comparison.png：观测与拟合曲线对比\n";
        md << "- angular_velocity.png：估计角速度曲线\n";
        md << "- residuals.png：残差曲线\n";
        md.close();
        std::cout << "[OK]   " << outdir << "/../task2_fit_result.md\n";
    }

    std::cout << "[DONE] Task 2 finished.\n";
    return summary.IsSolutionUsable() ? 0 : 1;
}