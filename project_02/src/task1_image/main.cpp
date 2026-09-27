#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

static bool saveImage(const Mat& img, const fs::path& output_dir, const string& filename)
{
    fs::path full_path = output_dir / filename;
    bool ok = imwrite(full_path.string(), img);

    if (ok) {
        cout << "[OK]   " << full_path.string() << "\n";
    } else {
        cout << "[FAIL] " << full_path.string() << "\n";
    }

    return ok;
}

int main(int argc, char** argv)
{
    // 默认输入输出路径
    string input_path = "resources/test_image.jpg";
    string output_dir = "result/task1_images";

    if (argc > 1) {
        input_path = argv[1];
    }

    if (argc > 2) {
        output_dir = argv[2];
    }

    // 创建输出目录
    fs::create_directories(output_dir);

    // 1. 读取图像
    Mat img = imread(input_path);
    if (img.empty()) {
        cerr << "Cannot read image: " << input_path << endl;
        return 1;
    }

    cout << "Input image: " << input_path << "\n";
    cout << "Image size: " << img.cols << " x " << img.rows << "\n\n";

    // 2. 灰度图
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    saveImage(gray, output_dir, "gray.png");

    // 3. HSV 三通道
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);

    saveImage(hsv_channels[0], output_dir, "hsv_h.png");
    saveImage(hsv_channels[1], output_dir, "hsv_s.png");
    saveImage(hsv_channels[2], output_dir, "hsv_v.png");

    // 4. 滤波对比
    Size filter_kernel_size(5, 5);
    double gaussian_sigma = 1.5;
    int median_ksize = 5;

    Mat mean_img, gaussian_img, median_img;

    blur(img, mean_img, filter_kernel_size);
    GaussianBlur(img, gaussian_img, filter_kernel_size, gaussian_sigma);
    medianBlur(img, median_img, median_ksize);

    saveImage(mean_img, output_dir, "mean_filter.png");
    saveImage(gaussian_img, output_dir, "gaussian_filter.png");
    saveImage(median_img, output_dir, "median_filter.png");

    // 5. HSV 红色提取
    Mat img_for_mask;
    GaussianBlur(img, img_for_mask, Size(5, 5), 1.5);

    Mat hsv_for_mask;
    cvtColor(img_for_mask, hsv_for_mask, COLOR_BGR2HSV);

    // 红色在 HSV 中跨越 0 附近和 179 附近
    Scalar lower_red1(0, 100, 100);
    Scalar upper_red1(10, 255, 255);

    Scalar lower_red2(170, 100, 100);
    Scalar upper_red2(179, 255, 255);

    Mat mask_low, mask_high, red_mask;
    inRange(hsv_for_mask, lower_red1, upper_red1, mask_low);
    inRange(hsv_for_mask, lower_red2, upper_red2, mask_high);
    bitwise_or(mask_low, mask_high, red_mask);

    saveImage(red_mask, output_dir, "red_mask.png");

    // 6. 形态学操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

    Mat eroded_img, dilated_img, opened_img, closed_img;

    erode(red_mask, eroded_img, kernel);
    dilate(red_mask, dilated_img, kernel);
    morphologyEx(red_mask, opened_img, MORPH_OPEN, kernel);
    morphologyEx(red_mask, closed_img, MORPH_CLOSE, kernel);

    saveImage(eroded_img, output_dir, "erode.png");
    saveImage(dilated_img, output_dir, "dilate.png");
    saveImage(opened_img, output_dir, "open.png");
    saveImage(closed_img, output_dir, "close.png");

    // 7. 轮廓提取与面积筛选
    Mat contour_source = opened_img.clone();

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(
        contour_source,
        contours,
        hierarchy,
        RETR_EXTERNAL,
        CHAIN_APPROX_SIMPLE
    );

    double image_area = static_cast<double>(img.total());
    double min_area   = image_area * 0.0015;
    double max_area   = image_area * 0.60;

    Mat contour_result = img.clone();
    vector<double> selected_areas;

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);

        if (area < min_area || area > max_area) {
            continue;
        }

        selected_areas.push_back(area);

        Rect box = boundingRect(contours[i]);

        // 绘制轮廓
        drawContours(
            contour_result,
            contours,
            static_cast<int>(i),
            Scalar(0, 255, 0),
            2
        );

        // 绘制外接矩形
        rectangle(
            contour_result,
            box,
            Scalar(0, 0, 255),
            2
        );

        // 在结果图上标明面积
        string label = cv::format("area=%d", static_cast<int>(area));
        int base_line = 0;
        Size text_size = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.6, 2, &base_line);
        Point text_pos(box.x, max(box.y - 8, text_size.height + 4));  // 防止越界
        putText(contour_result, label, text_pos,
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
    }

    saveImage(contour_result, output_dir, "contours_boxes.png");

    // 8. 绘制圆、矩形和文字
    Mat drawing = img.clone();

    Point image_center(img.cols / 2, img.rows / 2);

    // 绘制圆
    circle(
        drawing,
        image_center,
        min(img.cols, img.rows) / 8,
        Scalar(255, 0, 0),
        3
    );

    // 绘制矩形
    Rect draw_rect(
        img.cols / 8,
        img.rows / 8,
        img.cols / 4,
        img.rows / 4
    );

    rectangle(
        drawing,
        draw_rect,
        Scalar(0, 255, 0),
        3
    );

    // 绘制文字
    putText(
        drawing,
        "Task1 Drawing",
        Point(30, 50),
        FONT_HERSHEY_SIMPLEX,
        1.2,
        Scalar(0, 255, 255),
        2
    );

    saveImage(drawing, output_dir, "drawing.png");

    // 9. 绕图像中心旋转 35 度
    Point2f rotate_center(
        static_cast<float>(img.cols / 2.0),
        static_cast<float>(img.rows / 2.0)
    );

    double angle = 35.0;
    double scale = 1.0;

    Mat rotate_matrix = getRotationMatrix2D(rotate_center, angle, scale);

    Mat rotated_img;
    warpAffine(
        img,
        rotated_img,
        rotate_matrix,
        img.size(),
        INTER_LINEAR,
        BORDER_REPLICATE
    );

    saveImage(rotated_img, output_dir, "rotated_35deg.png");

    // 10. 裁剪左上角 1/4
    Rect crop_rect(
        0,
        0,
        img.cols / 2,
        img.rows / 2
    );

    Mat cropped_img = img(crop_rect).clone();
    saveImage(cropped_img, output_dir, "crop_top_left.png");

    // 11. 打印参数
    cout << "\n================ Task1 Parameters ================\n";
    cout << "Mean filter kernel: 5 x 5\n";
    cout << "Gaussian filter kernel: 5 x 5, sigmaX = " << gaussian_sigma << "\n";
    cout << "Median filter ksize: " << median_ksize << "\n";

    cout << "\nRed HSV threshold 1:\n";
    cout << "H: [" << int(lower_red1[0]) << ", " << int(upper_red1[0]) << "], ";
    cout << "S: [" << int(lower_red1[1]) << ", " << int(upper_red1[1]) << "], ";
    cout << "V: [" << int(lower_red1[2]) << ", " << int(upper_red1[2]) << "]\n";

    cout << "Red HSV threshold 2:\n";
    cout << "H: [" << int(lower_red2[0]) << ", " << int(upper_red2[0]) << "], ";
    cout << "S: [" << int(lower_red2[1]) << ", " << int(upper_red2[1]) << "], ";
    cout << "V: [" << int(lower_red2[2]) << ", " << int(upper_red2[2]) << "]\n";

    cout << "\nMorphology kernel: 5 x 5 ellipse\n";
    cout << "Contour area range: [" << min_area << ", " << max_area << "]\n";

    cout << "\nSelected contour areas:\n";
    if (selected_areas.empty()) {
        cout << "No contour passed the area filter.\n";
    } else {
        for (size_t i = 0; i < selected_areas.size(); ++i) {
            cout << "Contour " << i << ": " << selected_areas[i] << "\n";
        }
    }

    cout << "==================================================\n";

    return 0;
}