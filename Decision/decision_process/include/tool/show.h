#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sstream>

class CurveVisualizer {
private:
    cv::Mat image;  // 用于绘制曲线的图像
    int width;      // 图像的宽度
    int height;     // 图像的高度
    std::vector<double> values;  // 存储变量的值
    double minValue;  // 变量的最小值
    double maxValue;  // 变量的最大值
    int axisMargin;   // 纵轴与图像边缘的间距

    // 更新最小值和最大值
    void updateMinMax(double value) {
        if (values.empty()) {
            minValue = maxValue = value;
        } else {
            if (value < minValue) {
                minValue = value;
            }
            if (value > maxValue) {
                maxValue = value;
            }
        }
    }

    // 绘制纵轴
    void drawYAxis() {
        // 绘制纵轴线条
        cv::line(image, cv::Point(axisMargin, 0), cv::Point(axisMargin, height), cv::Scalar(255, 255, 255), 2);

        // 绘制刻度和标注
        int numTicks = 5;  // 刻度数量
        for (int i = 0; i <= numTicks; ++i) {
            int y = i * height / numTicks;
            cv::line(image, cv::Point(axisMargin - 5, y), cv::Point(axisMargin, y), cv::Scalar(255, 255, 255), 2);

            double value = minValue + (maxValue - minValue) * (1 - static_cast<double>(i) / numTicks);
            std::ostringstream oss;
            oss.precision(2);
            oss << std::fixed << value;
            std::string label = oss.str();
            cv::putText(image, label, cv::Point(5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }

public:
    // 构造函数，初始化图像的宽度和高度
    CurveVisualizer(int w, int h, int margin = 30) : width(w), height(h), axisMargin(margin) {
        image = cv::Mat::zeros(height, width, CV_8UC3);
        minValue = maxValue = 0;
    }

    // 添加一个新的值
    void addValue(double value) {
        values.push_back(value);
        updateMinMax(value);

        // 清除之前的图像
        image = cv::Scalar(0, 0, 0);

        // 绘制纵轴
        drawYAxis();

        // 绘制曲线
        if (values.size() > 1) {
            for (size_t i = 1; i < values.size(); ++i) {
                int x1 = axisMargin + (i - 1) * (width - axisMargin) / (values.size() - 1);
                int x2 = axisMargin + i * (width - axisMargin) / (values.size() - 1);
                int y1 = height - (values[i - 1] - minValue) * height / (maxValue - minValue + 1e-6);
                int y2 = height - (values[i] - minValue) * height / (maxValue - minValue + 1e-6);
                cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    // 显示曲线
    void show() {
        cv::imshow("Curve Visualization", image);
        cv::waitKey(1);
    }
};

