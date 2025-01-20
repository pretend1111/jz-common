#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <iostream>

int main() {
    // 1. 加载ONNX模型
    std::string onnx_model_path = "/home/pretend/code/dev_ws/src/armor_identify/video/best.onnx"; // 替换为你的ONNX模型路径
    cv::dnn::Net net = cv::dnn::readNetFromONNX(onnx_model_path);

    // 2. 打开视频文件
    std::string video_path = "/home/pretend/code/dev_ws/src/armor_identify/video/6.mp4"; // 替换为你的视频路径
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return -1;
    }

    // 3. 逐帧读取视频
    cv::Mat frame, blob, result;
    while (cap.read(frame)) {
        // 4. 预处理帧
        cv::resize(frame, frame, cv::Size(640, 640)); // 调整为模型输入尺寸
        blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

        // 5. 进行推理
        net.setInput(blob);
        result = net.forward();

        // 6. 解析结果（以目标检测为例）
        for (int i = 0; i < result.size[2]; ++i) {
            float* detection = result.ptr<float>(i); // 获取第i个检测框的数据指针
            float confidence = detection[4]; // 置信度，通常是第5个元素
            if (confidence > 0.5) { // 置信度阈值
                int x1 = static_cast<int>(detection[0] * frame.cols);
                int y1 = static_cast<int>(detection[1] * frame.rows);
                int x2 = static_cast<int>(detection[2] * frame.cols);
                int y2 = static_cast<int>(detection[3] * frame.rows);

                // 绘制检测框
                cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            }
        }

        // 7. 显示结果
        cv::imshow("Frame", frame);

        if (cv::waitKey(1) == 'q') { // 按下'q'退出
            break;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;
}