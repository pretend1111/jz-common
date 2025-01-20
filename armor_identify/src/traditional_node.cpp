#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp> // ROS2核心库
#include <std_msgs/msg/string.hpp> // 用于发布字符串消息

using namespace std;
using namespace cv;

#define ENEMY_RED 0
#define ENEMY_BLUE 1

// 定义参数结构体
struct Parameters
{
    int grayThreshold_RED;
    int separationThreshold_RED;
    int separationThreshold_GREEN;
};

Parameters _para = {50, 50, 50}; // 初始化参数

vector<int> areas;
vector<Rect> rects_ROI;
vector<Point2d> Start_Points;
int height_video, width_video;

// ROS2相关变量
rclcpp::Node::SharedPtr node; // ROS2节点
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr enemy_color_pub; // 发布者
std_msgs::msg::String enemy_color_msg; // 消息

// 函数声明
void processFrame(Mat &img, int enemyColor);
void findAndDrawContours(Mat &maxColor, Mat &img);
Mat ROI(Mat &img, vector<Rect> &rects_ROI, vector<Point2d> &Start_Points);

int main(int argc, char **argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("enemy_color_detector");
    enemy_color_pub = node->create_publisher<std_msgs::msg::String>("enemy_color", 10);

    cout << "请输入敌方颜色：(红色为0,蓝色为1)" << endl;
    int enemyColor;
    cin >> enemyColor;

    VideoCapture cap("/home/pretend/code/dev_ws/src/armor_identify/video/6.mp4");
    Mat img;
    Start_Points.push_back(Point(0, 0));

    if (!cap.isOpened())
    {
        cout << "未能打开视频！" << endl;
        return -1;
    }

    while (true)
    {
        cap >> img;

        if (img.empty())
        {
            break;
        }

        processFrame(img, enemyColor);
        imshow("video", img);

        if (waitKey(20) >= 0)
        {
            break;
        }
    }

    rclcpp::shutdown(); // 关闭ROS2节点
    return 0;
}

void processFrame(Mat &img, int enemyColor)
{
    Mat dst = img.clone();
    img.convertTo(img, -1, 1.0, -100);
    Mat img_ROI = img.clone();
    img_ROI = ROI(img, rects_ROI, Start_Points);
    Size imgSize = img.size();
    width_video = imgSize.width;
    height_video = imgSize.height;
    Mat src = img_ROI.clone();
    vector<Mat> splitSrc;
    cv::split(src, splitSrc);
    Mat graySrc, separationSrcWhite, maxColor;

    cv::cvtColor(src, graySrc, cv::COLOR_BGR2GRAY);
    cv::threshold(graySrc, separationSrcWhite, 240, 255, cv::THRESH_BINARY);
    cv::bitwise_not(separationSrcWhite, separationSrcWhite);

    Mat separationSrc;
    if (enemyColor == ENEMY_RED)
    {
        cv::threshold(graySrc, graySrc, _para.grayThreshold_RED, 255, cv::THRESH_BINARY);
        cv::subtract(splitSrc[2], splitSrc[0], separationSrc);
        cv::threshold(separationSrc, separationSrc, _para.separationThreshold_RED, 255, cv::THRESH_BINARY);
        cv::dilate(separationSrc, separationSrc, getStructuringElement(MORPH_RECT, Size(3, 3)));
        maxColor = separationSrc & graySrc & separationSrcWhite;
        cv::dilate(maxColor, maxColor, getStructuringElement(MORPH_RECT, Size(1, 1)));
    }
    else if (enemyColor == ENEMY_BLUE)
    {
        cv::threshold(graySrc, graySrc, _para.grayThreshold_RED, 255, cv::THRESH_BINARY);
        cv::subtract(splitSrc[0], splitSrc[2], separationSrc);
        cv::threshold(separationSrc, separationSrc, _para.separationThreshold_RED, 255, cv::THRESH_BINARY);
        cv::dilate(separationSrc, separationSrc, getStructuringElement(MORPH_RECT, Size(3, 3)));
        maxColor = separationSrc & graySrc & separationSrcWhite;
        cv::dilate(maxColor, maxColor, getStructuringElement(MORPH_RECT, Size(1, 1)));
    }
    //imshow("maxColor", maxColor);
    img = dst;
    findAndDrawContours(maxColor, img);

    // 发布敌人的颜色
    if (enemyColor == ENEMY_RED)
    {
        enemy_color_msg.data = "RED";
    }
    else if (enemyColor == ENEMY_BLUE)
    {
        enemy_color_msg.data = "BLUE";
    }
    enemy_color_pub->publish(enemy_color_msg);
}

void findAndDrawContours(Mat &maxColor, Mat &img)
{
    vector<vector<Point>> contours;
    vector<vector<Point>> contours_clone;
    vector<Vec4i> hierarchy;
    vector<Rect> rects;
    vector<Rect> rects_clone;
    vector<int> deletedIndices;

    findContours(maxColor, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours)
    {
        vector<Point> approxPoly;
        float perimeter = arcLength(contour, true);
        approxPolyDP(contour, approxPoly, 0.02 * perimeter, true);
        rects.push_back(boundingRect(approxPoly));
    }

    size_t n = rects.size();
    float minRatio = 1.0;
    int num1 = -1, num2 = -1;

    for (size_t i = 0; i < n; i++)
    {
        if (areas.size() == 2)
        {
            float ratio1, ratio2;

            if (areas[0] < rects[i].area())
                ratio1 = static_cast<float>(areas[0]) / rects[i].area();
            else
                ratio1 = static_cast<float>(rects[i].area()) / areas[0];
            if (areas[1] < rects[i].area())
                ratio2 = static_cast<float>(areas[1]) / rects[i].area();
            else
                ratio2 = static_cast<float>(rects[i].area()) / areas[1];

            if (ratio1 < 0.5 && ratio2 < 0.5)
            {
                deletedIndices.push_back(i);
            }
        }
    }
    rects_clone = rects;
    contours_clone = contours;
    // int deleteted = deletedIndices.size();
    // cout << "deleteted: " << deleteted << endl;

    for (size_t i = 0; i < deletedIndices.size(); i++)
    {
        rects.erase(rects.begin() + deletedIndices[i] - i);
        contours_clone.erase(contours_clone.begin() + deletedIndices[i] - i);
    }

    n = rects.size();

    if (n == 2)
    {
        float r1, r2, r3, r4;
        r1 = static_cast<float>(rects[num1].width) / rects[num1].height;
        r2 = static_cast<float>(rects[num2].width) / rects[num2].height;
        r3 = static_cast<float>(rects[num1].height) / abs(rects[num1].y - rects[num2].y);
        r4 = static_cast<float>(rects[num2].height) / abs(rects[num1].y - rects[num2].y);
        if (r1 > 1 || r2 > 1)
            rects = rects_clone;
        if (r3 < 1 || r4 < 1)
            rects = rects_clone;
    }

    n = rects.size();
    if (n == 2)
    {
        num1 = 0;
        num2 = 1;
    }
    else if (n > 2)
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                float ratio1, ratio2, ratio3, ratio4, ratio5, r1, r2, r3, r4, r5, r6, r7, r8, ratio;

                r1 = static_cast<float>(rects[i].width) / rects[i].height;
                r2 = static_cast<float>(rects[j].width) / rects[j].height;
                r3 = static_cast<float>(rects[i].height) / abs(rects[i].y - rects[j].y);
                r4 = static_cast<float>(rects[j].height) / abs(rects[i].y - rects[j].y);
                r5 = static_cast<float>(rects[i].area()) / (rects[j].area());
                r6 = static_cast<float>(rects[j].area()) / (rects[i].area());
                r7 = static_cast<float>(rects[i].width) / abs(rects[j].x - rects[i].x);
                r8 = static_cast<float>(rects[j].width) / abs(rects[j].x - rects[i].x);

                if (r5 < 0.25 || r6 < 0.25 || r5 > 5 || r6 > 5)
                    break;
                if (r3 < 1 || r4 < 1)
                    break;
                if (r1 > 1 || r2 > 1)
                    break;
                else if (r1 < r2)
                    ratio4 = r1 / r2;
                else
                    ratio4 = r2 / r1;
                if (r7 < 0.05 || r8 < 0.05)
                    break;

                if (rects[i].area() < 50)
                    break;
                else if (rects[i].area() < rects[j].area())
                    ratio1 = static_cast<float>(rects[i].area()) / rects[j].area();
                else
                    ratio1 = static_cast<float>(rects[j].area()) / rects[i].area();

                if (rects[i].height < rects[j].height)
                    ratio2 = static_cast<float>(rects[i].height) / rects[j].height;
                else
                    ratio2 = static_cast<float>(rects[j].height) / rects[i].height;

                if (rects[i].width < rects[j].width)
                    ratio3 = static_cast<float>(rects[i].width) / rects[j].width;
                else
                    ratio3 = static_cast<float>(rects[j].width) / rects[i].width;

                if (rects[i].br().y < rects[j].br().y)
                    ratio5 = 1 - (static_cast<float>(rects[j].br().y) - rects[i].br().y) / rects[j].br().y;
                else
                    ratio5 = 1 - (static_cast<float>(rects[i].br().y) - rects[j].br().y) / rects[j].br().y;

                ratio = 1 - ratio1 * ratio2 * ratio3 * ratio4 * ratio5;

                if (ratio < minRatio)
                {
                    minRatio = ratio;
                    num1 = i;
                    num2 = j;
                }
            }
        }
    }
    else
    {
        areas.clear();
        rects_ROI.clear();
        Start_Points.clear();
        Start_Points.push_back(Point(0, 0));
        return;
    }

    if (num1 == -1 || num2 == -1)
    {
        areas.clear();
        rects_ROI.clear();
        Start_Points.clear();
        Start_Points.push_back(Point(0, 0));
        return;
    }

    areas.clear();
    areas.push_back(rects[num1].area());
    areas.push_back(rects[num2].area());

    // rectangle(img, Point(rects[num1].tl().x + Start_Points[0].x, rects[num1].tl().y + Start_Points[0].y), Point(rects[num1].br().x + Start_Points[0].x, rects[num1].br().y + Start_Points[0].y), Scalar(0, 0, 255), 2);
    // rectangle(img, Point(rects[num2].tl().x + Start_Points[0].x, rects[num2].tl().y + Start_Points[0].y), Point(rects[num2].br().x + Start_Points[0].x, rects[num2].br().y + Start_Points[0].y), Scalar(0, 0, 255), 2);
    // cout <<contours.size()<<endl;

    // for (size_t i = 0; i < contours.size(); ++i)
    // {
    //     RotatedRect rrect = minAreaRect(contours[i]);
    //     Point2f points[4];
    //     rrect.points(points);

    //     for (int j = 0; j < 4; ++j)
    //     {
    //         line(img, Point(points[j].x + Start_Points[0].x, points[j].y + Start_Points[0].y), Point(points[(j + 1) % 4].x+Start_Points[0].x,points[(j + 1) % 4].y+Start_Points[0].y), Scalar(0, 0, 255), 2);
    //     }
    // }
    if (contours.size() == 2)
    {
        RotatedRect rrect1 = minAreaRect(contours[0]);
        Point2f points1[4];
        rrect1.points(points1);
        RotatedRect rrect2 = minAreaRect(contours[1]);
        Point2f points2[4];
        rrect2.points(points2);
        if (points1[0].x < points2[0].x || points1[0].y < points2[0].y)
        {
            line(img, Point(points1[0].x + Start_Points[0].x, points1[0].y + Start_Points[0].y), Point(points1[3].x + Start_Points[0].x, points1[3].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            line(img, Point(points1[0].x + Start_Points[0].x, points1[0].y + Start_Points[0].y), Point(points2[1].x + Start_Points[0].x, points2[1].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            line(img, Point(points2[1].x + Start_Points[0].x, points2[1].y + Start_Points[0].y), Point(points2[2].x + Start_Points[0].x, points2[2].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            line(img, Point(points1[3].x + Start_Points[0].x, points1[3].y + Start_Points[0].y), Point(points2[2].x + Start_Points[0].x, points2[2].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            //cout<<"1"<<endl;
        }
        else
        {
            line(img, Point(points2[1].x + Start_Points[0].x, points2[1].y + Start_Points[0].y), Point(points2[0].x + Start_Points[0].x, points2[0].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            line(img, Point(points2[1].x + Start_Points[0].x, points2[1].y + Start_Points[0].y), Point(points1[2].x + Start_Points[0].x, points1[2].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            line(img, Point(points1[2].x + Start_Points[0].x, points1[2].y + Start_Points[0].y), Point(points1[3].x + Start_Points[0].x, points1[3].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            line(img, Point(points2[0].x + Start_Points[0].x, points2[0].y + Start_Points[0].y), Point(points1[3].x + Start_Points[0].x, points1[3].y + Start_Points[0].y), Scalar(0, 0, 255), 2);
            //cout<<"2"<<endl;
        }
    }
    rects_ROI.clear();
    rects_ROI.push_back(rects[num1]);
    rects_ROI.push_back(rects[num2]);
}

Mat ROI(Mat &img, vector<Rect> &rects_ROI, vector<Point2d> &Start_Points)
{
    if (rects_ROI.size() == 2)
    {
        if (rects_ROI[0].tl().x < rects_ROI[1].tl().x)
        {
            int width, height;
            Point2d start_point;
            width = rects_ROI[1].br().x - rects_ROI[0].tl().x + 60;
            if (rects_ROI[0].tl().y < rects_ROI[1].tl().y)
            {
                height = rects_ROI[1].br().y - rects_ROI[0].tl().y + 60;
                start_point = Point2d(rects_ROI[0].tl().x - 30 + Start_Points[0].x, rects_ROI[0].tl().y - 30 + Start_Points[0].y);
            }
            else
            {
                height = rects_ROI[0].br().y - rects_ROI[1].tl().y + 60;
                start_point = Point2d(rects_ROI[0].tl().x - 30 + Start_Points[0].x, rects_ROI[1].tl().y - 30 + Start_Points[0].y);
            }

            if (start_point.x + width > width_video)
                width = width_video - start_point.x;
            if (start_point.y + height > height_video)
                height = height_video - start_point.y;
            if (start_point.x < 0)
                start_point.x = 0;
            if (start_point.y < 0)
                start_point.y = 0;

            Start_Points.clear();
            Start_Points.push_back(start_point);
            Rect ROI = Rect(start_point, Size(width, height));
            Mat ROI_img = img(ROI);
            // rectangle(img, ROI.tl(), ROI.br(), Scalar(0, 0, 255), 2);
            //  cout << "0" << endl;
            return ROI_img;
        }
        else
        {
            int width, height;
            Point2d start_point;
            width = rects_ROI[0].br().x - rects_ROI[1].tl().x + 60;
            if (rects_ROI[1].tl().y < rects_ROI[0].tl().y)
            {
                height = rects_ROI[0].br().y - rects_ROI[1].tl().y + 60;
                start_point = Point2d(rects_ROI[1].tl().x - 30 + Start_Points[0].x, rects_ROI[1].tl().y - 30 + Start_Points[0].y);
            }
            else
            {
                height = rects_ROI[1].br().y - rects_ROI[0].tl().y + 60;
                start_point = Point2d(rects_ROI[1].tl().x - 30 + Start_Points[0].x, rects_ROI[0].tl().y - 30 + Start_Points[0].y);
            }

            if (start_point.x + width > width_video)
                width = width_video - start_point.x;
            if (start_point.y + height > height_video)
                height = height_video - start_point.y;
            if (start_point.x < 0)
                start_point.x = 0;
            if (start_point.y < 0)
                start_point.y = 0;

            Start_Points.clear();
            Start_Points.push_back(start_point);
            Rect ROI = Rect(start_point, Size(width, height));
            Mat ROI_img = img(ROI);
            // rectangle(img, ROI.tl(), ROI.br(), Scalar(0, 0, 255), 2);
            //  cout << "1" << endl;
            return ROI_img;
        }
    }
    else
    {
        return img;
    }
}