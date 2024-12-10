#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

Scalar lower_blue(100, 100, 100);
Scalar upper_blue(115, 255, 255);
Scalar lower_red(160, 160, 30);
Scalar upper_red(179, 255, 255);
Scalar number_down(129, 45, 216);
Scalar number_up(142, 88, 255);
Scalar number2_down(0, 0, 113);
Scalar number2_up(138, 89, 255);

// int hmin = 90, smin = 90, vmin = 77;
// int hmax = 130, smax = 255, vmax = 255;

void contourred(Mat mask, Mat img);
void contourblue(Mat mask, Mat img);
void contournumber(Mat mask, Mat img);

vector<Rect> numbers;

class Pnp_Solve
{
private:
    Mat rVec;           // 旋转向量
    Mat rotationMatrix; // 旋转矩阵
    Mat tVec;           // 平移向量
    Mat cam;            // 相机内参
    Mat dis;            // 畸变矩阵
public:
    Pnp_Solve();
    vector<Point3d> World_Coor;
    vector<Point2d> Img_Coor;
    void calculate_rtVec();      // 计算平移、旋转向量
    double calculate_distance(); // 计算距离
};

Point2d i1, i2, i3, i4;

int main()
{
    auto image = imread("2.png");

    double c = 0.5; // 控制变换强度的常数
    Mat logt;
    image.convertTo(logt, -1, c, c * log(1 + 255));

    Mat hsv, maskred, maskblue, hsv_number, masknumber;

    cvtColor(image, hsv_number, COLOR_BGR2HSV);
    inRange(hsv_number, number_down, number_up, masknumber);
    contournumber(masknumber, image);
    if (numbers.size() == 0)
    {
        inRange(hsv_number, number2_down, number2_up, masknumber);
        contournumber(masknumber, image);
    }

    cvtColor(logt, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_red, upper_red, maskred);
    contourred(maskred, image);

    inRange(hsv, lower_blue, upper_blue, maskblue);
    contourblue(maskblue, image);

    imshow("img", image);

    Pnp_Solve pnp_solver;
    Point3d w1 = {0, 0, 0}, w2 = {0.08, 0, 0}, w3 = {0.08, 0.08, 0},
            w4 = {0, 0.08, 0};

    pnp_solver.World_Coor.push_back(w1);
    pnp_solver.World_Coor.push_back(w2);
    pnp_solver.World_Coor.push_back(w3);
    pnp_solver.World_Coor.push_back(w4);
    pnp_solver.Img_Coor.push_back(i1);
    pnp_solver.Img_Coor.push_back(i2);
    pnp_solver.Img_Coor.push_back(i3);
    pnp_solver.Img_Coor.push_back(i4);
    pnp_solver.calculate_rtVec();
    cout << "装甲板距离摄像头"<<pnp_solver.calculate_distance()<<" m" << endl;

    waitKey(0);

    return 0;
}

void contournumber(Mat mask, Mat img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<Point> jiaodian;
    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> conpoly;
        Rect rect;
        int area = contourArea(contours[i]);
        // cout << area << endl;
        if (area > 100)
        {
            float p = arcLength(contours[i], true);
            approxPolyDP(contours[i], conpoly, 0.02 * p, true);
            rect = boundingRect(conpoly);
            double ratio = (double)rect.width / (double)rect.height;
            if (rect.width < rect.height && ratio > 0.6)
            {
                // rectangle(img, rect.tl(), rect.br(), Scalar(255, 0, 255), 2);
                numbers.push_back(rect);
            }
        }
    }
}

void contourred(Mat mask, Mat img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<Point> jiaodian;
    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> conpoly;
        Rect rect;
        int area = contourArea(contours[i]);
        // cout << area << endl;
        if (area > 100 && area < 1000)
        {
            float p = arcLength(contours[i], true);
            approxPolyDP(contours[i], conpoly, 0.02 * p, true);
            rect = boundingRect(conpoly);
            double ratio = (double)rect.width / (double)rect.height;
            if (numbers.size() == 1)
            {
                double center_num = (numbers[0].tl().y + numbers[0].br().y) / 2;
                double center_rect = (rect.tl().y + rect.br().y) / 2;
                double w = fabs(center_num - center_rect);
                if (w < 10 && ratio > 0.20 && ratio < 0.6 && rect.tl().x > (2 * numbers[0].tl().x - numbers[0].br().x) && rect.br().x < (2 * numbers[0].br().x - numbers[0].tl().x) && rect.tl().y > numbers[0].tl().y - 50)
                {
                    // rectangle(img, rect.tl(), rect.br(), Scalar(255, 0, 255), 2);
                    jiaodian.push_back(rect.tl());
                    jiaodian.push_back(rect.br());
                }
            }
        }
    }
    if (jiaodian.size() == 4)
    {
        Point aaa, bbb;
        if (jiaodian[0].x < jiaodian[2].x)
        {
            aaa = {jiaodian[0].x, static_cast<int>(jiaodian[0].y * 1.5 - jiaodian[1].y * 0.5)};
            bbb = {jiaodian[3].x, static_cast<int>(jiaodian[3].y * 1.5 - jiaodian[2].y * 0.5)};
        }
        else
        {
            aaa = {jiaodian[2].x, static_cast<int>(jiaodian[0].y * 1.5 - jiaodian[1].y * 0.5)};
            bbb = {jiaodian[1].x, static_cast<int>(jiaodian[3].y * 1.5 - jiaodian[2].y * 0.5)};
        }
        rectangle(img, aaa, bbb, Scalar(0, 0, 255), 2);
        i1 = Point2d(aaa.x, aaa.y);
        i2 = Point2d(bbb.x, aaa.y);
        i3 = Point2d(bbb.x, bbb.y);
        i4 = Point2d(aaa.x, bbb.y);
        putText(img, "red", Point(aaa.x, aaa.y - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    }
    // cout<<jiaodian.size()<<endl;
}

void contourblue(Mat mask, Mat img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<Point> jiaodian;
    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> conpoly;
        Rect rect;
        int area = contourArea(contours[i]);
        // cout << area << endl;
        if (area > 50 && area < 1000)
        {
            float p = arcLength(contours[i], true);
            approxPolyDP(contours[i], conpoly, 0.02 * p, true);
            rect = boundingRect(conpoly);
            double ratio = (double)rect.width / (double)rect.height;
            if (numbers.size() == 1)
            {
                double center_num = (numbers[0].tl().y + numbers[0].br().y) / 2;
                double center_rect = (rect.tl().y + rect.br().y) / 2;
                double w = fabs(center_num - center_rect);
                if (w < 10 && ratio > 0.2 && ratio < 0.6 && rect.tl().x > (2 * numbers[0].tl().x - numbers[0].br().x) && rect.tl().y > numbers[0].tl().y - 50)
                {
                    // rectangle(img, rect.tl(), rect.br(), Scalar(255, 0, 255), 2);
                    jiaodian.push_back(rect.tl());
                    jiaodian.push_back(rect.br());
                }
            }
        }
        if (jiaodian.size() == 4)
        {
            Point aaa, bbb;
            if (jiaodian[0].x < jiaodian[2].x)
            {
                aaa = {jiaodian[0].x, static_cast<int>(jiaodian[0].y * 1.5 - jiaodian[1].y * 0.5)};
                bbb = {jiaodian[3].x, static_cast<int>(jiaodian[3].y * 1.5 - jiaodian[2].y * 0.5)};
            }
            else
            {
                aaa = {jiaodian[2].x, static_cast<int>(jiaodian[0].y * 1.5 - jiaodian[1].y * 0.5)};
                bbb = {jiaodian[1].x, static_cast<int>(jiaodian[3].y * 1.5 - jiaodian[2].y * 0.5)};
            }
            rectangle(img, aaa, bbb, Scalar(255, 0, 0), 2);
            i1 = Point2d(aaa.x, aaa.y);
            i2 = Point2d(bbb.x, aaa.y);
            i3 = Point2d(bbb.x, bbb.y);
            i4 = Point2d(aaa.x, bbb.y);
            putText(img, "blue", Point(aaa.x, aaa.y - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
        }
    }
    // cout<<jiaodian.size()<<endl;
}

Pnp_Solve::Pnp_Solve()
{
    rVec = Mat::zeros(3, 1, CV_64FC1);
    tVec = Mat::zeros(3, 1, CV_64FC1);
    cam = (Mat_<double>(3, 3) << 1462.3697, 0, 398.59394, 0, 1469.68385,
           110.68997, 0, 0, 1);
    dis =
        (Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0000);
}

void Pnp_Solve::calculate_rtVec()
{
    // pnp解算
    solvePnP(World_Coor, Img_Coor, cam, dis, rVec, tVec, false,
             SOLVEPNP_ITERATIVE);
    // 转化为矩阵
    Rodrigues(rVec, rotationMatrix);
}

double Pnp_Solve::calculate_distance()
{
    double distance =
        sqrt(pow(tVec.at<double>(0, 0), 2) + pow(tVec.at<double>(1, 0), 2) +
             pow(tVec.at<double>(2, 0), 2));
    distance = round(distance * 1000) / 1000;
    return distance;
}