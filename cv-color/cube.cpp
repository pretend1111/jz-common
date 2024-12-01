#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Scalar lower_yellow = Scalar(20, 100, 100);
Scalar upper_yellow = Scalar(30, 255, 255);
Scalar lower_blue = Scalar(70, 91, 0);
Scalar upper_blue = Scalar(130,255,255);
// int hmin=90,smin=70,vmin=70;
// int hmax=130,smax=255,vmax=255;


void contoury(Mat mask, Mat img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> conpoly;
        Rect rect;
        float p = arcLength(contours[i], true);
        approxPolyDP(contours[i], conpoly, 0.02 * p, true);
        rect = boundingRect(conpoly);
        rectangle(img, rect.tl(), rect.br(), Scalar(0, 255, 255), 2);
        putText(img, "yellow", Point(rect.tl().x,rect.tl().y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
    }
}
void contourb(Mat mask, Mat img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> conpoly;
        Rect rect;
        int area = contourArea(contours[i]);
        if(area>5000){
        float p = arcLength(contours[i], true);
        approxPolyDP(contours[i], conpoly, 0.02 * p, true);
        rect = boundingRect(conpoly);
        rectangle(img, rect.tl(), rect.br(), Scalar(255, 0, 0), 2);
        putText(img, "blue", Point(rect.tl().x,rect.tl().y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
        }
    }
}


int main()
{
    auto cube = imread("output.png");

    // namedWindow("track", (360,200));
    // createTrackbar("hue", "track", &hmin, 179);
    // createTrackbar("sat", "track", &smin, 255);
    // createTrackbar("val", "track", &vmin, 255);
    // createTrackbar("hue2", "track", &hmax, 179);
    // createTrackbar("sat2", "track", &smax, 255);
    // createTrackbar("val2", "track", &vmax, 255);

    Mat hsv, maskyellow, maskblue;
    cvtColor(cube, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_yellow, upper_yellow, maskyellow);


    inRange(hsv, lower_blue, upper_blue, maskblue);
    contoury(maskyellow, cube);
    contourb(maskblue, cube);
    imshow("img", cube);
    imshow("maskblue",maskblue);
    waitKey(0);
    return 0;
}