#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace std;
using namespace cv;
int main(){
    auto img = imread("robot.png");
    Mat gray,thres,blur,canny,dia,ero;

    cvtColor(img, gray, COLOR_BGR2GRAY);
    threshold(gray, thres, 150, 255, THRESH_BINARY);
    GaussianBlur(img,blur, Size(7,7), 5, 0);
    Canny(blur, canny, 50, 150);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));
    dilate(canny, dia, kernel);
    erode(dia, ero, kernel);

    imshow("image", img);
    imshow("gray", gray);
    imshow("threshold", thres);
    imshow("blur", blur);
    imshow("canny", canny);
    imshow("dia", dia);
    imshow("erode", ero);
    waitKey();
    return 0;   
}
