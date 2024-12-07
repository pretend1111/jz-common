#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
 
using namespace std;
using namespace cv;
 
int main()
{
	ifstream fin("calibdata.txt");                                  // 读取标定图片的路径，以输入方式打开文件
	if (!fin) 
	{
		cerr << "没有找到文件" << endl;
		return -1;
	}
	ofstream fout("calibration_result.txt");     // 输出结果保存在此文本文件下，以输出方式打开文件
 
	cout << "1、开始提取角点……" << endl;             // points_all_images
	int image_nums = 0;                                // 有效图片数量统计
	int points_per_row = 9;                            // 标定版每行的内点数
	int points_per_col = 6;                            // 标定版每列的内点数
	Size image_size;                                   // 图片尺寸
	Size corner_size(points_per_row, points_per_col);  // 标定板每行每列角点个数，共9*6个角点
	vector<Point2f> points_per_image;                  // 缓存每幅图检测到的角点
	vector<vector<Point2f>> points_all_images;         // 保存检测到的所有角点
	string image_file_name;			                   // 声明一个文件名的字符串
	Mat image_raw, image_gray;                         // 彩色图，灰度图
	while (getline(fin, image_file_name))
	{
		image_raw = imread(image_file_name);                                             // 按照RGB图像读取数据
		cvtColor(image_raw, image_gray, COLOR_BGR2GRAY);                                 // 将BGR图转化为灰度图
		bool success = findChessboardCorners(image_gray, corner_size, points_per_image); // 角点检测
		if (!success)
		{
			cout << "角点提取失败" << endl;
			exit(1);  // 非正常执行导致退出程序
		}
		else
		{
			find4QuadCornerSubpix(image_gray, points_per_image, Size(5, 5)); // 亚像素角点，也可使用cornerSubPix()
			points_all_images.push_back(points_per_image);                   // 保存亚像素角点
		}
		if (image_nums == 0)
		{
			cout << "channels = " << image_raw.channels() << endl;  // 图像的通道数
			cout << "image type = " << image_raw.type() << endl;    // 数据类型，CV_8UC3
			image_size.width = image_raw.cols;                      // 图像的宽,对应着列数(x)
			image_size.height = image_raw.rows;                     // 图像的高,对应着行数(y)
			cout << "image width = " << image_size.width << endl;   // 打印图像宽
			cout << "image height = " << image_size.height << endl; // 打印图像高
		}
		image_nums++;
	}
	cout << "image_nums = " << image_nums << endl;     // 输出图像数目
 
	cout << "2、开始计算角点3D坐标……" << endl;     // points3D_all_images
	Size block_size(10, 10);                         // 每个小方格实际大小10mm,(w,h)
	vector<Point3f> points3D_per_image;              // 初始化角点三维坐标,从左到右,从上到下
	Point3f point3D;                                 // 3D点(x,y,z)
	for (int i = 0; i < corner_size.height; i++)     // 第i行---y
	{
		for (int j = 0; j < corner_size.width; j++)  // 第j列---x
		{
			point3D = Point3f(block_size.width * j, block_size.height * i, 0);
			points3D_per_image.push_back(point3D);
		}
	}
	vector<vector<Point3f>> points3D_all_images(image_nums, points3D_per_image); // 保存所有图像角点的三维坐标
	int point_counts = corner_size.area();                                       // 每张图片上角点个数
 
	cout << "3、开始标定相机……" << endl;           // calibrateCamera
	Mat cameraMat(3, 3, CV_32FC1, Scalar::all(0));   // 内参矩阵3*3
	Mat distCoeffs(1, 5, CV_32FC1, Scalar::all(0));  // 畸变矩阵1*5，既考虑径向畸变，又考虑切向
	vector<Mat> rotationMat;                         // 旋转矩阵
	vector<Mat> translationMat;                      // 平移矩阵
	calibrateCamera(points3D_all_images, points_all_images, image_size, cameraMat, distCoeffs, rotationMat, translationMat, 0); // 标定
 
	cout << "4、开始对标定结果进行评价……" << endl; // projectPoints
	double total_err = 0.0;                          // 所有图像平均误差总和
	double err = 0.0;                                // 每幅图像的平均误差
	vector<Point2f> points_reproject;                // 重投影点
	fout << "计算每幅图像的标定误差：" << endl;
	for (int i = 0; i < image_nums; i++)
	{
		points_per_image = points_all_images[i];         // 第i张图像提取角点
		points3D_per_image = points3D_all_images[i];     // 第i张图像中角点的3D坐标
		projectPoints(points3D_per_image, rotationMat[i], translationMat[i], cameraMat, distCoeffs, points_reproject); // 重投影
		Mat detect_points_Mat(1, points_per_image.size(), CV_32FC2);  // 变为1*S的矩阵,2通道保存提取角点的像素坐标
		Mat points_reproj_Mat(1, points_reproject.size(), CV_32FC2);  // 变为1*S的矩阵,2通道保存投影角点的像素坐标
		for (int j = 0; j < points_per_image.size(); j++)
		{
			detect_points_Mat.at<Vec2f>(0, j) = Vec2f(points_per_image[j].x, points_per_image[j].y);
			points_reproj_Mat.at<Vec2f>(0, j) = Vec2f(points_reproject[j].x, points_reproject[j].y);
		}
		err = norm(points_reproj_Mat, detect_points_Mat, NormTypes::NORM_L2);  // 计算两者之间的误差
		total_err += err /= point_counts;
		fout << "第" << i + 1 << "幅图像的平均误差为： " << err << "像素" << endl;  
	}
	fout << "总体平均误差为： " << total_err / image_nums << "像素" << endl << endl;
 
	cout << "5、将标定结果写入文件……" << endl;
	fout << "相机内参数矩阵:" << endl << cameraMat << endl << endl;
	fout << "相机的畸变系数:" << endl << distCoeffs << endl << endl;
	Mat rotate_Mat = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 保存旋转矩阵
	for (int i = 0; i < image_nums; i++)
	{
		Rodrigues(rotationMat[i], rotate_Mat);  // 将旋转向量通过罗德里格斯公式转换为旋转矩阵
		fout << "第" << i + 1 << "幅图像的旋转矩阵为：" << endl << rotate_Mat << endl << endl;
		fout << "第" << i + 1 << "幅图像的平移向量为：" << endl << translationMat[i] << endl << endl;
	}
	fout << endl;
	fout.close();
 
	return 0;
}