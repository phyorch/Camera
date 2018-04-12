#include "StereoCalibration.h"
#include "StereoMatching.h"
#include "videoio.hpp"
#include <iostream>
using namespace std;

int num_frames = 20;
string stereo_path = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/OPENCV/Stereo/";
string left_image_path = "left_image_list.xml";
string right_image_path = "right_image_list.xml";
string test_left_image_path = "test1.jpg";
string test_right_image_path = "test2.jpg";
bool calibrated = false;
bool captured = false;
bool realtime_disparity = true;

int main()
{
	cv::Size image_size = cv::Size(640, 480);
	cv::Size board_size = cv::Size(8, 6);
	float square_width = 36;
	string left_image = stereo_path + left_image_path;
	string right_image = stereo_path + right_image_path;
	vector<string> left_image_list;
	vector<string> right_image_list;
	readStringList(left_image, left_image_list);
	readStringList(right_image, right_image_list);

	if (captured == false)
	{
		cv::Mat frame1, frame2;
		cv::VideoCapture cap1, cap2;
		cap1.open(0);
		cap2.open(1);
		int count = 0;
		string image_msg;
		while (cap1.read(frame1) && cap2.read(frame2) && count <= num_frames)
		{
			if (frame1.empty() || frame2.empty())
			{
				cerr << "can not read frame";
				break;
			}

			if (count != num_frames)
				image_msg = "Please capture "" image" + to_string(count + 1);
			else
				image_msg = "Please capture test image";
			int baseLine = 0;

			cv::Size textSize = cv::getTextSize(image_msg, cv::FONT_HERSHEY_COMPLEX, 1, 1, &baseLine);
			cv::Point textOrigin((frame1.cols - textSize.width) / 2, (frame1.rows - textSize.height) / 2);
			cv::putText(frame1, image_msg, textOrigin, 1, 1, (0, 255, 0));
			cv::putText(frame2, image_msg, textOrigin, 1, 1, (0, 255, 0));


			cv::imshow("camera1", frame1);
			cv::imshow("camera2", frame2);
			char c = cv::waitKey(30);
			if (c == ' ')
			{
				cv::imwrite(left_image_list[count], frame1);
				cv::imwrite(right_image_list[count], frame2);
				count++;
			}
			if (count == num_frames && c == ' ')
			{
				cv::imwrite(stereo_path + test_left_image_path, frame1);
				cv::imwrite(stereo_path + test_right_image_path, frame2);
				cap1.release();
				cap2.release();
			}
		}
	}

	if (calibrated == false)
	{
		StereoCalib caliber = StereoCalib();
		caliber.initCornerData(num_frames, image_size, board_size, square_width);
		cv::Mat view1, view2;
		// read image corner points data
		for (int i = 0; i < num_frames; i++)
		{
			view1 = cv::imread(left_image_list[i]);
			view2 = cv::imread(right_image_list[i]);

			caliber.detectCorners(view1, view2, caliber.corner_datas, i);
		}
		caliber.calibrateStereoCamera(caliber.corner_datas, caliber.stereo_params, true);
		double error;
		caliber.getCameraCalibrateError(caliber.corner_datas.objectPoints, caliber.corner_datas.imagePoints1, caliber.stereo_params.cameraParams1, error);
		cout << "The left calibration error is " << error << endl;
		caliber.getCameraCalibrateError(caliber.corner_datas.objectPoints, caliber.corner_datas.imagePoints2, caliber.stereo_params.cameraParams2, error);
		cout << "The left calibration error is " << error << endl;
		caliber.getStereoCalibrateError(caliber.corner_datas, caliber.stereo_params, error);
		cout << "The stereo calibration error is " << error << endl;
		RECTIFYMETHOD method = RECTIFY_BOUGUET;
		caliber.rectifyStereoCamera(caliber.corner_datas, caliber.stereo_params, caliber.remap_matrixs, method);
		string para_path = stereo_path + "calibration_parameters.xml";
		caliber.saveCalibrationDatas(para_path, method, caliber.corner_datas, caliber.stereo_params, caliber.remap_matrixs);
	}



	StereoMatch matcher = StereoMatch();
	/*matcher.init(caliber.camera_params.imageSize.width, caliber.camera_params.imageSize.height, para_path);*/
	cv::Mat view1 = cv::imread(stereo_path + "test1.jpg");
	cv::Mat view2 = cv::imread(stereo_path + "test2.jpg");
	cv::Mat disparty, left, right;
	matcher.bmMatch(view1, view2, disparty, left, right);
	cv::imwrite(stereo_path + "disparty.jpg", disparty);
	cv::Mat color_disparty, point_cloud, topdown_view;
	matcher.getDisparityImage(disparty, color_disparty, true);
	cv::imwrite(stereo_path + "color_disparty.jpg", disparty);
	matcher.getPointClouds(disparty, point_cloud);
	matcher.getTopDownView(point_cloud, topdown_view, view1);
	cv::imwrite(stereo_path + "topdown.jpg", disparty);

	if (realtime_disparity)
	{
		cv::Mat frame1, frame2, disp, color;
		cv::VideoCapture cap1, cap2;
		cap1.open(0);
		cap2.open(1);
		while (cap1.read(frame1) && cap2.read(frame2))
		{
			matcher.bmMatch(frame1, frame2, disp, left, right);
			matcher.getDisparityImage(disp, color, true);
			cv::imshow("real time disparty map", color);
			cv::imshow("Reference map", frame1);
			char c = cv::waitKey(30);
			if (c == ' ')
			{
				break;
			}
		}
	}

	return 0;
}



//for (int i = 0; i < 8; i++)
//{
//	int ni1 = cv::InputArrayOfArrays(caliber.corner_datas.imagePoints2).getMat(i).checkVector(2, CV_32F);
//	cout << ni1 << endl;
//}
//cv::Mat q = cv::InputArrayOfArrays(caliber.corner_datas.imagePoints2).getMat(7);
//int wq = cv::InputArrayOfArrays(caliber.corner_datas.imagePoints2).getMat(7).checkVector(2, CV_32F);
//cout << q << wq << endl;
//此处曾遇到有图片为检测出角点的问题，以上是找出问题的测试代码