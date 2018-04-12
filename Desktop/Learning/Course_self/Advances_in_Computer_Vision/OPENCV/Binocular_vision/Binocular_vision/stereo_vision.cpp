#include <iostream>
#include "calibration.h"

//camera_param


int Camera::loadCameraParameters()
{
	FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		system("pause");
		return -1;
	}
	fs["nr_of_frames"] >> nr_frames;
	fs["image_width"] >> image_size.width;
	fs["image_height"] >> image_size.height;
	fs["board_width"] >> board_size.width;
	fs["board_height"] >> board_size.height;
	fs["square_size"] >> square_size;
	fs["camera_matrix"] >> camera_mat;
	fs["distortion_coefficients"] >> dist_coeff;
	fs["image_points"] >> image_corners;
	fs.release();
	return 0;
}

int Camera::mat_initilize()
{
	image_corners_mat.create(image_corners.size(), image_corners[0].size(), CV_32FC2);
	return 0;
}

void Camera::get3DPoint(vector<Point3d> & corners)
{
	calcBoardCornerPositions(board_size, square_size, corners);
}


void rectifyCamera(Settings& setting, Camera& cameral, Camera& camerar, Mat& R, Mat& T, Mat& E, Mat& F,
	Mat& Rl, Mat& Rr, Mat& Pl, Mat& Pr, Mat& Q)
{
	/*
	R,T are the rotation matrix and translation matrix from camerl to camerar
	E is the essential matrix  F is the fundamental matrix
	Rl, Rr is Output 3x3 rectification transform (rotation matrix) for the  camera.
	Pl, Pr is Output 3x4 projection matrix in the new (rectified) coordinate systems for the camera.
	*/
	//vector<Point3f> object_corners;
	//vector<vector<Point3f>> objects_corners(1);
	std::vector<std::vector<cv::Point3d> > objects_corners(cameral.nr_frames);
	calcBoardCornerPositions(cameral.board_size, cameral.square_size, objects_corners[0], setting.calibrationPattern);
	objects_corners.resize(cameral.image_corners.size(), objects_corners[0]);
	
	if (setting.useFisheye)
	{
		
		/*Mat objects_corners_mat(objects_corners.size(), objects_corners[0].size(), CV_32FC3);
		for (int i = 0; i < objects_corners.size(); i++)
		{
			for (int j = 0; j < objects_corners[0].size(); j++)
			{
				objects_corners_mat.at<Point3f>(i, j) = objects_corners[i][j];
			}
		}
		for (int i = 0; i < cameral.image_corners.size(); i++)
		{
			for (int j = 0; j < cameral.image_corners[0].size(); j++)
			{
				cameral.image_corners_mat.at<Point2f>(i, j) = cameral.image_corners[i][j];
				camerar.image_corners_mat.at<Point2f>(i, j) = camerar.image_corners[i][j];
			}
		}*/
		//cout << "a" << cameral.image_corners_mat;
		//Mat objects_corners_mat(objects_corners);

		fisheye::stereoCalibrate(objects_corners, cameral.image_corners, camerar.image_corners, cameral.camera_mat,
			cameral.dist_coeff, camerar.camera_mat, camerar.dist_coeff, cameral.image_size, R, T);
		
	}
	else
	{
		stereoCalibrate(objects_corners, cameral.image_corners, camerar.image_corners, cameral.camera_mat,
			cameral.dist_coeff, camerar.camera_mat, camerar.dist_coeff, cameral.image_size, R, T, E, F);
	}

	//alpha setting

	if (setting.useFisheye)
	{
		fisheye::stereoRectify(cameral.camera_mat, cameral.dist_coeff, camerar.camera_mat, cameral.dist_coeff, setting.image_size,
			R, T, Rl, Rr, Pl, Pr, Q, 0);
	}
	else
	{
		stereoRectify(cameral.camera_mat, cameral.dist_coeff, camerar.camera_mat, cameral.dist_coeff, setting.image_size,
			R, T, Rl, Rr, Pl, Pr, Q);
	}

	if (setting.useFisheye)
	{
		fisheye::initUndistortRectifyMap(cameral.camera_mat, cameral.dist_coeff, Rl, Pl, cameral.image_size, CV_16SC2,
			cameral.mapx, cameral.mapy);
		fisheye::initUndistortRectifyMap(camerar.camera_mat, camerar.dist_coeff, Rr, Pr, cameral.image_size, CV_16SC2,
			camerar.mapx, camerar.mapy);
	}
	else
	{
		initUndistortRectifyMap(cameral.camera_mat, cameral.dist_coeff, Rl, Pl, cameral.image_size, CV_16SC2,
			cameral.mapx, cameral.mapy);
		initUndistortRectifyMap(camerar.camera_mat, camerar.dist_coeff, Rr, Pr, cameral.image_size, CV_16SC2,
			camerar.mapx, camerar.mapy);
	}
}



void remapImageAndSave(Settings& setting_left, Settings& setting_right, Camera& cameral, Camera& camerar, char ESC_KEY)
{
	if (setting_left.inputType == Settings::IMAGE_LIST && setting_left.showUndistorsed)
	{
		Mat lview, rview, lview_remap, rview_remap;

		string left_path = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/Opencv/Binocular_vision/left_remap_images";
		string right_path = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/Opencv/Binocular_vision/right_remap_images";
		for (size_t i = 0; i < setting_left.imageList.size(); i++)
		{
			lview = imread(setting_left.imageList[i], IMREAD_COLOR);
			rview = imread(setting_right.imageList[i], IMREAD_COLOR);
			if (lview.empty())
				continue;
			if (rview.empty())
				continue;
			remap(lview, lview_remap, cameral.mapx, cameral.mapy, INTER_LINEAR);/////
			remap(rview, rview_remap, camerar.mapx, camerar.mapy, INTER_LINEAR);/////
			imshow("lview_remap", lview_remap);
			imshow("rview_remap", rview_remap);
			imwrite(left_path + "image" + to_string(i) + ".jpg", lview_remap);
			imwrite(right_path + "image" + to_string(i) + ".jpg", rview_remap);
			char c = (char)waitKey();
			if (c == ESC_KEY || c == 'q' || c == 'Q')
				break;
		}
	}
}