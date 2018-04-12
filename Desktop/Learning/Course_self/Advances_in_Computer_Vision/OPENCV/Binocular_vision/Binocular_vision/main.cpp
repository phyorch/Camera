#include "calibration.h"
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	see_help();
	int cam;
	string path[2] = { "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/Opencv/Binocular_vision/left.xml",
		"C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/Opencv/Binocular_vision/right.xml" };
	//! [file_read]
	//bool state = false;
	for (cam = 1; cam <= 2; cam++)
	{
		Settings s;
		const string inputSettingsFile = argc > cam ? argv[cam-1] : path[cam-1]; //如果不另设置参数文件，则使用默认参数文件
		FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
		if (!fs.isOpened())
		{
			cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
			system("pause");
			return -1;
		}

		//fs["Settings"] >> s;
		fs["Settings"] >> s;
		fs.release();                                       // close Settings file
															//! [file_read]

															//FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
															//fout << "Settings" << s;

		if (!s.goodInput)
		{
			cout << "Invalid input detected. Application stopping. " << endl;
			return -1;
		}

		vector<vector<Point2d> > imagePoints;
		Mat cameraMatrix, distCoeffs;
		Size imageSize;
		int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
		clock_t prevTimestamp = 0;
		const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
		const char ESC_KEY = 27;

		//! [get_input]
		int count = 1;
		for (;;)
		{
			Mat view;
			bool blinkOutput = false;
			view = s.nextImage();

			//-----  If no more image, or got enough, then stop calibration and show result -------------
			//如果imagePoints中已经储存了足够多的点，那么就进行相机标定
			if (mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames)
			{
				cout << "CAPTURING" << endl;
				if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
					mode = CALIBRATED;
				else
					mode = DETECTION;
			}
			if (view.empty())          // If there are no more images stop the loop
			{
				cout << "STREAM END" << endl;
				// if calibration threshold was not reached yet, calibrate now
				if (mode != CALIBRATED && !imagePoints.empty())
					runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
				break;
			}
			//! [get_input]

			imageSize = view.size();  // Format input image.
			if (s.flipVertical)    flip(view, view, 0);

			//! [find_pattern]
			vector<Point2f> pointBuf;

			bool found;

			int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

			if (!s.useFisheye) {
				// fast check erroneously fails with high distortions like fisheye
				chessBoardFlags |= CALIB_CB_FAST_CHECK;
			}

			switch (s.calibrationPattern) // Find feature points on the input format
			{
			case Settings::CHESSBOARD:
				found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
				break;
			case Settings::CIRCLES_GRID:
				found = findCirclesGrid(view, s.boardSize, pointBuf);
				break;
			case Settings::ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				found = false;
				break;
			}
			//! [find_pattern]
			//! [pattern_found]
			if (found)                // If done with success,
			{
				// improve the found corners' coordinate accuracy for chessboard
				if (s.calibrationPattern == Settings::CHESSBOARD)
				{
					Mat viewGray;
					cvtColor(view, viewGray, COLOR_BGR2GRAY);
					cornerSubPix(viewGray, pointBuf, Size(11, 11),
						Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
				}

				if (mode == CAPTURING &&  // For camera only take new samples after delay time
					(!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC))
				{
					imagePoints.resize(imagePoints.size() + 1);
						
					for(auto p : pointBuf) {
						(imagePoints[imagePoints.size()-1]).push_back(cv::Point2d(p));

					}
					prevTimestamp = clock();
					blinkOutput = s.inputCapture.isOpened();
				}

				// Draw the corners.
				drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
			}
			//! [pattern_found]
			//----------------------------- Output Text ------------------------------------------------
			//! [output_text]
			string msg = (mode == CAPTURING) ? "100/100" :
				mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
			int baseLine = 0;
			Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
			Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

			if (mode == CAPTURING)
			{
				if (s.showUndistorsed)
					msg = format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
				else
					msg = format("%d/%d", (int)imagePoints.size(), s.nrFrames);
			}

			putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

			if (blinkOutput)
				bitwise_not(view, view);
			//! [output_text]
			//------------------------- Video capture  output  undistorted ------------------------------
			//! [output_undistorted]
			/*if (mode == CALIBRATED && s.showUndistorsed)
			{
				Mat temp = view.clone();
				if (s.useFisheye)
					cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
				else
					undistort(temp, view, cameraMatrix, distCoeffs);
			}*/
			//! [output_undistorted]
			//------------------------------ Show image and check for input commands -------------------
			//! [await_input]
			imshow("Image View", view);
			char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

			if (key == ESC_KEY)
				break;

			if (key == 'u' && mode == CALIBRATED)
				s.showUndistorsed = !s.showUndistorsed;

			if (s.inputCapture.isOpened() && key == 'g')
			{
				mode = CAPTURING;
				imagePoints.clear();
			}
			//! [await_input]
			count++;
		}

		// -----------------------Show the undistorted image for the image list ------------------------
		//! [show_results]
		if (s.inputType == Settings::IMAGE_LIST && s.showUndistorsed)
		{
			Mat view, rview, map1, map2;

			if (s.useFisheye)
			{
				Mat newCamMat;
				fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
					Matx33d::eye(), newCamMat, 1);
				fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
					CV_16SC2, map1, map2);
			}
			else
			{
				initUndistortRectifyMap(
					cameraMatrix, distCoeffs, Mat(),/////
					getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
					CV_16SC2, map1, map2);
			}

			for (size_t i = 0; i < s.imageList.size(); i++)
			{
				view = imread(s.imageList[i], IMREAD_COLOR);
				if (view.empty())
					continue;
				remap(view, rview, map1, map2, INTER_LINEAR);/////
				imshow("Image View", rview);
				char c = (char)waitKey();
				if (c == ESC_KEY || c == 'q' || c == 'Q')
					break;
			}
		}
	}
	
	
	
	//------------------------------Stereo calibration and rectification---------------------------
	Mat R, T, E, F;
	Mat Rl, Rr, Pl, Pr, Q;
	const char ESC_KEY = 27;
	const string left_para = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/OPENCV/Binocular_vision/left_output.xml";
	const string right_para = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/OPENCV/Binocular_vision/right_output.xml";
	Camera cl(left_para);
	Camera cr(right_para);
	Settings sl, sr;

	string inputSettingsFile = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/OPENCV/Binocular_vision/left.xml";
	FileStorage fsl(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fsl.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		system("pause");
		return -1;
	}
	fsl["Settings"] >> sl;
	fsl.release();

	inputSettingsFile = "C:/Users/Phyorch/Desktop/Learning/Course_self/Advances_in_Computer_Vision/OPENCV/Binocular_vision/right.xml";
	FileStorage fsr(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fsr.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		system("pause");
		return -1;
	}
	fsr["Settings"] >> sr;
	fsr.release();

	cl.loadCameraParameters();
	cr.loadCameraParameters();
	cl.mat_initilize();
	cr.mat_initilize();
	rectifyCamera(sl, cl, cr, R, T, E, F, Rl, Rr, Pl, Pr, Q);
	remapImageAndSave(sl, sr, cl, cr, ESC_KEY);
	return 0;
}