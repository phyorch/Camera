#pragma once
#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;


class Camera
{
public:

	float square_size;
	Size board_size;
	Size image_size;
	Mat camera_mat;
	Mat dist_coeff;
	Mat mapx;
	Mat mapy;
	vector<vector<Point2d>> image_corners;
	Mat image_corners_mat;
	int nr_frames;
	string inputSettingsFile;

	Camera(string address) : inputSettingsFile(address) {};/////

	int loadCameraParameters();
	int mat_initilize();
	void get3DPoint(vector<Point3d> & corners);
};



class Settings
{
public:
	Settings(void);
	//~Settings(void);
	enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };
	void write(FileStorage& fs) const;
	void read(const FileNode& node);
	void validate();
	Mat nextImage();
	static bool readStringList(const string& filename, vector<string>& l);
	static bool isListOfImages(const string& filename);

public:
	Size boardSize;              // The size of the board -> Number of items by width and height
	Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames;                // The number of frames to use from the input for calibration
	float aspectRatio;           // The aspect ratio
	int delay;                   // In case of a video input
	bool writePoints;            // Write detected feature points
	bool writeExtrinsics;        // Write extrinsic parameters
	bool calibZeroTangentDist;   // Assume zero tangential distortion
	bool calibFixPrincipalPoint; // Fix the principal point at the center
	bool flipVertical;           // Flip the captured images around the horizontal axis
	string outputFileName;       // The name of the file where to write
	bool showUndistorsed;        // Show undistorted images after calibration
	string input;                // The input ->
	bool useFisheye;             // use fisheye camera model for calibration
	bool fixK1;                  // fix K1 distortion coefficient
	bool fixK2;                  // fix K2 distortion coefficient
	bool fixK3;                  // fix K3 distortion coefficient
	bool fixK4;                  // fix K4 distortion coefficient
	bool fixK5;                  // fix K5 distortion coefficient

	int cameraID;
	vector<string> imageList;
	size_t atImageList;
	VideoCapture inputCapture;
	InputType inputType;
	bool goodInput;
	int flag;
	double alpha;
	Size image_size;

private:
	string patternToUse;
};


enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

void see_help();


static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	//外部全局读取函数，用于opencv内部操作符>>调用，类似回调函数，其实是一个inline函数template
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}


static inline void write(FileStorage& fs, const String&, const Settings& s)
{
	s.write(fs);
}

static double computeReprojectionErrors(const vector<vector<Point3d> >& objectPoints,
	const vector<vector<Point2d> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors, bool fisheye);

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3d>& corners,
	Settings::Pattern patternType = Settings::CHESSBOARD);

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2d> > imagePoints);

void rectifyCamera(Settings& setting, Camera& cameral, Camera& camerar, Mat& R, Mat& T, Mat& E, Mat& F,
	Mat& Rl, Mat& Rr, Mat& Pl, Mat& Pr, Mat& Q);

void remapImageAndSave(Settings& setting_left, Settings& setting_right, Camera& cameral, Camera& camerar, char ESC_KEY);

#endif

