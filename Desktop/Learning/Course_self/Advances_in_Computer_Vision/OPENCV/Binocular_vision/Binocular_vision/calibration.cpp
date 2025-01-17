
#include "calibration.h"
using namespace cv;
using namespace std;

static void help()
{
	cout << "This is a camera calibration sample." << endl
		<< "Usage: camera_calibration [configuration_file -- default ./default.xml]" << endl
		<< "Near the sample file you'll find the configuration file, which has detailed help of "
		"how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
void see_help()
{
	help();
}




Settings::Settings() : goodInput(false) {};
//enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
//enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

void Settings::write(FileStorage& fs) const                        //Write serialization for this class
{
	fs << "{"
		<< "BoardSize_Width" << boardSize.width
		<< "BoardSize_Height" << boardSize.height
		<< "Square_Size" << squareSize
		<< "Calibrate_Pattern" << patternToUse
		<< "Calibrate_NrOfFrameToUse" << nrFrames
		<< "Calibrate_FixAspectRatio" << aspectRatio
		<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
		<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

		<< "Write_DetectedFeaturePoints" << writePoints
		<< "Write_extrinsicParameters" << writeExtrinsics
		<< "Write_outputFileName" << outputFileName
		<< "Show_UndistortedImage" << showUndistorsed
		<< "Input_FlipAroundHorizontalAxis" << flipVertical
		<< "Input_Delay" << delay
		<< "Input" << input
		<< "}";
}
void Settings::read(const FileNode& node)                          //Read serialization for this class
{
	node["BoardSize_Width"] >> boardSize.width;
	node["BoardSize_Height"] >> boardSize.height;
	node["Calibrate_Pattern"] >> patternToUse;
	node["Square_Size"] >> squareSize;
	node["Calibrate_NrOfFrameToUse"] >> nrFrames;
	node["Calibrate_FixAspectRatio"] >> aspectRatio;
	node["Write_DetectedFeaturePoints"] >> writePoints;
	node["Write_extrinsicParameters"] >> writeExtrinsics;
	node["Write_outputFileName"] >> outputFileName;
	node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
	node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
	node["Calibrate_UseFisheyeModel"] >> useFisheye;
	node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
	node["Show_UndistortedImage"] >> showUndistorsed;
	node["Input"] >> input;
	node["Input_Delay"] >> delay;
	node["Fix_K1"] >> fixK1;
	node["Fix_K2"] >> fixK2;
	node["Fix_K3"] >> fixK3;
	node["Fix_K4"] >> fixK4;
	node["Fix_K5"] >> fixK5;
	node["alpha"] >> alpha;
	validate();
}
void Settings::validate()
{
	goodInput = true;
	if (boardSize.width <= 0 || boardSize.height <= 0)
	{
		cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
		goodInput = false;
	}
	if (squareSize <= 10e-6)
	{
		cerr << "Invalid square size " << squareSize << endl;
		goodInput = false;
	}
	if (nrFrames <= 0)
	{
		cerr << "Invalid number of frames " << nrFrames << endl;
		goodInput = false;
	}

	if (input.empty())      // Check for valid input
		inputType = INVALID;
	else
	{
		if (input[0] >= '0' && input[0] <= '9')
		{
			stringstream ss(input);
			ss >> cameraID;
			inputType = CAMERA;
		}
		else
		{
			if (isListOfImages(input) && readStringList(input, imageList))
			{
				inputType = IMAGE_LIST;
				nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
			}
			else
				inputType = VIDEO_FILE;
		}
		if (inputType == CAMERA)
			inputCapture.open(cameraID);
		if (inputType == VIDEO_FILE)
			inputCapture.open(input);
		if (inputType != IMAGE_LIST && !inputCapture.isOpened())
			inputType = INVALID;
	}
	if (inputType == INVALID)
	{
		cerr << " Input does not exist: " << input;
		goodInput = false;
	}

	flag = 0;
	if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
	if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
	if (aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
	if (fixK1)                  flag |= CALIB_FIX_K1;
	if (fixK2)                  flag |= CALIB_FIX_K2;
	if (fixK3)                  flag |= CALIB_FIX_K3;
	if (fixK4)                  flag |= CALIB_FIX_K4;
	if (fixK5)                  flag |= CALIB_FIX_K5;

	if (useFisheye) {
		// the fisheye model has its own enum, so overwrite the flags
		flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
		if (fixK1)                   flag |= fisheye::CALIB_FIX_K1;
		if (fixK2)                   flag |= fisheye::CALIB_FIX_K2;
		if (fixK3)                   flag |= fisheye::CALIB_FIX_K3;
		if (fixK4)                   flag |= fisheye::CALIB_FIX_K4;
		if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
	}

	calibrationPattern = NOT_EXISTING;
	if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
	if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
	if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
	if (calibrationPattern == NOT_EXISTING)
	{
		cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
		goodInput = false;
	}
	atImageList = 0;

}
Mat Settings::nextImage()
{
	Mat result;
	if (inputCapture.isOpened())
	{
		Mat view0;
		inputCapture >> view0;
		view0.copyTo(result);
	}
	else if (atImageList < imageList.size())
		result = imread(imageList[atImageList++], IMREAD_COLOR);

	return result;
}

bool Settings::readStringList(const string& filename, vector<string>& l)   //static
{
	l.clear();
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}

bool Settings::isListOfImages(const string& filename)   //static 
{
	string s(filename);  //构造函数
						 // Look for file extension
	if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos)
		return false;
	else
		return true;
}



//enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };  //如果打算只使用常量而不创建枚举类型的变量，可以省略美剧类型的名称
// detection是指使用camera获取图像， capturing是image list


//! [compute_errors]
static double computeReprojectionErrors(const vector<vector<Point3d> >& objectPoints,
	const vector<vector<Point2d> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors, bool fisheye)
{
	vector<Point2d> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}


//! [compute_errors]
//! [board_corners]
//此函数计算所有corners的三维世界坐标
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3d>& corners,
	Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
	corners.clear();

	switch (patternType)
	{
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(Point3d(j*squareSize, i*squareSize, 0));
		break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3d((2 * j + i % 2)*squareSize, i*squareSize, 0));
		break;
	default:
		break;
	}
}


//! [board_corners]
static bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2d> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr)
{
	//! [fixed_aspect]
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (s.flag & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = s.aspectRatio;
	//! [fixed_aspect]
	if (s.useFisheye) {
		distCoeffs = Mat::zeros(4, 1, CV_64F);
	}
	else {
		distCoeffs = Mat::zeros(8, 1, CV_64F);
	}

	vector<vector<Point3d> > objectPoints(1);
	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms;

	if (s.useFisheye) {
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
			_tvecs, s.flag);

		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);
		for (int i = 0; i < int(objectPoints.size()); i++) {
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));
		}
	}
	else {
		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
			s.flag);
	}

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		distCoeffs, reprojErrs, s.useFisheye);

	return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs, const vector<vector<Point2d> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(s.outputFileName, FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf), "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << s.boardSize.width;
	fs << "board_height" << s.boardSize.height;
	fs << "square_size" << s.squareSize;

	if (s.flag & CALIB_FIX_ASPECT_RATIO)
		fs << "fix_aspect_ratio" << s.aspectRatio;

	if (s.flag)
	{
		std::stringstream flagsStringStream;
		if (s.useFisheye)
		{
			flagsStringStream << "flags:"
				<< (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
				<< (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
		}
		else
		{
			flagsStringStream << "flags:"
				<< (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
				<< (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
				<< (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
				<< (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
				<< (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
		}
		fs.writeComment(flagsStringStream.str());
	}

	fs << "flags" << s.flag;

	fs << "fisheye_model" << s.useFisheye;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (s.writeExtrinsics && !reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
		bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
		bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

		for (size_t i = 0; i < rvecs.size(); i++)
		{
			Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
			Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

			if (needReshapeR)
				rvecs[i].reshape(1, 1).copyTo(r);
			else
			{
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				r = rvecs[i].t();
			}

			if (needReshapeT)
				tvecs[i].reshape(1, 1).copyTo(t);
			else
			{
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				t = tvecs[i].t();
			}
		}
		fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
		fs << "extrinsic_parameters" << bigmat;
	}

	if (s.writePoints && !imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (size_t i = 0; i < imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		//fs << "image_points" << imagePtMat;
		fs << "image_points" << imagePoints;
	}
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s,  Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2d> > imagePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
		totalAvgErr);
	cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr << endl;

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
			totalAvgErr);
	return ok;
}