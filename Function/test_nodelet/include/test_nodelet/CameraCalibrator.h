#pragma once
//opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>  
#include<opencv/cv.h>
#include <sstream>
#include <iostream>

//std

class CameraCalibrator
{
public:
	CameraCalibrator(void);
	CameraCalibrator(cv::Size);
	virtual ~CameraCalibrator(void);
	// input points: 
	// the points in world coordinates 
	std::vector< std::vector<cv::Point3f> > objectPoints; 

	// the point positions in pixels 
	std::vector< std::vector<cv::Point2f> > imagePoints; 

    //chessboard pictures, which may be similar with each other, so you need to analyse them to find out the pics should be cali
   //pics index is same as the imagePoints
	std::vector<cv::Mat> pics;

	// output Matrices 
	cv::Mat A;//cameraMatrix; (f,u0,v0,dx,dy)
	cv::Mat D;//distCoeffs; (k1-k3,p1-p2)
	std::vector<cv::Mat> rvecs, tvecs; 
	// flag to specify how calibration is done 
	int flag; 

	// used in image undistortion  
	cv::Mat map1,map2;  
	bool mustInitUndistort; 
	cv::Size  boardSize_;



public:
	// Open chessboard images and extract corner points 
	int addChessboardPoints(const std::vector<std::string>& filelist);
	//Get chessboard images from camera and check corner points 
	bool addChessboardPoints(const cv::Mat &image);
	// Add scene points and corresponding image points 
	void addPoints(const std::vector<cv::Point2f>&imageCorners, const std::vector<cv::Point3f>& objectCorners);
	//检查是否达到标定的条件
	bool checkPrepareForCalibrate();
	// Calibrate the camera 
	// 如果标定精度在设定的范围内，则认为成功
    bool calibrate();
	bool calibrate(cv::Size &imageSize);
	// remove distortion in an image (after calibration) 
	cv::Mat remap(const cv::Mat &image);
	void reprojectFromImageToObject();
	void reprojectFromImageToObject(cv::Mat Image);
	cv::Point3f  getDistanceFromObjToImag(int picNum, int cornerNum);
	//mode--> 0:using  default R(使用默认的旋转矩阵，认为相机坐标系和世界坐标系平行，Z轴距离未知
    bool checkCircle(cv::Mat Image);
	//通过获取图像，图像上有已知具体尺寸的直线，通过识别其长度（像素值），计算出像素的尺寸
	int  getPixelLength(const cv::Mat &image);

};

cv::Mat mergeByCols(cv::Mat &a, cv::Mat &b);
cv::Mat reprojectFromImageToObject2TO3(cv::Point2f imagP, cv::Mat cameraMatrix, cv::Mat disMatrix,
									   cv::Mat RodMatrix, cv::Mat TMatrix, cv::Mat *RandTMaxtix=NULL, int flags=0);
