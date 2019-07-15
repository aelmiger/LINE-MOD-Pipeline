#pragma once

#include <opencv2/core.hpp>

#include "OpenglRender.h"
#include "HighLevelLinemod.h"
#include "HighLevelLinemodIcp.h"
#include "Kinect2.h"
#include "utility.h"
#include "Benchmark.h"

class PoseDetection
{
public:
	PoseDetection();
	~PoseDetection();
	void run();
private:
	OpenGLRender* opengl;
	HighLevelLineMOD* line;
	HighLevelLinemodIcp* icp;

	std::vector<cv::Mat> inputImg;
	std::vector<std::vector<ObjectPose>> detectedPoses;
	std::vector<ObjectPose> finalObjectPoses;
	cv::Mat colorImg;
	cv::Mat depthImg;

	std::vector<std::string> modelFiles;
	std::string modelFolder;
	std::vector<cv::String> ids;
	cv::Mat cameraMatrix;

	void readLinemodFromFile();
};
