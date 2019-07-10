#pragma once

#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/videoio.hpp>
//#include <glm/gtx/string_cast.hpp>

#include "OpenglRender.h"
#include "High_Level_Linemod.h"
#include "HighLevelLinemod_Icp.h"
#include "Kinect2.h"

class PoseDetection
{
public:
	PoseDetection(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings);
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
