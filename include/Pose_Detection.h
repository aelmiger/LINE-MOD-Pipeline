#pragma once

#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/videoio.hpp>
//#include <glm/gtx/string_cast.hpp>

#include "Opengl_Render.h"
#include "High_Level_Linemod.h"
#include "High_Level_Linemod_Icp.h"
#include "Kinect2.h"

class Pose_Detection
{
public:
	Pose_Detection(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings);
	~Pose_Detection();
	void run();
private:
	OpenGLRender* opengl;
	HighLevelLinemod* line;
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
