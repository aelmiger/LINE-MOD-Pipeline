#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <glm/gtx/string_cast.hpp>

#include "opengl_render.h"
#include "high_level_linemod.h"
#include "high_level_linemod_icp.h"
#include "kinect2.h"


class Pose_Detection
{
public:
	Pose_Detection(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings);
	~Pose_Detection();
	void run();
private:
	OpenGLRender* opengl;
	lineMOD::HighLevelLinemod* line;
	lineMODIcp::HighLevelLinemodIcp* icp;

	std::vector<cv::Mat> inputImg;
	std::vector<ObjectPose> detectedPoses;
	cv::Mat colorImg;
	cv::Mat depthImg;

	std::vector<std::string> modelFiles;
	std::string modelFolder;
	std::vector<cv::String> ids;
	cv::Mat cameraMatrix;
	float32 bestMean;

	void readLinemodFromFile();
};

