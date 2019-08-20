#pragma once

#include <opencv2/core.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iterator>

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
	void cleanup();
	void run(std::vector<cv::Mat>& in_imgs, std::string const& in_className, uint16_t const& in_numberOfObjects, std::vector<ObjectPose>& in_objPose);
	void setupBenchmark(std::string const& in_className);
private:
	OpenGLRender* opengl;
	HighLevelLineMOD* line;
	HighLevelLinemodIcp* icp;
	Benchmark* bench;

	std::vector<cv::Mat> inputImg;
	std::vector<std::vector<ObjectPose>> detectedPoses;
	std::vector<ObjectPose> finalObjectPoses;
	cv::Mat colorImg;
	cv::Mat depthImg;

	CameraParameters camParams;
	TemplateGenerationSettings templateSettings;
	std::vector<std::string> modelFiles;
	std::vector<cv::String> ids;

	void readLinemodFromFile();
	uint16_t findIndexInVector(std::string const& in_stringToFind, std::vector<std::string>& in_vectorToLookIn);

	void drawCoordinateSystem(cv::Mat& in_srcDstImage, const cv::Mat& in_camMat, float in_coordinateSystemLength,
		ObjectPose& in_objPos);
	cv::Mat translateImg(cv::Mat &in_img, int in_offsetx, int in_offsety);
};
