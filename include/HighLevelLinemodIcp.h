#pragma once

#include <opencv2/core.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/rgbd.hpp>

//#include <glm/glm.hpp>
//#include <glm/gtx/quaternion.hpp>

#include "defines.h"
#include "utility.h"
#include "OpenglRender.h"

class HighLevelLinemodIcp
{
public:
	HighLevelLinemodIcp(uint16_t in_iteration, float in_tolerance, float in_rejectionScale, uint16_t in_numIterations,
		uint16_t in_sampleStep, std::vector<std::string> in_modelFiles,
		std::string in_modFolder);
	~HighLevelLinemodIcp();

	void prepareDepthForIcp(cv::Mat& in_depth, const cv::Mat& in_camMatrix, cv::Rect& bb);
	void registerToScene(std::vector<ObjectPose>& in_poses, uint16_t in_modelNumber);
	uint16_t estimateBestMatch(cv::Mat in_depthImg, std::vector<ObjectPose> in_poses, OpenGLRender* in_openglRend,
		uint16_t in_modelIndice, uint16_t& in_bestPose);
private:
	uint16_t correctEstimateTreshold = 20;

	cv::ppf_match_3d::ICP* icp;
	std::vector<cv::Mat> modelVertices;
	std::vector<std::string> modelFiles;
	std::string modelFolder;
	cv::Mat sceneVertices;
	uint16_t sampleStep;
	std::vector<cv::ppf_match_3d::Pose3DPtr> poses;
	void remove_if(const cv::Mat& in_mat, cv::Mat& in_res, bool in_removeRows = true);
	void depthToBinary(cv::Mat& in_gray, cv::Mat& in_binary, uint32_t in_threshold = 1);
	void loadModels();
};
