#pragma once

#include <opencv2/core.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "defines.h"
#include "utility.h"
#include "opengl_render.h"

class HighLevelLinemodIcp
{
public:
	HighLevelLinemodIcp(uint16 in_iteration, float32 in_tolerance, float32 in_rejectionScale, uint16 in_numIterations, uint16 in_sampleStep, std::vector<std::string> in_modelFiles, std::string in_modFolder);
	~HighLevelLinemodIcp();

	void prepareDepthForIcp(cv::Mat& in_depth, const cv::Mat& in_camMatrix, cv::Rect& bb);
	void registerToScene(std::vector<ObjectPose>& in_poses, uint16 in_modelNumber);
	uint16 estimateBestMatch(cv::Mat in_depthImg, std::vector<ObjectPose> in_poses, OpenGLRender* in_openglRend, uint16 in_modelIndice);
private:
	cv::ppf_match_3d::ICP* icp;
	std::vector<cv::Mat> modelVertices;
	std::vector<std::string> modelFiles;
	std::string modelFolder;
	cv::Mat sceneVertices;
	uint16 sampleStep;
	std::vector< cv::ppf_match_3d::Pose3DPtr> poses;
	void remove_if(const cv::Mat &in_mat, cv::Mat &in_res, bool in_removeRows = true);
	void depthToBinary(cv::Mat &in_gray, cv::Mat &in_binary, uint32 in_threshold = 1);
	void loadModels();
};
