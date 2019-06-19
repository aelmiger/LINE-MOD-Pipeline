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
	HighLevelLinemodIcp(uint16 in_iteration, float32 in_tolerance, float32 in_rejectionScale, uint16 in_numIterations, uint16 in_sampleStep);
	~HighLevelLinemodIcp();

	void loadModel(std::string in_model);
	void prepareDepthForIcp(cv::Mat& in_depth, const cv::Mat& in_camMatrix, cv::Rect& bb);
	void registerToScene(std::vector<ObjectPose>& in_poses);
	uint16 estimateBestMatch(cv::Mat in_depthImg, std::vector<ObjectPose> in_poses, OpenGLRender* in_openglRend, uint16 in_modelIndice);
private:
	cv::ppf_match_3d::ICP* icp;
	cv::Mat modelVertices;
	cv::Mat sceneVertices;
	uint16 sampleStep;
	std::vector< cv::ppf_match_3d::Pose3DPtr> poses;
	typedef bool(*remove_predicate)(const cv::Mat &in_rc);
	void remove_if(const cv::Mat &in_mat, cv::Mat &in_res, remove_predicate in_pred, bool in_removeRows = true);
	bool is_zero(const cv::Mat &in_rc);
	void depthToBinary(cv::Mat &in_gray, cv::Mat &in_binary, uint32 in_threshold = 1);
};

