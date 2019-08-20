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

/**
 * @brief The class deals with the ICP pose refinement and picks the best pose out of multiple refined poses
 * 
 */
class HighLevelLinemodIcp
{
public:
/**
 * @brief Construct a new High Level Linemod Icp object
 * 
 * @param in_iteration The maximum amount of iterations the algorithm performs
 * @param in_tolerance The sought after matching score. Changing this is not advised
 * @param in_rejectionScale Rejection Scale of outlier points. Also not advised to change it
 * @param in_numIterations A quality setting for the icp algorithm. Does not seem to have any effect on the outcome.
 * @param in_sampleStep Factor that reduces the amont of points used for ICP. 
 * @param in_modelFiles Vector of model file names. It is equal to the class names of the detector
 * @param in_modFolder Location of the model files
 */
	HighLevelLinemodIcp(uint16_t in_iteration, float in_tolerance, float in_rejectionScale, uint16_t in_numIterations,
		uint16_t in_sampleStep, std::vector<std::string> in_modelFiles,
		std::string in_modFolder);
	~HighLevelLinemodIcp();

	/**
	 * @brief Setting ICP with the input depth image
	 * 
	 * @param in_depth Input depth image
	 * @param in_camMatrix Camera Matrix to convert depth image into 3D points
	 * @param bb Bounding box of a match
	 */
	void prepareDepthForIcp(cv::Mat& in_depth, const cv::Mat& in_camMatrix, cv::Rect& bb);

	/**
	 * @brief Refining poses with the ICP algorithm
	 * 
	 * @param[in/out] in_poses Poses are changed by ICP 
	 * @param in_modelNumber Index of the model from model name vector
	 */
	void registerToScene(std::vector<ObjectPose>& in_poses, uint16_t in_modelNumber);

	/**
	 * @brief Estimate which of the refined poses is the best fit
	 * 
	 * @param in_depthImg Input depth image
	 * @param in_poses Poses to consider in comparison
	 * @param in_openglRend Pointer to the Opengl instance
	 * @param in_modelIndice Index of the model from model name vector
	 * @param[out] in_bestPose Index of the best pose from poses vector
	 * @return bool Returns if a pose was found that fits to the depth image
	 */
	bool estimateBestMatch(cv::Mat in_depthImg, std::vector<ObjectPose> in_poses, OpenGLRender* in_openglRend,
		uint16_t in_modelIndice, uint16_t& in_bestPose);
private:
	/**
	 * @brief Sets the threshold for the mean difference between the rendered pose and the depth image
	 * @detail Lowering the value means that the pose needs to fit better to the depth image
	 * 
	 */
	uint16_t correctEstimateTreshold = 35;


	cv::ppf_match_3d::ICP* icp;
	std::vector<cv::Mat> modelVertices;
	std::vector<std::string> modelFiles;
	std::string modelFolder;
	cv::Mat sceneVertices;
	uint16_t sampleStep;
	std::vector<cv::ppf_match_3d::Pose3DPtr> poses;
	void removeIfTooFarFromMean(const cv::Mat& in_mat, cv::Mat& in_res, bool in_removeRows = true);
	void depthToBinary(cv::Mat& in_gray, cv::Mat& in_binary, uint32_t in_threshold = 1);
	void loadModels();
	void updatePosition(cv::Matx44d in_mat, ObjectPose& in_objPose);

};
