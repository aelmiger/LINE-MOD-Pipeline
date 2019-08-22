#pragma once
#include <opencv2/core.hpp>

#include "OpenglRender.h"

/**
 * @brief Benchmark class with functions to evaluate the estimated pose of different datasets
 * @detail Benchmark from Arcuo class should be in "benchmark" folder and linemod benchmark in "benchmarkLINEMOD" folder
 */
class Benchmark
{
public:
	/**
	 * @brief Construct a new Benchmark object
	 * 
	 */
	Benchmark();
	~Benchmark();

	/**
	 * @brief Increases the image counter and prints the current score
	 * @detail Has to be called every time the detector is run on an image to get stats
	 * 
	 */
	void increaseImgCounter();

	/**
	 * @brief Calculate the error with the BOP 6D benchmark evalutation method by Hodan et al
	 * 
	 * @param in_depthImg Input depth image
	 * @param in_opengl Pointer to opengl instance
	 * @param in_estimatePose The estimated Pose to compare
	 * @param in_modelIndice 
	 * @return float error
	 */
	float calculateErrorHodan(cv::Mat& in_depthImg, OpenGLRender* in_opengl,
	                          ObjectPose const& in_estimatePose, uint16_t const& in_modelIndice);

	/**
	 * @brief Calculate the error with the linemod evalutation method
	 * 
	 * @param in_estimate Estimated pose
	 * @return float error
	 */
	float calculateErrorLM(ObjectPose& in_estimate);

	/**
	 * @brief Calculate the error with the linemod evalutation method for ambigous objects
	 * 
	 * @param in_estimate Estimated pose
	 * @return float error
	 */
	float calculateErrorLMAmbigous(ObjectPose& in_estimate);

	/**
	 * @brief Loading the model comparison
	 * 
	 * @param in_opengl Pointer to opengl instance
	 * @param in_modelLocation Name of the model
	 */
	void loadModel(OpenGLRender* in_opengl, std::string in_modelLocation);

private:
	/**
	 * @brief How many times was detector run
	 * 
	 */
	int32_t imageCounter = 0;

	/**
	 * @brief How many times was pose correct with hodan evalutation method
	 * 
	 */
	int32_t hodanCounter = 0;

	/**
	 * @brief How many times was pose correct with linemod evalutation method
	 * 
	 */
	int32_t lineCounter = 0;

	/**
	 * @brief Reducing the number of model vertices for faster linemod evalutaion
	 * 
	 */
	uint32_t subsampleStep = 40;

	/**
	 * @brief Taken from the bop 6d benchmark paper. Do not change 
	 * 
	 */
	int32_t visibilityThreshold = 15;

	/**
	 * @brief Taken from the bop 6d benchmark paper. Do not change 
	 * 
	 */
	int32_t errorThreshold = 20;

	/**
	 * @brief Diameter of object in cm. Relevant for Linemod evaluation method.
	 * @detail Change for each object to get a correct score
	 * 
	 */
	float objectDiameter = 21;

	cv::Mat inputDepth;
	cv::Mat groundTruthDepthRender;
	cv::Mat estimateDepthRender;
	cv::Mat renderedAbsDiff;

	cv::Mat groundTruthVisibility;
	cv::Mat estimateVisibility;
	cv::Mat visibilityIntersection;
	cv::Mat visibilityCombination;
	cv::Mat absDiffMaskApplied;

	Model model;
	Model subsampledModel;

	ObjectPose groundTruth;

	/**
	 * @brief A mask describing the object occlusion
	 * 
	 */
	void calculateVisibilityMasks();

	/**
	 * @brief Render the depth image at given pose
	 * 
	 * @param in_opengl Pointer to opengl instance
	 * @param in_pose Pose to render object under
	 * @param in_modelIndice Index to model name in model name vector
	 * @return cv::Mat Rendered depth image
	 */
	cv::Mat renderPose(OpenGLRender* in_opengl, ObjectPose const& in_pose,
	                   uint16_t const& in_modelIndice);

	/**
	 * @brief Converting to proper coordinate system
	 * 
	 * @param in_pose Pose to convert in opengl coordinate system
	 * @return glm::mat4 View mat
	 */
	glm::mat4 calculateViewMat(ObjectPose const& in_pose);

	/**
	 * @brief Reduce the amount of model vertices
	 * 
	 */
	void subsamplingModel();

	/**
	 * @brief Read the ground truth pose in "benchmark" folder
	 * 
	 */
	void readGroundTruthPose();

	/**
	 * @brief Read linemod ground truht folder in benchmarkLINEMOD folder
	 * 
	 */
	void readGroundTruthLinemodDataset();

	/**
	 * @brief Print the percentage of correct estimated poses with hodan
	 * 
	 */
	void printHodanScore();

	/**
	 * @brief Print the percentage of correct estimated poses with linemod
	 * 
	 */
	void printLMScore();

	/**
	 * @brief Calculate length of cv vec3
	 * 
	 * @param in_vecA 
	 * @return float 
	 */
	float length(cv::Vec3f& in_vecA);
};
