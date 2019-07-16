#pragma once
#include <opencv2/core.hpp>

#include "OpenglRender.h"

class Benchmark
{
public:
	Benchmark();
	~Benchmark();

	void increaseImgCounter();
	float calculateErrorHodan(cv::Mat& in_depthImg, OpenGLRender* in_opengl, ObjectPose const& in_estimatePose, uint16_t const& in_modelIndice);
	float calculateErrorLM(ObjectPose& in_estimate);
	void loadModel(OpenGLRender* in_opengl, std::string in_modelLocation);
	float calculateErrorLMAmbigous(ObjectPose& in_estimate);

private:
	int32_t imageCounter=0;
	int32_t hodanCounter = 0;
	int32_t lineCounter = 0;
	uint32_t subsampleStep = 20;

	int32_t visibilityThreshold = 15;
	int32_t errorThreshold = 20;
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



	void calculateVisibilityMasks();
	cv::Mat renderPose(OpenGLRender* in_opengl, ObjectPose const& in_pose, uint16_t const& in_modelIndice);
	glm::mat4 calculateViewMat(ObjectPose const &in_pose);
	void subsamplingModel();
	void readGroundTruthLinemodDataset();
	void printHodanScore();
};

