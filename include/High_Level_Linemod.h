#pragma once

#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/imgproc.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "defines.h"
#include "utility.h"





class HighLevelLinemod
{

public:
	HighLevelLinemod(bool in_onlyColor, CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings);
	~HighLevelLinemod();

	std::vector<cv::String> getClassIds();
	uint16 getNumClasses();
	uint32 getNumTemplates();

	bool addTemplate(std::vector<cv::Mat> in_images, std::string in_modelName, glm::vec3 in_cameraPosition);
	bool detectTemplate(std::vector<cv::Mat>& in_imgs);
	void writeLinemod();
	void readLinemod();

	std::vector<ObjectPose> getObjectPoses();



private:
	const uint16 percentToPassCheck = 50;
	const uint16 numberWantedPoses = 3;

	cv::Ptr<cv::linemod::Detector> detector;

	bool onlyColor;

	uint16 videoWidth;
	uint16 videoHeight;
	float32 cx;
	float32 cy;
	float32 fieldOfView;

	std::vector<cv::Mat> inPlaneRotationMat;
	float32 lowerAngleStop;
	float32 upperAngleStop;
	float32 angleStep;
	float32 stepSize;

	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
	int32 tempDepth;


	struct Template
	{
		Template() {}
		Template(std::string str, glm::vec3 tra, glm::qua<float32> qua, cv::Rect bb, uint16 med) :
			modelName(str),
			translation(tra),
			quaternions(qua),
			boundingBox(bb),
			medianDepth(med)
		{}
		std::string modelName;
		glm::vec3 translation;
		glm::qua<float32> quaternions;
		cv::Rect boundingBox;
		uint16 medianDepth;
	};
	std::vector<Template> templates;
	std::vector<ObjectPose> objectPoses;
	std::vector<cv::linemod::Match> matches;


	//Utility Functions
	void generateRotMatForInplaneRotation();
	uint16 medianMat(cv::Mat in_mat, cv::Rect &in_bb, uint8 in_medianPosition);
	void calculateTemplatePose(glm::vec3& in_translation, glm::qua<float32>& in_quats, glm::vec3& in_cameraPosition, float32& in_inplaneRot);
	glm::qua<float32> openglCoordinatesystem2opencv(glm::mat4& in_viewMat);
	bool applyPostProcessing(std::vector<cv::Mat>& in_imgs);
	bool colorCheck(cv::Mat &in_hueImg, uint32& in_numMatch, float32 in_percentCorrectColor);
	bool depthCheck(cv::Mat &in_depth, uint32& in_numMatch);
	void updateTranslationAndCreateObjectPose(uint32 in_numMatch);
};

