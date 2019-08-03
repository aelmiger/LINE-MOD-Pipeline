#pragma once

#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/imgproc.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <utility>

#include "defines.h"
#include "utility.h"

class HighLevelLineMOD
{
public:
	HighLevelLineMOD(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings);
	~HighLevelLineMOD();

	std::vector<cv::String> getClassIds();

	uint16_t getNumClasses();
	uint32_t getNumTemplates();

	bool addTemplate(std::vector<cv::Mat>& in_images, const std::string& in_modelName, glm::vec3 in_cameraPosition);
	bool detectTemplate(std::vector<cv::Mat>& in_imgs, uint16_t in_classNumber);
	void writeLinemod();
	void readLinemod();
	void pushBackTemplates();

	std::vector<std::vector<ObjectPose>> getObjectPoses();

private:
	cv::Ptr<cv::linemod::Detector> detector;

	bool onlyColorModality;

	uint16_t videoWidth;
	uint16_t videoHeight;
	float cx;
	float cy;
	float fx;
	float fy;
	float fieldOfViewHeight;

	std::vector<cv::Mat> inPlaneRotationMat;
	int16_t lowerAngleStop;
	int16_t upperAngleStop;
	uint16_t angleStep;
	uint16_t stepSize;
	float detectorThreshold;
	uint16_t percentToPassCheck;
	uint16_t numberWantedPoses;
	float radiusThresholdNewObject;
	float discardGroupRatio;
	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
	int32_t tempDepth;

	struct Template
	{
		Template()
		{
		}

		Template(glm::vec3 tra, glm::qua<float> qua, cv::Rect bb, uint16_t med) :
			translation(tra),
			quaternions(qua),
			boundingBox(std::move(bb)),
			medianDepth(med)
		{
		}

		glm::vec3 translation;
		glm::qua<float> quaternions;
		cv::Rect boundingBox;
		uint16_t medianDepth;
	};

	struct PotentialMatch
	{
		PotentialMatch(cv::Point in_point, size_t in_indices) :
			position(std::move(in_point))
		{
			matchIndices.push_back(in_indices);
		}

		cv::Point position;
		std::vector<uint32_t> matchIndices;
	};

	cv::Mat colorImgHue;
	std::vector<Template> templates;
	std::vector<std::vector<Template>> modelTemplates;
	std::vector<std::vector<ObjectPose>> posesMultipleObj;
	std::vector<cv::linemod::Match> matches;
	std::vector<cv::linemod::Match> groupedMatches;
	std::vector<PotentialMatch> potentialMatches;
	std::vector<ModelProperties> modProps;

	std::vector<std::string> modelFiles;
	std::string modelFolder;

	void generateRotMatForInplaneRotation();
	uint16_t medianMat(cv::Mat const& in_mat, cv::Rect& in_bb, uint8_t in_medianPosition);
	void calculateTemplatePose(glm::vec3 & in_translation, glm::qua<float>& in_quats, glm::vec3 & in_cameraPosition, int16_t & in_inplaneRot);
	glm::qua<float> openglCoordinatesystem2opencv(glm::mat4& in_viewMat);
	bool applyPostProcessing(std::vector<cv::Mat>& in_imgs, std::vector<ObjectPose>& in_objPoses);
	bool colorCheck(cv::Mat& in_hueImg, uint32_t& in_numMatch, float in_percentCorrectColor);
	bool depthCheck(cv::Mat& in_depth, uint32_t& in_numMatch);
	void updateTranslationAndCreateObjectPose(uint32_t const& in_numMatch, std::vector<ObjectPose>& in_objPoses);
	void calcPosition(uint32_t const& in_numMatch, glm::vec3& in_position, float const& in_directDepth);
	void calcRotation(uint32_t const& in_numMatch, glm::vec3 const& in_position, glm::qua<float>& in_quats);
	void matchToPixelCoord(uint32_t const& in_numMatch, float& in_x, float& in_y);
	float pixelDistToCenter(float in_x, float in_y);
	float calcTrueZ(float const& in_directDist, float const& in_angleFromCenter);
	void templateMask(cv::linemod::Match const& in_match, cv::Mat& dst);
	void groupSimilarMatches();
	void discardSmallMatchGroups();
	std::vector<cv::linemod::Match> elementsFromListOfIndices(std::vector<cv::linemod::Match>& in_matches,
		const std::vector<uint32_t>& in_indices);
	void readColorRanges();
};
