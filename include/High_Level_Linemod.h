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

class HighLevelLinemod
{
public:
	HighLevelLinemod(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings);
	~HighLevelLinemod();

	std::vector<cv::String> getClassIds();
	uint16 getNumClasses();
	uint32 getNumTemplates();

	bool addTemplate(std::vector<cv::Mat> in_images, const std::string& in_modelName, glm::vec3 in_cameraPosition);
	bool detectTemplate(std::vector<cv::Mat>& in_imgs, uint16 in_classNumber);
	void writeLinemod();
	void readLinemod();
	void pushBackTemplates();

	std::vector<std::vector<ObjectPose>> getObjectPoses();

private:
	const uint16 percentToPassCheck = 60;
	const uint16 numberWantedPoses = 3;

	cv::Ptr<cv::linemod::Detector> detector;

	bool onlyColor;

	uint16 videoWidth;
	uint16 videoHeight;
	float32 cx;
	float32 cy;
	float32 fx;
	float32 fy;
	float32 fieldOfViewHeight;

	std::vector<cv::Mat> inPlaneRotationMat;
	uint16 lowerAngleStop;
	uint16 upperAngleStop;
	uint16 angleStep;
	uint16 stepSize;

	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
	int32 tempDepth;

	struct ColorRangeOfObject
	{
		ColorRangeOfObject()
		{
		}

		ColorRangeOfObject(cv::Scalar s1, cv::Scalar s2) :
			lowerBoundary(std::move(s1)),
			upperBoundary(std::move(s2))
		{
		}

		cv::Scalar lowerBoundary;
		cv::Scalar upperBoundary;
	};

	struct Template
	{
		Template()
		{
		}

		Template(glm::vec3 tra, glm::qua<float32> qua, cv::Rect bb, uint16 med) :
			translation(tra),
			quaternions(qua),
			boundingBox(std::move(bb)),
			medianDepth(med)
		{
		}

		glm::vec3 translation;
		glm::qua<float32> quaternions;
		cv::Rect boundingBox;
		uint16 medianDepth;
	};

	struct PotentialMatch
	{
		PotentialMatch(cv::Point in_point, size_t in_indices) :
			position(std::move(in_point))
		{
			matchIndices.push_back(in_indices);
		}

		cv::Point position;
		std::vector<uint32> matchIndices;
	};

	cv::Mat colorImgHue;
	std::vector<Template> templates;
	ColorRangeOfObject currentColorRange;
	std::vector<std::vector<Template>> modelTemplates;
	std::vector<ColorRangeOfObject> modelColors;
	std::vector<std::vector<ObjectPose>> posesMultipleObj;
	std::vector<cv::linemod::Match> matches;
	std::vector<cv::linemod::Match> groupedMatches;
	std::vector<PotentialMatch> potentialMatches;

	std::vector<std::string> modelFiles;
	std::string modelFolder;

	//Utility Functions
	void generateRotMatForInplaneRotation();
	uint16 medianMat(cv::Mat const& in_mat, cv::Rect& in_bb, uint8 in_medianPosition);
	void calculateTemplatePose(glm::vec3& in_translation, glm::qua<float32>& in_quats, glm::vec3& in_cameraPosition,
	                           float32& in_inplaneRot);
	glm::qua<float32> openglCoordinatesystem2opencv(glm::mat4& in_viewMat);
	bool applyPostProcessing(std::vector<cv::Mat>& in_imgs, std::vector<ObjectPose>& in_objPoses);
	bool colorCheck(cv::Mat& in_hueImg, uint32& in_numMatch, float32 in_percentCorrectColor);
	bool depthCheck(cv::Mat& in_depth, uint32& in_numMatch);
	void updateTranslationAndCreateObjectPose(uint32 const& in_numMatch, std::vector<ObjectPose>& in_objPoses);
	void calcPosition(uint32 const& in_numMatch, glm::vec3& in_position, float32 const& in_directDepth);
	void calcRotation(uint32 const& in_numMatch, glm::vec3 const& in_position, glm::qua<float32>& in_quats);
	void matchToPixelCoord(uint32 const& in_numMatch, float32& in_x, float32& in_y);
	float32 pixelDistToCenter(float32 in_x, float32 in_y);
	float32 calcTrueZ(float32 const& in_directDist, float32 const& in_angleFromCenter);
	void templateMask(cv::linemod::Match const& in_match, cv::Mat& dst);
	void groupSimilarMatches();
	void discardSmallMatchGroups();
	std::vector<cv::linemod::Match> elementsFromListOfIndices(std::vector<cv::linemod::Match>& in_matches,
	                                                          const std::vector<uint32>& in_indices);
	void readColorRanges();
};
