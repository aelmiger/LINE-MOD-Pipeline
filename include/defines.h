#pragma once

#include <cstdint>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <opencv2/core.hpp>

#define M_PI 3.141592653589793238

struct Index
{
	uint32_t a;
	uint32_t b;
	uint32_t c;
};

struct Model
{
	uint64_t numVertices;
	uint64_t numIndices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> colors;
	std::vector<uint32_t> indices;
};

struct TemplatePosition
{
	TemplatePosition();
	TemplatePosition(std::string in_s, glm::vec3 in_v, float in_rotation, cv::Rect in_boundingBox,
	                 uint16_t in_depthAtCenter);
	std::string modelName;
	glm::vec3 positionCam;
	float rotation;
	cv::Rect boundingBox;
	uint32_t depthAtCenter;
};

struct ObjectPose
{
	ObjectPose();
	ObjectPose(glm::vec3 tra, glm::qua<float> qua);
	ObjectPose(glm::vec3 tra, glm::qua<float> qua, cv::Rect bb);
	glm::vec3 translation;
	glm::qua<float> quaternions;
	cv::Rect boundingBox;
};

struct CameraParameters
{
	float fx;
	float fy;
	float cx;
	float cy;
	cv::Mat cameraMatrix;
	uint16_t videoWidth;
	uint16_t videoHeight;
	cv::Mat distortionCoefficients;
};

struct TemplateGenerationSettings
{	
	std::string modelFileEnding;
	std::string modelFolder;

	bool onlyUseColorModality;

	uint16_t startDistance;
	uint16_t endDistance;
	uint16_t stepSize;
	uint8_t subdivisions;
	int16_t angleStart;
	int16_t angleStop;
	int16_t angleStep;
};
