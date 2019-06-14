#pragma once

#include <cstdint>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <opencv2/core.hpp>


#define M_PI 3.141592653589793238


typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef float float32;
typedef double float64;



struct Index {
	uint32 a;
	uint32 b;
	uint32 c;
};

struct Model{
	uint64 numVertices;
	uint64 numIndices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> colors;
	std::vector<uint32> indices;
};

struct TemplatePosition {
	TemplatePosition();
	TemplatePosition(std::string in_s, glm::vec3 in_v, float32 in_rotation, cv::Rect in_boundingBox, uint16 in_depthAtCenter);
	std::string modelName;
	glm::vec3 positionCam;
	float32 rotation;
	cv::Rect boundingBox;
	uint32 depthAtCenter;
};

struct ObjectPose
{
	ObjectPose();
	ObjectPose(glm::vec3 tra, glm::qua<float32> qua);
	ObjectPose(glm::vec3 tra, glm::qua<float32> qua, cv::Rect bb);
	glm::vec3 translation;
	glm::qua<float32> quaternions;
	cv::Rect boundingBox;
};