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

struct Model {
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

struct CameraParameters
{
	//////CAMERA PARAMETER LINE_MOD BENCHMARK
	//const float32 fx = 572.41140;
	//const float32 cx = 325.26110;
	//const float32 fy = 573.57043;
	//const float32 cy = 242.04899;
	///////CAMERA PARAMETER KINECT V2
	const float32 fx = 1044.871;
	const float32 cx = 320;
	const float32 fy = 1045.69141;
	const float32 cy = 240;
	cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -2.7167827743927644e-03, 2.0942424424199252e-01,
		1.1120545920170163e-03, -6.6420567497010334e-03, 0.);
	const uint16 videoWidth = 640;
	const uint16 videoHeight = 480;
};

struct TemplateGenerationSettings
{
	const std::string modelFileEnding = ".ply";
	const std::string modelFolder = "models/";

	const bool onlyUseColorModality = true;

	const float32 startDistance = 500;
	const float32 endDistance = 900;
	const float32 stepSize = 50.0f;
	const uint8 subdivisions = 2;
	const float32 angleStart = -45;
	const float32 angleStop = 45;
	const float32 angleStep = 10;
};