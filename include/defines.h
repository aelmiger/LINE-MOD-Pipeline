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
	////CAMERA PARAMETER LINE_MOD BENCHMARK
	const float fx = 572.41140;
	const float cx = 325.26110;
	const float fy = 573.57043;
	const float cy = 242.04899;
	/////////CAMERA PARAMETER KINECT V2
	//const float32 fx = 1044.871;
	//const float32 cx = 320;
	//const float32 fy = 1045.69141;
	//const float32 cy = 240;
	cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -2.7167827743927644e-03, 2.0942424424199252e-01,
		1.1120545920170163e-03, -6.6420567497010334e-03, 0.);
	const uint16_t videoWidth = 640;
	const uint16_t videoHeight = 480;
};

struct TemplateGenerationSettings
{
	const std::string modelFileEnding = ".ply";
	const std::string modelFolder = "models/";

	const bool onlyUseColorModality = false;

	const uint16_t startDistance = 600;
	const uint16_t endDistance = 1200;
	const uint16_t stepSize = 50;
	const uint8_t subdivisions = 2;
	const uint16_t angleStart = -45;
	const uint16_t angleStop = 45;
	const uint16_t angleStep = 10;
};
