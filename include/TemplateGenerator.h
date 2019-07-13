#pragma once
#include <vector>
#include <iostream>

#include "defines.h"
//#include "model_buffer.h"
#include "utility.h"
#include "HighLevelLinemod.h"
#include "OpenglRender.h"
#include "CameraViewPoints.h"

class TemplateGenerator
{
public:
	TemplateGenerator();
	~TemplateGenerator();
	void run();

private:
	OpenGLRender* opengl;
	HighLevelLineMOD* line;
	std::vector<glm::vec3> camVertices;

	std::vector<std::string> modelFiles;
	std::string modelFolder;
	uint16_t startDistance;
	uint16_t endDistance;
	uint16_t stepSize;
	uint16_t subdivisions;

	uint32_t numCameraVertices;

	void createCamViewPoints(float in_radiusToModel);
	void renderImages(std::vector<cv::Mat>& in_imgVec, uint16_t in_modelIterator, uint16_t in_vertIterator);

	void printProgBar(uint16_t in_percent, std::string const& in_mfile);
	uint16_t calculateCurrentPercent(uint16_t const& in_spehreRadius, uint16_t const& in_currentIteration);
	void writeSettings();
};
