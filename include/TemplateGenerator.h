#pragma once
#include <vector>
#include <iostream>

#include "defines.h"
#include "utility.h"
#include "HighLevelLinemod.h"
#include "OpenglRender.h"
#include "CameraViewPoints.h"

/**
 * @brief High level class covering the template generation
 * 
 */
class TemplateGenerator
{
public:
	/**
	 * @brief Construct a new Template Generator object
	 * 
	 */
	TemplateGenerator();
	~TemplateGenerator();

	/**
	 * @brief Cleaning up function
	 * 
	 */
	void cleanup();

	/**
	 * @brief Running the template generation and saving it to a file afterwards
	 * 
	 */
	void run();

private:
	OpenGLRender* opengl;
	HighLevelLineMOD* line;
	CameraViewPoints* camPoints;
	std::vector<glm::vec3> camVertices;

	std::vector<std::string> modelFiles;
	std::string modelFolder;
	uint16_t startDistance;
	uint16_t endDistance;
	uint16_t stepSize;
	uint16_t subdivisions;

	uint32_t numCameraVertices;

	/**
	 * @brief Create the viewpoints of the camera
	 * 
	 * @param in_radiusToModel Distance to model in mm
	 */
	void createCamViewPoints(float in_radiusToModel);

	/**
	 * @brief Render the color and depth image
	 * 
	 * @param[out] in_imgVec output image vector
	 * @param in_modelIterator Index of model name
	 * @param in_vertIterator Index of camera position from vector
	 */
	void renderImages(std::vector<cv::Mat>& in_imgVec, uint16_t in_modelIterator,
	                  uint16_t in_vertIterator);

	/**
	 * @brief Utility function to print a progress bar
	 * 
	 * @param in_percent Percent progress
	 * @param in_mfile Name of model
	 */
	void printProgBar(uint16_t in_percent, std::string const& in_mfile);

	/**
	 * @brief Calculate progress of model in percent
	 * 
	 * @param in_spehreRadius 
	 * @param in_currentIteration 
	 * @return uint16_t 
	 */
	uint16_t calculateCurrentPercent(uint16_t const& in_spehreRadius,
	                                 uint16_t const& in_currentIteration);

	/**
	 * @brief Create a yaml settings file
	 * 
	 */
	void writeSettings();
};
