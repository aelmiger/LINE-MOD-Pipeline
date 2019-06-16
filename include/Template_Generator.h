#pragma once
#include <vector>

#include "defines.h"
#include "model_buffer.h"
#include "utility.h"
#include "high_level_linemod.h"
#include "opengl_render.h"
#include "camera_view_points.h"

class Template_Generator
{
public:
	Template_Generator(CameraParameters const& in_camParameters, TemplateGenerationSettings const& in_templateSettings);
	~Template_Generator();
	void run();

private:
	OpenGLRender* opengl;
	lineMOD::HighLevelLinemod* line;
	std::vector<glm::vec3> camVertices;

	std::vector<ModelBuffer> modBuff;
	std::vector<std::string> modelFiles;
	std::string modelFolder;
	float32 startDistance;
	float32 endDistance;
	float32 stepSize;

	uint32 numCameraVertices;
	
	void creatModBuffFromFile(uint16 in_iteration);
	void createCamViewPoints(float32 in_radiusToModel);
	void renderImages(std::vector<cv::Mat>& in_imgVec, uint16 in_modelIterator, uint16 in_vertIterator);

	void printProgBar(uint16 in_percent, std::string in_mfile);
	uint16 calculateCurrentPercent(uint16 const& in_spehreRadius, uint16 const& in_currentIteration);

};

