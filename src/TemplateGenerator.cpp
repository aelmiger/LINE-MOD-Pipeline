#include "TemplateGenerator.h"

TemplateGenerator::TemplateGenerator(CameraParameters const& in_camParameters,
                                       TemplateGenerationSettings const& in_templateSettings) :
	modelFolder(in_templateSettings.modelFolder),
	startDistance(in_templateSettings.startDistance),
	endDistance(in_templateSettings.endDistance),
	stepSize(in_templateSettings.stepSize),
	subdivisions(in_templateSettings.subdivisions)
{
	opengl = new OpenGLRender(in_camParameters);
	line = new HighLevelLineMOD(in_camParameters, in_templateSettings);
	filesInDirectory(modelFiles, modelFolder, in_templateSettings.modelFileEnding);
}

TemplateGenerator::~TemplateGenerator()
{
	if(opengl) { //TODO das in eine cleanup funktion
		delete opengl;
	}
	if (line) {
		delete line;
	}
}

void TemplateGenerator::run()
{
	for (size_t i = 0; i < modelFiles.size(); i++)
	{
		opengl->creatModBuffFromFiles(modelFolder + modelFiles[i]);
		for (uint16_t radiusToModel = startDistance; radiusToModel <= endDistance; radiusToModel += stepSize)
		{
			createCamViewPoints(radiusToModel);
			for (size_t j = 0; j < numCameraVertices; j++)
			{
				printProgBar(calculateCurrentPercent(radiusToModel, j), modelFiles[i]);

				if (!(camVertices[j].y >= 0))
				{
					//TODO NON HARDCODE
					continue;
				}
				std::vector<cv::Mat> images;
				renderImages(images, i, j);
				line->addTemplate(images, modelFiles[i], camVertices[j]);
			}
		}
		line->pushBackTemplates();
	}
	line->writeLinemod();
}

void TemplateGenerator::createCamViewPoints(float in_radiusToModel)
{
	//CameraViewPoints camPoints(in_radiusToModel);
	CameraViewPoints camPoints(in_radiusToModel, subdivisions);
	camVertices = camPoints.getVertices();
	numCameraVertices = camVertices.size();
}

void TemplateGenerator::renderImages(std::vector<cv::Mat>& in_imgVec, uint16_t in_modelIterator, uint16_t in_vertIterator)
{
	in_imgVec.clear();
	opengl->renderDepthToFrontBuff(in_modelIterator, glm::vec3(camVertices[in_vertIterator]));
	cv::Mat depth = opengl->getDepthImgFromBuff();
	opengl->renderColorToFrontBuff(in_modelIterator, glm::vec3(camVertices[in_vertIterator]));
	cv::Mat color = opengl->getColorImgFromBuff();
	std::vector<cv::Mat> images;
	in_imgVec.push_back(color);
	in_imgVec.push_back(depth);
}

void TemplateGenerator::printProgBar(uint16_t in_percent, std::string const& in_mfile)
{
	std::string bar;

	for (int i = 0; i < 50; i++)
	{
		if (i < (in_percent / 2))
		{
			bar.replace(i, 1, "=");
		}
		else if (i == (in_percent / 2))
		{
			bar.replace(i, 1, ">");
		}
		else
		{
			bar.replace(i, 1, " ");
		}
	}
	std::cout << "\r" "[" << bar << "] ";
	std::cout.width(3);
	std::cout << in_percent << "%     " + in_mfile << std::flush;
	if (in_percent == 100)
	{
		std::cout << std::endl;
	}
}

uint16_t TemplateGenerator::calculateCurrentPercent(uint16_t const& in_spehreRadius, uint16_t const& in_currentIteration)
{
	uint16_t numberOfDiffRadius = std::floor((endDistance - startDistance + stepSize) / stepSize);
	return std::round(
		(float)(in_currentIteration + 1) * 100.0f / (float)numCameraVertices / (float)numberOfDiffRadius + (float)(
			in_spehreRadius - startDistance) / (float)stepSize * 100 / (float)numberOfDiffRadius);
}
