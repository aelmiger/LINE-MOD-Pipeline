#include "Template_Generator.h"



Template_Generator::Template_Generator(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings) :
	startDistance(in_templateSettings.startDistance),
	endDistance(in_templateSettings.endDistance),
	stepSize(in_templateSettings.stepSize),
	subdivisions(in_templateSettings.subdivisions),
	modelFolder(in_templateSettings.modelFolder)
{
	opengl = new OpenGLRender(in_camParams);
	line = new HighLevelLinemod(true, in_camParams, in_templateSettings);
	filesInDirectory(modelFiles, modelFolder, in_templateSettings.modelFileEnding);

}


Template_Generator::~Template_Generator()
{
	delete opengl;
	delete line;
}


void Template_Generator::run()
{
	for (size_t i = 0; i < modelFiles.size(); i++)
	{
		opengl->creatModBuffFromFiles(modelFolder + modelFiles[i]);
		for (float32 radiusToModel = startDistance; radiusToModel <= endDistance; radiusToModel += stepSize)
		{
			createCamViewPoints(radiusToModel);
			for (size_t j = 0; j < numCameraVertices; j++)
			{
				printProgBar(calculateCurrentPercent(radiusToModel, j), modelFiles[i]);

				//if (!(camVertices[j].y >= 0)) { //TODO NON HARDCODE
				//	continue;
				//}
				std::vector<cv::Mat> images;
				renderImages(images, i, j);
				line->addTemplate(images, modelFiles[i], camVertices[j]);
			}

		}
		line->pushBackTemplates();
	}
	line->writeLinemod();

}


void Template_Generator::createCamViewPoints(float32 in_radiusToModel) {
	CameraViewPoints camPoints(in_radiusToModel);
	//CameraViewPoints camPoints(in_radiusToModel, subdivisions);
	camVertices = camPoints.getVertices();
	numCameraVertices = camVertices.size();
}

void Template_Generator::renderImages(std::vector<cv::Mat>& in_imgVec, uint16 in_modelIterator, uint16 in_vertIterator) {
	in_imgVec.clear();
	opengl->renderDepthToFrontBuff(in_modelIterator, glm::vec3(camVertices[in_vertIterator]));
	cv::Mat depth = opengl->getDepthImgFromBuff();
	opengl->renderColorToFrontBuff(in_modelIterator, glm::vec3(camVertices[in_vertIterator]));
	cv::Mat color = opengl->getColorImgFromBuff();
	std::vector<cv::Mat> images;
	in_imgVec.push_back(color);
	in_imgVec.push_back(depth);
}

void Template_Generator::printProgBar(uint16 in_percent, std::string in_mfile) {
	std::string bar;

	for (int i = 0; i < 50; i++) {
		if (i < (in_percent / 2)) {
			bar.replace(i, 1, "=");
		}
		else if (i == (in_percent / 2)) {
			bar.replace(i, 1, ">");
		}
		else {
			bar.replace(i, 1, " ");
		}
	}
	std::cout << "\r" "[" << bar << "] ";
	std::cout.width(3);
	std::cout << in_percent << "%     " + in_mfile << std::flush;
	if (in_percent == 100) {
		std::cout << std::endl;
	}

}

uint16 Template_Generator::calculateCurrentPercent(uint16 const& in_spehreRadius, uint16 const& in_currentIteration) {
	uint16 numberOfDiffRadius = std::floor((endDistance - startDistance + stepSize) / stepSize);
	return((((in_currentIteration + 1) * 100) / numCameraVertices) / numberOfDiffRadius + (in_spehreRadius - startDistance) / stepSize * 100 / numberOfDiffRadius);
}

