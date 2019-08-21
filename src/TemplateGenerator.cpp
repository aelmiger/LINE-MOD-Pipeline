#include "TemplateGenerator.h"

TemplateGenerator::TemplateGenerator()
{
	CameraParameters camParams;
	TemplateGenerationSettings templateSettings;
	readSettings(camParams, templateSettings);
	modelFolder = templateSettings.modelFolder;
	startDistance = templateSettings.startDistance;
	endDistance = templateSettings.endDistance;
	stepSize = templateSettings.stepSize;
	subdivisions = templateSettings.subdivisions;

	opengl = new OpenGLRender(camParams);
	line = new HighLevelLineMOD(camParams, templateSettings);
	camPoints = new CameraViewPoints();
	filesInDirectory(modelFiles, modelFolder, templateSettings.modelFileEnding);
}

TemplateGenerator::~TemplateGenerator()
{
	cleanup();
}

void TemplateGenerator::cleanup()
{
	if (opengl)
	{
		delete opengl;
	}
	if (line)
	{
		delete line;
	}
	if (camPoints)
	{
		delete camPoints;
	}
}

void TemplateGenerator::run()
{
	for (size_t i = 0; i < modelFiles.size(); i++)
	{
		camPoints->readModelProperties(modelFolder + modelFiles[i]);
		opengl->creatModBuffFromFiles(modelFolder + modelFiles[i]);
		for (uint16_t radiusToModel = startDistance; radiusToModel <= endDistance; radiusToModel +=
		     stepSize)
		{
			createCamViewPoints(radiusToModel);
			for (size_t j = 0; j < numCameraVertices; j++)
			{
				printProgBar(calculateCurrentPercent(radiusToModel, j), modelFiles[i]);
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
	camPoints->createCameraViewPoints(in_radiusToModel, subdivisions);
	camVertices = camPoints->getVertices();
	numCameraVertices = camVertices.size();
}

void TemplateGenerator::renderImages(std::vector<cv::Mat>& in_imgVec, uint16_t in_modelIterator,
                                     uint16_t in_vertIterator)
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

uint16_t TemplateGenerator::calculateCurrentPercent(uint16_t const& in_spehreRadius,
                                                    uint16_t const& in_currentIteration)
{
	uint16_t numberOfDiffRadius = std::floor((endDistance - startDistance + stepSize) / stepSize);
	return std::round(
		(float)(in_currentIteration + 1) * 100.0f / (float)numCameraVertices / (float)
		numberOfDiffRadius + (float)(
			in_spehreRadius - startDistance) / (float)stepSize * 100 / (float)numberOfDiffRadius);
}

void TemplateGenerator::writeSettings()
{
	CameraParameters camParam = CameraParameters();
	TemplateGenerationSettings temp = TemplateGenerationSettings();
	std::string filename = "linemod_settings.yml";
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs.writeComment("###### CAMERA PARAMETERS ######");
	fs << "video width" << camParam.videoWidth;
	fs << "video height" << camParam.videoHeight;
	fs << "camera fx" << camParam.fx;
	fs << "camera fy" << camParam.fy;
	fs << "camera cx" << camParam.cx;
	fs << "camera cy" << camParam.cy;
	fs << "distortion parameters" << camParam.distortionCoefficients;
	fs << "color" << cv::Scalar(0, 20, 100);
	fs.writeComment("###### TEMPLATE GENERATION SETTINGS ######");
	fs << "model folder" << temp.modelFolder;
	fs << "model file ending" << temp.modelFileEnding;
	fs << "only use color modality" << temp.onlyUseColorModality;
	fs << "in plane rotation starting angle" << temp.angleStart;
	fs << "in plane rotation stopping angle" << temp.angleStop;
	fs << "in plane rotation angle step" << temp.angleStep;
	fs << "distance start" << temp.startDistance;
	fs << "distance stop" << temp.endDistance;
	fs << "distance step" << temp.stepSize;
	fs << "icosahedron subdivisions" << temp.subdivisions;
}
