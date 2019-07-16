#include "PoseDetection.h"

PoseDetection::PoseDetection()
{
	readSettings(camParams, templateSettings);

	filesInDirectory(modelFiles, templateSettings.modelFolder, templateSettings.modelFileEnding);
	opengl = new OpenGLRender(camParams); //TODO unique_ptr shared_ptr
	line = new HighLevelLineMOD(camParams, templateSettings);
	icp = new HighLevelLinemodIcp(5, 0.1f, 2.5f, 8,5, modelFiles, templateSettings.modelFolder);
}

PoseDetection::~PoseDetection()
{
	delete opengl;
	delete line;
	delete icp;
}

void PoseDetection::run()
{
	for (const auto& modelFile : modelFiles)
	{
		opengl->creatModBuffFromFiles(templateSettings.modelFolder + modelFile);
	}
	readLinemodFromFile();
	
	//TODO eventuell noch eine exe für Benchmarks
	Benchmark bench;
	bench.loadModel(opengl, templateSettings.modelFolder + modelFiles[0]);
	int counter=0;
	//Kinect2 kin2;
	cv::VideoCapture sequence("data/color%0d.jpg");

	while (true)
	{
		std::string numb;
		sequence >> colorImg;
		//kin2.getKinectFrames(colorImg, depthImg);
		
		if (colorImg.empty())
		{
			std::cout << "End of Sequence" << std::endl;
			break;
		}
		depthImg = loadDepth("data/depth" + std::to_string(counter) + ".dpt");
		cv::Mat correctedTranslationColor = colorImg.clone();
		cv::Mat correctedTranslationDepth = depthImg.clone();
		translateImg(correctedTranslationColor, -camParams.cx+camParams.videoWidth/2, -camParams.cy+camParams.videoHeight/2);
		translateImg(correctedTranslationDepth, -camParams.cx + camParams.videoWidth / 2, -camParams.cy + camParams.videoHeight / 2);

		float scoreNew = 100;
		float error = 1;
		inputImg.push_back(correctedTranslationColor);
		inputImg.push_back(correctedTranslationDepth);
		for (size_t numClass = 0; numClass < line->getNumClasses(); numClass++)
		{
			finalObjectPoses.clear();
			line->detectTemplate(inputImg, numClass);
			detectedPoses = line->getObjectPoses();
			uint16_t bestPose = 0;
			if (!detectedPoses.empty())
			{
				for (auto& detectedPose : detectedPoses)
				{
					icp->prepareDepthForIcp(depthImg, camParams.cameraMatrix, detectedPose[0].boundingBox);
					icp->registerToScene(detectedPose, numClass);
					bestPose = icp->estimateBestMatch(correctedTranslationDepth, detectedPose, opengl, numClass);
					drawCoordinateSystem(colorImg, camParams.cameraMatrix, 75.0f, detectedPose[bestPose]);
					finalObjectPoses.push_back(detectedPose[bestPose]);
				}
				error = bench.calculateErrorHodan(correctedTranslationDepth, opengl, detectedPoses[0][bestPose], numClass);
				scoreNew = bench.calculateErrorLM(detectedPoses[0][bestPose]);
				//std::cout << "final " << bestPose << ": " << scoreNew << "  newBench: "<<error<<std::endl;
			}
		}
		bench.increaseImgCounter();
		counter++;
		imshow("color", colorImg);
		if (cv::waitKey(1) == 27)
		{
			break;
		}
		inputImg.clear();
		finalObjectPoses.clear();
	}
}

void PoseDetection::readLinemodFromFile()
{
	line->readLinemod();
	ids = line->getClassIds();
	int num_classes = line->getNumClasses();
	std::cout << "Loaded with " << num_classes << " classes and " << line->getNumTemplates() << " templates\n" << std::
		endl;
	if (!ids.empty())
	{
		std::cout << "Class ids: " << std::endl;
		std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	}
	if (ids != modelFiles)
	{
		std::cout << "ERROR::Models in file folder do not match with generated models!" << std::endl;
	}
}
