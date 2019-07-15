#include "PoseDetection.h"

PoseDetection::PoseDetection()
{
	CameraParameters camParams;
	TemplateGenerationSettings templateSettings;
	readSettings(camParams, templateSettings);
	modelFolder= templateSettings.modelFolder;
	cameraMatrix = camParams.cameraMatrix;

	filesInDirectory(modelFiles, modelFolder, templateSettings.modelFileEnding);
	opengl = new OpenGLRender(camParams); //TODO unique_ptr shared_ptr
	line = new HighLevelLineMOD(camParams, templateSettings);
	icp = new HighLevelLinemodIcp(5, 0.1f, 2.5f, 8, 2, modelFiles, modelFolder);
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
		opengl->creatModBuffFromFiles(modelFolder + modelFile);
	}
	readLinemodFromFile();
	
	//TODO eventuell noch eine exe für Benchmarks
	////////////////ONLY RELEVANT FOR BENCHMARK
	Benchmark bench;
	Model tmp;
	int counter = 0;
	double accumDiff = 0;
	ObjectPose groundTruth;
	opengl->readModelFile(modelFolder + modelFiles[0], tmp); //TODO welches file
	///////////////
	std::vector<float> score;

	//Kinect2 kin2;
	cv::VideoCapture sequence("data/color%0d.jpg");

	while (true)
	{
		std::string numb;
		sequence >> colorImg;
		//kin2.getKinectFrames(colorImg, depthImg);
		readGroundTruthLinemodDataset(counter, groundTruth);
		
		if (colorImg.empty())
		{
			std::cout << "End of Sequence" << std::endl;
			std::cout << "Mean Diff: " << accumDiff / counter << std::endl;
			break;
		}
		depthImg = loadDepth("data/depth" + std::to_string(counter) + ".dpt");

		//float scoreNew = 100;
		inputImg.push_back(colorImg);
		inputImg.push_back(depthImg);
		for (size_t numClass = 0; numClass < line->getNumClasses(); numClass++)
		{
			finalObjectPoses.clear();
			line->detectTemplate(inputImg, numClass);
			detectedPoses = line->getObjectPoses();
			uint16_t bestPose = 0;
			if (!detectedPoses.empty())
			{
				bench.calculateError(depthImg, opengl, groundTruth, detectedPoses[0][0], numClass);
				for (auto& detectedPose : detectedPoses)
				{
				//	icp->prepareDepthForIcp(depthImg, cameraMatrix, detectedPose[0].boundingBox);
				//	icp->registerToScene(detectedPose, numClass);
				//	bestPose = icp->estimateBestMatch(depthImg, detectedPose, opengl, numClass);
					drawCoordinateSystem(colorImg, cameraMatrix, 75.0f, detectedPose[bestPose]);
					finalObjectPoses.push_back(detectedPose[bestPose]);
				}
				//scoreNew = matchingScoreParallel(tmp, groundTruth, detectedPoses[0][bestPose]);
				//std::cout << "final " << bestPose << ": " << scoreNew << std::endl;

				//if (scoreNew <= 11)
				//{
					//TODO MAGIC NUMBER
				//	accumDiff++;
				//}
				//cv::putText(colorImg, glm::to_string(detectedPoses[bestPose].translation), cv::Point(50, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 0, 255), 2.0f);
				//cv::putText(colorImg, glm::to_string(glm::degrees(glm::eulerAngles(detectedPoses[bestPose].quaternions))), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 255, 255), 2.0f);
			}
		}
		//score.push_back(scoreNew);

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
