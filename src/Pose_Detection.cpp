#include "Pose_Detection.h"

Pose_Detection::Pose_Detection(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings) :
	modelFolder(in_templateSettings.modelFolder),
	cameraMatrix(in_camParams.cameraMatrix)
{
	filesInDirectory(modelFiles, modelFolder, in_templateSettings.modelFileEnding);
	opengl = new OpenGLRender(in_camParams);
	line = new HighLevelLinemod(in_camParams, in_templateSettings);
	icp = new HighLevelLinemodIcp(5, 0.1f, 2.5f, 8, 2, modelFiles, modelFolder);
}

Pose_Detection::~Pose_Detection()
{
	delete opengl;
	delete line;
	delete icp;
}

void Pose_Detection::run() {
	for (size_t i = 0; i < modelFiles.size(); i++)
	{
		opengl->creatModBuffFromFiles(modelFolder + modelFiles[i]);
	}
	readLinemodFromFile();

	////////////////ONLY RELEVANT FOR BENCHMARK
	Model tmp;
	int counter = 0;
	float64 accumDiff = 0;
	ObjectPose groundTruth;
	opengl->readModelFile(modelFolder + modelFiles[0], tmp); //TODO welches file
	///////////////
	std::vector<float> score;

	//Kinect2 kin2;
	//cv::VideoCapture cap(0);
	cv::VideoCapture sequence("data/color%0d.jpg");

	while (true) {
		std::string numb;
		//cap.read(A);
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

		float32 scoreNew = 100;
		inputImg.push_back(colorImg);
		inputImg.push_back(depthImg);
		for (size_t numClass = 0; numClass < line->getNumClasses(); numClass++)
		{
			finalObjectPoses.clear();
			line->detectTemplate(inputImg, numClass);
			detectedPoses = line->getObjectPoses();
			uint16 bestPose = 0;
			if (!detectedPoses.empty()) {
				for (size_t numDetectedPoses = 0; numDetectedPoses < detectedPoses.size(); numDetectedPoses++)
				{
					//icp->prepareDepthForIcp(depthImg, cameraMatrix, detectedPoses[numDetectedPoses][0].boundingBox);
					//icp->registerToScene(detectedPoses[numDetectedPoses], numClass);
					//bestPose = icp->estimateBestMatch(depthImg, detectedPoses[numDetectedPoses], opengl, numClass);
					drawCoordinateSystem(colorImg, cameraMatrix, 75.0f, detectedPoses[numDetectedPoses][bestPose]);
					finalObjectPoses.push_back(detectedPoses[numDetectedPoses][bestPose]);
				}
				scoreNew = matchingScoreParallel(tmp, groundTruth, detectedPoses[0][bestPose]);
				std::cout << "final " << bestPose << ": " << scoreNew << std::endl;

				if (scoreNew <= 11) { //TODO MAGIC NUMBER
					accumDiff++;
				}
				//cv::putText(colorImg, glm::to_string(detectedPoses[bestPose].translation), cv::Point(50, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 0, 255), 2.0f);
				//cv::putText(colorImg, glm::to_string(glm::degrees(glm::eulerAngles(detectedPoses[bestPose].quaternions))), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 255, 255), 2.0f);
			}
		}
		score.push_back(scoreNew);

		counter++;
		cv::imshow("color", colorImg);
		if (cv::waitKey(1) == 27) {
			break;
		}
		inputImg.clear();
		finalObjectPoses.clear();
	}
}

void Pose_Detection::readLinemodFromFile() {
	line->readLinemod();
	ids = line->getClassIds();
	int num_classes = line->getNumClasses();
	std::cout << "Loaded with " << num_classes << " classes and " << line->getNumTemplates() << " templates\n" << std::endl;
	if (!ids.empty())
	{
		std::cout << "Class ids: " << std::endl;
		std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	}
	if (ids != modelFiles) {
		std::cout << "ERROR::Models in file folder do not match with generated models!" << std::endl;
	}
}