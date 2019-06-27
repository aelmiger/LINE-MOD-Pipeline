#include "Pose_Detection.h"


Pose_Detection::Pose_Detection(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings) :
	modelFolder(in_templateSettings.modelFolder),
	cameraMatrix(in_camParams.cameraMatrix)
{
	opengl = new OpenGLRender(in_camParams);
	line = new HighLevelLinemod(true, in_camParams, in_templateSettings);
	icp = new HighLevelLinemodIcp(5, 0.1f, 2.5f, 8, 10);
	filesInDirectory(modelFiles, modelFolder, in_templateSettings.modelFileEnding);
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
	//Model tmp;
	int counter = 0;
	//float64 accumDiff = 0;
	//opengl.readModelFile(modelFolder + modelFiles[0], tmp); //TODO welches file
	///////////////



	Kinect2 kin2;
	//cv::VideoCapture cap(0);
	//cv::VideoCapture sequence("data/color%0d.jpg");
	//glPolygonMode(GL_FRONT, GL_LINE);


	icp->loadModel("mesh.ply");

	while (true) {
		std::string numb;
		//cap.read(A);
		//sequence >> colorImg;
		//depthImg = loadDepth("data/depth" + std::to_string(counter) + ".dpt");
		kin2.getKinectFrames(colorImg, depthImg);
		/*
		if (colorImg.empty() || counter == 250)
		{
			std::cout << "End of Sequence" << std::endl;
			std::cout << "Mean Diff: " <<accumDiff / counter<<std::endl;
			break;
		}
		*/

		inputImg.push_back(colorImg);
		inputImg.push_back(depthImg);
		for (size_t numClass = 0; numClass < line->getNumClasses(); numClass++)
		{	
			finalObjectPoses.clear();
			line->detectTemplate(inputImg, numClass);
			detectedPoses = line->getObjectPoses();
			uint16 bestPose = 0;
			if (!detectedPoses.empty()) {

				//icp->prepareDepthForIcp(depthImg, cameraMatrix, detectedPoses[0].boundingBox);
				//icp->registerToScene(detectedPoses);

				counter++;

				//bestPose = icp->estimateBestMatch(depthImg, detectedPoses, opengl, 0); //TODO ANPASSEN
				drawCoordinateSystem(colorImg, cameraMatrix, 75.0f, detectedPoses[bestPose]);
				finalObjectPoses.push_back(detectedPoses[bestPose]);
				//float32 scoreNew = matchingScoreParallel(tmp, groundTruth, detectedPoses[bestPose]);
				//std::cout << "final " << bestPose << ": " << scoreNew << " : "<<bestMean<< std::endl;

				//if (scoreNew <= 14) {
				//	accumDiff++;
				//}
				//cv::putText(colorImg, glm::to_string(detectedPoses[bestPose].translation), cv::Point(50, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 0, 255), 2.0f);
				//cv::putText(colorImg, glm::to_string(glm::degrees(glm::eulerAngles(detectedPoses[bestPose].quaternions))), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 255, 255), 2.0f);

			}
		}

		//Model tmp;
		//ObjectPose groundTruth;
		//readGroundTruthLinemodDataset(counter, groundTruth);
		//opengl->readModelFile(modelFolder + modelFiles[0], tmp); //TODO welches file


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
		std::cout<<"Class ids: "<<std::endl;
		std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	}
	if (ids != modelFiles) {
		std::cout << "ERROR::Models in file folder do not match with generated models!" << std::endl;
	}
}