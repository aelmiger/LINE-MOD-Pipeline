#include "PoseDetection.h"

PoseDetection::PoseDetection()
{
	readSettings(camParams, templateSettings);

	filesInDirectory(modelFiles, templateSettings.modelFolder, templateSettings.modelFileEnding);
	opengl = new OpenGLRender(camParams); //TODO unique_ptr shared_ptr
	line = new HighLevelLineMOD(camParams, templateSettings);
	icp = new HighLevelLinemodIcp(6, 0.1f, 2.5f, 8, templateSettings.icpSubsamplingFactor, modelFiles, templateSettings.modelFolder);

	for (const auto& modelFile : modelFiles)
	{
		opengl->creatModBuffFromFiles(templateSettings.modelFolder + modelFile);
	}
	readLinemodFromFile();
}

PoseDetection::~PoseDetection()
{
	cleanup();
}

void PoseDetection::cleanup() {
	if (opengl) {
		delete opengl;
	}
	if (line) {
		delete line;
	}
	if (icp)
	{
		delete icp;
	}
	if (bench)
	{
		delete bench;
	}
}

void PoseDetection::run(std::vector<cv::Mat>& in_imgs,std::string const& in_className, uint16_t const& in_numberOfObjects,std::vector<ObjectPose>& in_objPose)
{
	uint16_t numClassIndex = findIndexInVector(in_className, ids);

	colorImg = in_imgs[0];
	depthImg = in_imgs[1];
	//TODO eventuell noch eine exe f�r Benchmarks
		
	cv::Mat correctedTranslationColor = colorImg.clone();
	cv::Mat correctedTranslationDepth = depthImg.clone();
	translateImg(correctedTranslationColor, -camParams.cx + camParams.videoWidth / 2, -camParams.cy + camParams.videoHeight / 2);
	translateImg(correctedTranslationDepth, -camParams.cx + camParams.videoWidth / 2, -camParams.cy + camParams.videoHeight / 2);

	
	inputImg.push_back(correctedTranslationColor);
	inputImg.push_back(correctedTranslationDepth);

	finalObjectPoses.clear();
	line->detectTemplate(inputImg, numClassIndex);
	detectedPoses = line->getObjectPoses();
	if (!detectedPoses.empty())
	{
		for (auto& detectedPose : detectedPoses)
		{
			if (templateSettings.useIcp) {
				uint16_t bestPose = 0;
				icp->prepareDepthForIcp(depthImg, camParams.cameraMatrix, detectedPose[0].boundingBox);
				icp->registerToScene(detectedPose, numClassIndex);
				bool truePositivMatch = icp->estimateBestMatch(correctedTranslationDepth, detectedPose, opengl, numClassIndex,bestPose);
				if (truePositivMatch) {
					finalObjectPoses.push_back(detectedPose[bestPose]);
				}
			}
			else {
				finalObjectPoses.push_back(detectedPose[0]);
			}
			if (finalObjectPoses.size() == in_numberOfObjects) {
					break;
			}
		}
		if (!finalObjectPoses.empty()) {
			if (bench) {
				float scoreNew = 100;
				float error = 1;
				error = bench->calculateErrorHodan(correctedTranslationDepth, opengl, finalObjectPoses[0], numClassIndex);
				//scoreNew = bench->calculateErrorLM(finalObjectPoses[0]);
				//scoreNew = bench->calculateErrorLMAmbigous(finalObjectPoses[0]);
				std::cout << "final " << ": " << scoreNew << "  newBench: " << error << std::endl;

				//cv::putText(colorImg, glm::to_string(finalObjectPoses[0].translation), cv::Point(50, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 0, 255), 2.0f);
				//cv::putText(colorImg, glm::to_string(glm::degrees(glm::eulerAngles(finalObjectPoses[0].quaternions))), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0, 255, 255), 2.0f);
				//cv::putText(colorImg, std::to_string(error), cv::Point(50, 140), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(255, 0, 0), 2.0f);
			}
			for (auto& finalObjectPose:finalObjectPoses)
			{
				in_objPose.push_back(finalObjectPose);
				drawCoordinateSystem(colorImg, camParams.cameraMatrix, 75.0f, finalObjectPose);
			}
		}
	}
	bench->increaseImgCounter();
	imshow("color", colorImg);
	if (cv::waitKey(1) == 27)
	{
	}
	inputImg.clear();
	
}

void PoseDetection::setupBenchmark(std::string const& in_className) {
	bench = new Benchmark;
	uint16_t numClassIndex = findIndexInVector(in_className, ids);
	bench->loadModel(opengl, templateSettings.modelFolder + modelFiles[numClassIndex]);
}

uint16_t PoseDetection::findIndexInVector(std::string const& in_stringToFind, std::vector<std::string>& in_vectorToLookIn) {
	auto ind = std::find(in_vectorToLookIn.begin(), in_vectorToLookIn.end(), in_stringToFind);
	return std::distance(in_vectorToLookIn.begin(), ind);
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

void PoseDetection::drawCoordinateSystem(cv::Mat& in_srcDstImage, const cv::Mat& in_camMat, float in_coordinateSystemLength,
	ObjectPose& in_objPos)
{
	cv::Mat rotMat;
	fromGLM2CV(toMat3(in_objPos.quaternions), &rotMat);

	cv::Vec3f tVec(in_objPos.translation.x, in_objPos.translation.y, in_objPos.translation.z);
	cv::Mat rVec;
	std::vector<cv::Point3f> coordinatePoints;
	std::vector<cv::Point2f> projectedPoints;
	cv::Point3f center(0.0f, 0.0f, 0.0f);
	cv::Point3f xm(in_coordinateSystemLength, 0.0f, 0.0f);
	cv::Point3f ym(0.0f, in_coordinateSystemLength, 0.0f);
	cv::Point3f zm(0.0f, 0.0f, in_coordinateSystemLength);

	coordinatePoints.push_back(center);
	coordinatePoints.push_back(xm);
	coordinatePoints.push_back(ym);
	coordinatePoints.push_back(zm);

	cv::Rodrigues(rotMat, rVec);

	cv::projectPoints(coordinatePoints, rVec, tVec, in_camMat, std::vector<double>(), projectedPoints);
	cv::line(in_srcDstImage, projectedPoints[0], projectedPoints[1], cv::Scalar(0, 0, 255), 2);
	cv::line(in_srcDstImage, projectedPoints[0], projectedPoints[2], cv::Scalar(0, 255, 0), 2);
	cv::line(in_srcDstImage, projectedPoints[0], projectedPoints[3], cv::Scalar(255, 0, 0), 2);
}

cv::Mat PoseDetection::translateImg(cv::Mat &in_img, int in_offsetx, int in_offsety)
{
	cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, in_offsetx, 0, 1, in_offsety);
	cv::warpAffine(in_img, in_img, trans_mat, in_img.size());
	return in_img;
}

