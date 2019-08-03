#include "Benchmark.h"

Benchmark::Benchmark()
{
}

Benchmark::~Benchmark()
{
}

void Benchmark::increaseImgCounter()
{
	imageCounter++;
	printHodanScore();
	printLMScore();
}

float Benchmark::calculateErrorHodan(cv::Mat& in_depthImg, OpenGLRender* in_opengl, ObjectPose const& in_estimatePose, uint16_t const& in_modelIndice)
{
	readGroundTruthLinemodDataset();
	//readGroundTruthPose();
	inputDepth = in_depthImg;
	groundTruthDepthRender = renderPose(in_opengl, groundTruth, in_modelIndice);
	estimateDepthRender = renderPose(in_opengl, in_estimatePose, in_modelIndice);
	calculateVisibilityMasks();
	cv::absdiff(groundTruthDepthRender, estimateDepthRender, renderedAbsDiff);
	cv::threshold(renderedAbsDiff, renderedAbsDiff, errorThreshold, 65536, cv::THRESH_BINARY_INV);
	cv::bitwise_and(visibilityIntersection, renderedAbsDiff, absDiffMaskApplied);
	float error = 1 - (float)cv::countNonZero(absDiffMaskApplied) / (float)cv::countNonZero(visibilityCombination);
	if (error < 0.3f) {
		hodanCounter++;
	}
	return error;
}

float Benchmark::calculateErrorLM(ObjectPose& in_estimate)
{
	readGroundTruthLinemodDataset();
	cv::Mat rotMatGroundTruth;
	cv::Mat rotMatEstimate;

	fromGLM2CV(toMat3(groundTruth.quaternions), &rotMatGroundTruth);
	fromGLM2CV(toMat3(in_estimate.quaternions), &rotMatEstimate);

	cv::Mat difference(model.numVertices, 1, CV_32F);

#pragma omp parallel for
	for (int32_t i = 0; i < model.numVertices; i++)
	{
		cv::Matx31f vertModel(model.vertices[i].x, model.vertices[i].y, model.vertices[i].z);

		cv::Mat groundTruthRotatedTranslated = rotMatGroundTruth * vertModel + cv::Matx31f(
			groundTruth.translation.x, groundTruth.translation.y, groundTruth.translation.z);

		cv::Mat estimateTruthRotatedTranslated = rotMatEstimate * vertModel + cv::Matx31f(
			in_estimate.translation.x, in_estimate.translation.y, in_estimate.translation.z);

		cv::Mat differenceBetweenMat = groundTruthRotatedTranslated - estimateTruthRotatedTranslated;
		cv::Vec3f differenceBetweenVec(differenceBetweenMat.at<float>(0, 0), differenceBetweenMat.at<float>(1, 0),
			differenceBetweenMat.at<float>(2, 0));
		difference.at<float>(i, 0) = length(differenceBetweenVec);
	}
	float mean = sum(difference)[0] / model.numVertices;
	if (mean<= 15.2633)
	{
		lineCounter++;
	}
	return mean;
}

void Benchmark::loadModel(OpenGLRender* in_opengl, std::string in_modelLocation)
{
	in_opengl->readModelFile(in_modelLocation, model);

	subsamplingModel();
}

float Benchmark::calculateErrorLMAmbigous(ObjectPose& in_estimate)
{
	readGroundTruthLinemodDataset();
	cv::Mat rotMatGroundTruth;
	cv::Mat rotMatEstimate;

	fromGLM2CV(toMat3(groundTruth.quaternions), &rotMatGroundTruth);
	fromGLM2CV(toMat3(in_estimate.quaternions), &rotMatEstimate);

	cv::Mat difference(subsampledModel.numVertices, 1, CV_32F);

#pragma omp parallel for
	for (int32_t i = 0; i < subsampledModel.numVertices; i++)
	{
		cv::Matx31f vertModelGT(subsampledModel.vertices[i].x, subsampledModel.vertices[i].y, subsampledModel.vertices[i].z);

		cv::Mat groundTruthRotatedTranslated = rotMatGroundTruth * vertModelGT + cv::Matx31f(
			groundTruth.translation.x, groundTruth.translation.y, groundTruth.translation.z);
		float absDifference = 999999;
		for (int32_t j = 0; j < subsampledModel.numVertices; j++)
		{
			cv::Matx31f vertModelEst(subsampledModel.vertices[j].x, subsampledModel.vertices[j].y, subsampledModel.vertices[j].z);

			cv::Mat estimateTruthRotatedTranslated = rotMatEstimate * vertModelEst + cv::Matx31f(
				in_estimate.translation.x, in_estimate.translation.y, in_estimate.translation.z);
			cv::Mat differenceBetweenMat = groundTruthRotatedTranslated - estimateTruthRotatedTranslated;
			cv::Vec3f differenceBetweenVec(differenceBetweenMat.at<float>(0, 0), differenceBetweenMat.at<float>(1, 0),
				differenceBetweenMat.at<float>(2, 0));
			float tmpDiff = length(differenceBetweenVec);
			if (tmpDiff < absDifference) {
				absDifference = tmpDiff;
			}
		}

		difference.at<float>(i, 0) = absDifference;
	}
	float mean = sum(difference)[0] / subsampledModel.numVertices;
	if (mean <= 15.2633)
	{
		lineCounter++;
	}
	return mean;
}

void Benchmark::calculateVisibilityMasks()
{
	cv::Mat binaryGTRender;
	cv::Mat binaryEstRender;

	groundTruthVisibility = groundTruthDepthRender - inputDepth;
	cv::threshold(groundTruthVisibility, groundTruthVisibility, visibilityThreshold, 65536, cv::THRESH_BINARY);
	cv::threshold(groundTruthDepthRender, binaryGTRender, 1, 65536, cv::THRESH_BINARY);
	groundTruthVisibility = binaryGTRender - groundTruthVisibility;

	estimateVisibility = estimateDepthRender - inputDepth;
	cv::threshold(estimateVisibility, estimateVisibility, visibilityThreshold, 65536, cv::THRESH_BINARY);
	cv::threshold(estimateDepthRender, binaryEstRender, 1, 65536, cv::THRESH_BINARY);
	estimateVisibility = binaryEstRender - estimateVisibility;
	cv::bitwise_and(groundTruthVisibility, estimateDepthRender, binaryEstRender);
	cv::bitwise_or(estimateVisibility, binaryEstRender, estimateVisibility);

	cv::bitwise_and(groundTruthVisibility, estimateVisibility, visibilityIntersection);
	cv::bitwise_or(groundTruthVisibility, estimateVisibility, visibilityCombination);
}

cv::Mat Benchmark::renderPose(OpenGLRender* in_opengl, ObjectPose const& in_pose, uint16_t const& in_modelIndice)
{
	glm::mat4 viewMat = calculateViewMat(in_pose);
	in_opengl->renderDepthToFrontBuff(in_modelIndice, viewMat, in_pose.translation);
	cv::Mat renderedImg = in_opengl->getDepthImgFromBuff();
	return renderedImg.clone();
}

glm::mat4 Benchmark::calculateViewMat(ObjectPose const &in_pose)
{
	glm::vec3 eul = eulerAngles(in_pose.quaternions); //TODO FIX COODINATE TRANSFORM
	glm::qua quats(glm::vec3(eul.x - M_PI, -eul.y, -eul.z)); //TODO IMPORTANT adjust to benchmark
	return toMat4(quats);
}

void Benchmark::subsamplingModel()
{
	subsampledModel.vertices.clear();
	for (int i = 0; i < model.numVertices; i += subsampleStep)
	{
		subsampledModel.vertices.push_back(model.vertices[i]);
	}
	subsampledModel.numVertices = subsampledModel.vertices.size();
}

void Benchmark::readGroundTruthPose() {
	std::string filename = "benchmark/pose" + std::to_string(imageCounter) + ".yml";
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	cv::Matx33d rotMat;
	cv::Vec3d transVec;
	fs["rotMat"] >> rotMat;
	fs["position"] >> transVec;

	glm::mat3 glmRotMat;
	fromCV2GLM(cv::Mat(rotMat), &glmRotMat);
	groundTruth = { glm::vec3(transVec[0],transVec[1],transVec[2]), glm::toQuat(glmRotMat) };
}

void Benchmark::readGroundTruthLinemodDataset()
{
	glm::vec3 translation;
	cv::Vec3f angles;
	double tempNumber;

	std::ifstream fileStreamTranslation("data/tra" + std::to_string(imageCounter) + ".tra");
	if (!fileStreamTranslation.is_open())
	{
		std::cout << "Error Opening Ground Truth Translation file from Linemod Dataset" << std::endl;
	}

	//skip first to numbers
	fileStreamTranslation >> tempNumber;
	fileStreamTranslation >> tempNumber;

	fileStreamTranslation >> translation.x;
	fileStreamTranslation >> translation.y;
	fileStreamTranslation >> translation.z;
	fileStreamTranslation.close();

	cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64FC1);
	std::ifstream fileStreamRotation("data/rot" + std::to_string(imageCounter) + ".rot");
	if (!fileStreamRotation.is_open())
	{
		std::cout << "Error Opening Ground Truth Rotation file from Linemod Dataset" << std::endl;
	}
	int cnt = 0;
	//skip first to numbers
	fileStreamRotation >> tempNumber;
	fileStreamRotation >> tempNumber;

	while (fileStreamRotation >> tempNumber)
	{
		int temprow = cnt / 3;
		int tempcol = cnt % 3;
		rotMat.at<double>(temprow, tempcol) = tempNumber;
		cnt++;
	}
	fileStreamRotation.close();
	glm::mat3 rotMatGlm;
	fromCV2GLM(rotMat, &rotMatGlm);
	glm::qua<float> quaternions = quat_cast(rotMatGlm);
	glm::vec3 eul = eulerAngles(quaternions); //TODO FIX COORDINATE TRANSFORM
	glm::qua adjustedQuats(glm::vec3(eul.x - M_PI / 2, eul.y, eul.z)); // converts the LINEMOD coordinate system
	translation *= 10;
	groundTruth = { translation, adjustedQuats };
}

void Benchmark::printHodanScore()
{
	std::cout << "Hodan Score: " << (float)hodanCounter * 100 / (float)imageCounter << " Counter: " << imageCounter << "            " << std::endl;
}
void Benchmark::printLMScore()
{
	std::cout << "LM Score: " << (float)lineCounter * 100 / (float)imageCounter << " Counter: " << imageCounter << "            " << std::endl;
}