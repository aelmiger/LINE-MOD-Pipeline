#include "HighLevelLinemodIcp.h"

HighLevelLinemodIcp::HighLevelLinemodIcp(uint16_t in_iteration, float in_tolerance, float in_rejectionScale,
	uint16_t in_numIterations, uint16_t in_sampleStep,
	std::vector<std::string> in_modelFiles,
	std::string in_modFolder) :
	modelFiles(std::move(in_modelFiles)),
	modelFolder(std::move(in_modFolder)),
	sampleStep(in_sampleStep)
{
	icp = new cv::ppf_match_3d::ICP(in_iteration, in_tolerance, in_rejectionScale, in_numIterations);
	loadModels();
}

HighLevelLinemodIcp::~HighLevelLinemodIcp()
{
	delete icp;
}

void HighLevelLinemodIcp::loadModels()
{
	for (const auto& modelFile : modelFiles)
	{
		cv::Mat tempModel = cv::ppf_match_3d::loadPLYSimple((modelFolder + modelFile).c_str(), 1);
		int numRows = tempModel.rows / sampleStep;
		cv::Mat sampledPC = cv::Mat(numRows, tempModel.cols, tempModel.type());
		int c = 0;
		for (int i = 0; i < tempModel.rows && c < numRows; i += sampleStep)
		{
			tempModel.row(i).copyTo(sampledPC.row(c++));
		}
		modelVertices.push_back(sampledPC);
	}
}

void HighLevelLinemodIcp::prepareDepthForIcp(cv::Mat& in_depth, const cv::Mat& in_camMatrix, cv::Rect& bb)
{
	cv::Mat blurredDepth;
	cv::blur(in_depth, blurredDepth, cv::Size(3, 3));
	cv::Mat mask = cv::Mat::zeros(cv::Size(640, 480), CV_8U);
	cv::rectangle(mask, bb, cv::Scalar(255, 255, 255), -1);

	cv::Mat_<cv::Vec3f> pointsModel;
	cv::rgbd::depthTo3d(blurredDepth, in_camMatrix, pointsModel, mask);
	cv::Mat modelMat;
	modelMat = pointsModel.t();
	modelMat = modelMat.reshape(1);
	modelMat = modelMat * 1000;
	cv::patchNaNs(modelMat);
	cv::Mat patchedNaNs;
	removeIfTooFarFromMean(modelMat, patchedNaNs, true);
	int numRows = patchedNaNs.rows / sampleStep;
	cv::Mat sampledpatchedNaNs = cv::Mat(numRows, patchedNaNs.cols, patchedNaNs.type());;
	int c = 0;
	for (int i = 0; i < patchedNaNs.rows && c < numRows; i += sampleStep)
	{
		patchedNaNs.row(i).copyTo(sampledpatchedNaNs.row(c++));
	}

	cv::Vec3d viewpoint(0, 0, 0);
	cv::ppf_match_3d::computeNormalsPC3d(sampledpatchedNaNs, sceneVertices, 12, false, viewpoint);
	//TODO Change what to do if patchedNaNs = 0
}

void HighLevelLinemodIcp::registerToScene(std::vector<ObjectPose>& in_poses, uint16_t in_modelNumber)
{
	poses.clear();
	for (auto& in_pose : in_poses)
	{
		cv::ppf_match_3d::Pose3DPtr pose(new cv::ppf_match_3d::Pose3D());
		cv::Matx33d rotM;
		cv::Vec3d tranV;
		tranV[0] = in_pose.translation.x;
		tranV[1] = in_pose.translation.y;
		tranV[2] = in_pose.translation.z;

		fromGLM2CV(toMat3(in_pose.quaternions), &rotM);

		pose->updatePose(rotM, tranV);
		poses.push_back(pose);
	}

	icp->registerModelToScene(modelVertices[in_modelNumber], sceneVertices, poses);
	for (size_t n = 0; n < poses.size(); n++)
	{
		updatePosition(poses[n]->pose, in_poses[n]);
	}
}

uint16_t HighLevelLinemodIcp::estimateBestMatch(cv::Mat in_depthImg, std::vector<ObjectPose> in_poses,
	OpenGLRender* in_openglRend, uint16_t in_modelIndice, uint16_t& in_bestPose)
{
	uint16_t bestMean = 0;
	uint16_t bestPose = 0;
	for (size_t i = 0; i < in_poses.size(); i++)
	{
		glm::vec3 eul = eulerAngles(in_poses[i].quaternions); //TODO FIX COORDINATE TRANSFORM
		glm::qua<float> quats(glm::vec3(eul.x + M_PI, -eul.y, -eul.z)); //TODO IMPORTANT adjust to benchmark
		glm::mat4 newViewMat = glm::toMat4(quats);
		in_openglRend->renderDepthToFrontBuff(in_modelIndice, newViewMat, in_poses[i].translation);
		cv::Mat depth = in_openglRend->getDepthImgFromBuff();
		cv::Mat binary;
		depthToBinary(depth, binary);
		cv::Mat mask;
		depthToBinary(in_depthImg, mask, 600);
		mask = 255 - mask;
		binary = binary - mask;
		cv::Mat maskedDepthImg;
		cv::Mat maskedDepth;
		cv::Mat diffImg;
		erodeMask(binary, binary, 2);
		in_depthImg.copyTo(maskedDepthImg, binary);
		depth.copyTo(maskedDepth, binary);
		absdiff(maskedDepthImg, maskedDepth, diffImg);

		cv::Scalar mean = cv::mean(diffImg, binary);

		if ((mean[0] < bestMean && mean[0] != 0) || i == 0)
		{
			bestPose = i;
			bestMean = mean[0];
		}
	}
	if (bestMean <= 25 && !in_poses.empty())
	{
		in_bestPose = bestPose;
		return true;
	}
	else {
		return false;
	}
}

void HighLevelLinemodIcp::removeIfTooFarFromMean(const cv::Mat& in_mat, cv::Mat& in_res, bool in_removeRows)
{
	in_res.release();
	cv::Mat depthCol = cv::Mat(in_mat, cv::Rect(2, 0, 1, in_mat.rows));
	cv::Scalar mean = cv::mean(depthCol);
	int n = in_removeRows ? in_mat.rows : in_mat.cols;
	for (int i = 0; i < n; i++)
	{
		cv::Mat rc = in_removeRows ? in_mat.row(i) : in_mat.col(i);
		if (abs(rc.at<float>(0,2) - mean[0]) > 300)
		{
			continue; // remove element
		}
		if (in_res.empty())
		{
			in_res = rc;
		}
		else
		{
			if (in_removeRows)
				vconcat(in_res, rc, in_res);
			else
				hconcat(in_res, rc, in_res);
		}
	}
}

void HighLevelLinemodIcp::depthToBinary(cv::Mat& in_gray, cv::Mat& in_binary, uint32_t in_threshold)
{
	cv::threshold(in_gray, in_binary, in_threshold, 65535, cv::THRESH_BINARY);
	in_binary.convertTo(in_binary, CV_8UC1);
}