#include "high_level_linemod_icp.h"


HighLevelLinemodIcp::HighLevelLinemodIcp(uint16 in_iteration, float32 in_tolerance, float32 in_rejectionScale, uint16 in_numIterations, uint16 in_sampleStep) :
	sampleStep(in_sampleStep)
{
	icp = new cv::ppf_match_3d::ICP(in_iteration, in_tolerance, in_rejectionScale, in_numIterations);
}

HighLevelLinemodIcp::~HighLevelLinemodIcp()
{
	delete icp;
}

void HighLevelLinemodIcp::loadModel(std::string in_model) {
	modelVertices = cv::ppf_match_3d::loadPLYSimple(std::string("mesh.ply").c_str(), 1);
	int numRows = modelVertices.rows / sampleStep;
	cv::Mat sampledPC = cv::Mat(numRows, modelVertices.cols, modelVertices.type());
	int c = 0;
	for (int i = 0; i < modelVertices.rows && c < numRows; i += sampleStep)
	{
		modelVertices.row(i).copyTo(sampledPC.row(c++));
	}
	modelVertices = sampledPC;
}


void HighLevelLinemodIcp::prepareDepthForIcp(cv::Mat& in_depth, const cv::Mat& in_camMatrix, cv::Rect& bb) {
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
	remove_if(modelMat, patchedNaNs, true);
	cv::Vec3d viewpoint(0, 0, 0);
	cv::ppf_match_3d::computeNormalsPC3d(patchedNaNs, sceneVertices, 12, false, viewpoint);
}
void HighLevelLinemodIcp::registerToScene(std::vector<ObjectPose>& in_poses) {
	poses.clear();
	for (size_t i = 0; i < in_poses.size(); i++)
	{
		cv::ppf_match_3d::Pose3DPtr pose(new cv::ppf_match_3d::Pose3D());
		cv::Matx33d rotM;
		cv::Vec3d tranV;
		tranV[0] = in_poses[i].translation.x;
		tranV[1] = in_poses[i].translation.y;
		tranV[2] = in_poses[i].translation.z;

		fromGLM2CV(glm::toMat3(in_poses[i].quaternions), &rotM);

		pose->updatePose(rotM, tranV);
		poses.push_back(pose);

	}
	icp->registerModelToScene(modelVertices, sceneVertices, poses);


	for (size_t n = 0; n < poses.size(); n++)
	{
		updatePosition(poses[n]->pose, in_poses[n]);
	}


}

uint16 HighLevelLinemodIcp::estimateBestMatch(cv::Mat in_depthImg, std::vector<ObjectPose> in_poses, OpenGLRender* in_openglRend, uint16 in_modelIndice) {
	uint16 bestMean = 0;
	uint16 bestPose = 0;
	for (size_t i = 0; i < poses.size(); i++)
	{
		glm::vec3 eul = glm::eulerAngles(in_poses[i].quaternions);
		glm::qua quats(glm::vec3(eul.x + M_PI / 2.0f, -eul.y, -eul.z));
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
		//erodeMask(binary, binary, 2);
		in_depthImg.copyTo(maskedDepthImg, binary);
		depth.copyTo(maskedDepth, binary);
		cv::absdiff(maskedDepthImg, maskedDepth, diffImg);

		cv::Scalar mean = cv::mean(diffImg, binary);

		if ((mean[0] < bestMean && mean[0] != 0) || i == 0) {
			bestPose = i;
			bestMean = mean[0];
		}
	}
	return bestPose;

}



void HighLevelLinemodIcp::remove_if(const cv::Mat &in_mat, cv::Mat &in_res, bool in_removeRows)
{
	in_res.release();
	int n = in_removeRows ? in_mat.rows : in_mat.cols;
	for (int i = 0; i < n; i++)
	{
		cv::Mat rc = in_removeRows ? in_mat.row(i) : in_mat.col(i);
		if (cv::sum(rc)[0] == 0) {
			continue; // remove element
		}
		if (in_res.empty()) {
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



void HighLevelLinemodIcp::depthToBinary(cv::Mat &in_gray, cv::Mat &in_binary, uint32 in_threshold) {
	cv::threshold(in_gray, in_binary, in_threshold, 65535, cv::THRESH_BINARY);
	in_binary.convertTo(in_binary, CV_8UC1);
}
