#include "utility.h"

/*
#################### CONVERSION UTILITY ####################
*/

bool fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat)
{
	cv::Mat tempMat;
	cvmat.convertTo(tempMat, CV_32FC1);
	if (tempMat.cols != 4 || tempMat.rows != 4 || tempMat.type() != CV_32FC1)
	{
		std::cout << "CV to GLM Matrix conversion Error" << std::endl;
		return false;
	}
	memcpy(value_ptr(*glmmat), tempMat.data, 16 * sizeof(float));
	*glmmat = transpose(*glmmat);
	return true;
}

bool fromCV2GLM(const cv::Mat& cvmat, glm::mat3* glmmat)
{
	cv::Mat tempMat;
	cvmat.convertTo(tempMat, CV_32FC1);
	if (tempMat.cols != 3 || tempMat.rows != 3 || tempMat.type() != CV_32FC1)
	{
		std::cout << "CV to GLM Matrix conversion Error" << std::endl;
		return false;
	}
	memcpy(value_ptr(*glmmat), tempMat.data, 9 * sizeof(float));
	*glmmat = transpose(*glmmat);
	return true;
}

bool fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat)
{
	if (cvmat->cols != 4 || cvmat->rows != 4)
	{
		(*cvmat) = cv::Mat(4, 4, CV_32F);
	}
	memcpy(cvmat->data, value_ptr(glmmat), 16 * sizeof(float));
	*cvmat = cvmat->t();
	return true;
}

bool fromGLM2CV(const glm::mat3& glmmat, cv::Mat* cvmat)
{
	if (cvmat->cols != 3 || cvmat->rows != 3)
	{
		(*cvmat) = cv::Mat(3, 3, CV_32F);
	}
	memcpy(cvmat->data, value_ptr(glmmat), 9 * sizeof(float));
	*cvmat = cvmat->t();
	return true;
}

bool fromGLM2CV(const glm::mat3& glmmat, cv::Matx33d* in_mat)
{
	cv::Mat cvmat;
	if (cvmat.cols != 3 || cvmat.rows != 3)
	{
		cvmat = cv::Mat(3, 3, CV_32F);
	}
	memcpy(cvmat.data, value_ptr(glmmat), 9 * sizeof(float));
	cvmat = cvmat.t();
	*in_mat = cvmat;
	return true;
}

/*
#################### FILE UTILITY ####################
*/

void filesInDirectory(std::vector<std::string>& in_filePathVector, const std::string& in_path, const std::string&
	in_extension)
{
	in_filePathVector.clear();
	for (auto& p : std::filesystem::directory_iterator(in_path)){
		if (p.path().extension() == in_extension)
		{
			in_filePathVector.push_back(p.path().filename().string());
		}
	}
	if (in_filePathVector.empty())
	{
		std::cout << "No " << in_extension << " files found in " << in_path << std::endl;
	}
	else
	{
		std::cout << "Found " << in_filePathVector.size() << " " << in_extension << " files in " << in_path << std::
			endl;
	}
}


cv::Mat loadDepth(const std::string& a_name)
{
	std::ifstream l_file(a_name, std::ios::in | std::ios::binary);

	if (l_file.fail())
	{
		printf("ERROR LOADING DEPTH FILE!\n");
	}

	int l_row;
	int l_col;

	l_file.read((char*)&l_row, sizeof(l_row));
	l_file.read((char*)&l_col, sizeof(l_col));

	cv::Mat lp_image = cv::Mat::zeros(l_row, l_col, CV_16UC1);
	for (int l_r = 0; l_r < l_row; ++l_r)
	{
		for (int l_c = 0; l_c < l_col; ++l_c)
		{
			uint16_t depthVal;
			l_file.read((char*)&depthVal, sizeof(uint16_t));
			lp_image.at<uint16_t>(l_r, l_c) = depthVal;
		}
	}
	l_file.close();

	return lp_image;
}

/*
#################### CALC UTILITY ####################
*/



/*
#################### IMAGE UTILITY ####################
*/






void readSettings(CameraParameters& in_camParams, TemplateGenerationSettings& in_tempGenSettings) {
	std::string filename = "linemod_settings.yml";
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["video width"] >> in_camParams.videoWidth;
	fs["video height"] >> in_camParams.videoHeight;
	fs["camera fx"] >> in_camParams.fx;
	fs["camera fy"] >> in_camParams.fy;
	fs["camera cx"] >> in_camParams.cx;
	fs["camera cy"] >> in_camParams.cy;
	in_camParams.cameraMatrix = (cv::Mat1d(3, 3) << in_camParams.fx, 0, in_camParams.cx, 0, in_camParams.fy, in_camParams.cy, 0, 0, 1);
	fs["distortion parameters"] >> in_camParams.distortionCoefficients;

	fs["model folder"] >> in_tempGenSettings.modelFolder;
	fs["model file ending"] >> in_tempGenSettings.modelFileEnding;
	fs["only use color modality"] >> in_tempGenSettings.onlyUseColorModality;
	fs["in plane rotation starting angle"] >> in_tempGenSettings.angleStart;
	fs["in plane rotation stopping angle"] >> in_tempGenSettings.angleStop;
	fs["in plane rotation angle step"] >> in_tempGenSettings.angleStep;
	fs["distance start"] >> in_tempGenSettings.startDistance;
	fs["distance stop"] >> in_tempGenSettings.endDistance;
	fs["distance step"] >> in_tempGenSettings.stepSize;
	fs["icosahedron subdivisions"] >> in_tempGenSettings.subdivisions;
	fs["detector threshold"] >> in_tempGenSettings.detectorThreshold;
	fs["percent to pass check"] >> in_tempGenSettings.percentToPassCheck;
	fs["number of poses to compare"] >> in_tempGenSettings.numberWantedPoses;
	fs["distance to match to be considered same object"] >> in_tempGenSettings.radiusThresholdNewObject;
	fs["ratio to determine if group is too small"] >> in_tempGenSettings.discardGroupRatio;
	fs["use depth improvement"] >> in_tempGenSettings.useDepthImprovement;
	fs["depth offset"] >> in_tempGenSettings.depthOffset;
	fs["use icp"] >> in_tempGenSettings.useIcp;
	fs["icp subsampling factor"] >> in_tempGenSettings.icpSubsamplingFactor;
}