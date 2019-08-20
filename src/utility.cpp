#include "utility.h"

/*
#################### CONVERSION UTILITY ####################
*/

glm::qua<float> openGL2openCVRotation(glm::mat4& in_viewMat)
{
	glm::qua<float> tempQuat = toQuat(in_viewMat);//TODO FIX COORDINATE TRANSFORM
	glm::vec3 eul = eulerAngles(tempQuat);
	return glm::qua<float>(glm::vec3(eul.x - CV_PI / 2.0f, -eul.y, -eul.z));
}

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

std::string fileToString(const char* filename)
{
	std::ifstream t(filename);
	std::stringstream buffer;
	buffer << t.rdbuf();
	t.close();
	std::string fds = buffer.str();
	return buffer.str();
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

float length(cv::Vec3f& in_vecA)
{
	return sqrt(in_vecA[0] * in_vecA[0] + in_vecA[1] * in_vecA[1] + in_vecA[2] * in_vecA[2]);
}

/*
#################### IMAGE UTILITY ####################
*/

void erodeMask(cv::Mat& in_mask, cv::Mat& in_erode, int in_numberIterations)
{
	erode(in_mask, in_erode, cv::Mat(), cv::Point(-1, -1), in_numberIterations);
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
	int num_modalities, cv::Mat& dst, const cv::Point& offset, int T)
{
	static const cv::Scalar COLORS[5] = {
		CV_RGB(0, 0, 255),
		CV_RGB(0, 255, 0),
		CV_RGB(255, 255, 0),
		CV_RGB(255, 140, 0),
		CV_RGB(0, 0,255)
	};

	for (int m = 0; m < num_modalities; ++m)
	{
		cv::Scalar color = COLORS[m];
		for (const auto f : templates[m].features)
		{
			const cv::Point pt(f.x + offset.x, f.y + offset.y);
			circle(dst, pt, T / 2, color);
		}
	}
}

void drawCoordinateSystem(cv::Mat& in_srcDstImage, const cv::Mat& in_camMat, float in_coordinateSystemLength,
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

	Rodrigues(rotMat, rVec);

	projectPoints(coordinatePoints, rVec, tVec, in_camMat, std::vector<double>(), projectedPoints);
	line(in_srcDstImage, projectedPoints[0], projectedPoints[1], cv::Scalar(0, 0, 255), 2);
	line(in_srcDstImage, projectedPoints[0], projectedPoints[2], cv::Scalar(0, 255, 0), 2);
	line(in_srcDstImage, projectedPoints[0], projectedPoints[3], cv::Scalar(255, 0, 0), 2);
}

void updatePosition(cv::Matx44d in_mat, ObjectPose& in_objPose)
{
	glm::mat4 transMat;
	fromCV2GLM(cv::Mat(in_mat), &transMat);
	in_objPose.quaternions = toQuat(transMat);
	in_objPose.translation = glm::vec3(in_mat(0, 3), in_mat(1, 3), in_mat(2, 3));
}

cv::Mat translateImg(cv::Mat &in_img, int in_offsetx, int in_offsety)
{
	cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, in_offsetx, 0, 1, in_offsety);
	cv::warpAffine(in_img, in_img, trans_mat, in_img.size());
	return in_img;
}

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