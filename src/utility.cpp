#include "utility.h"


/*
#################### GENERAL UTILITY ####################
*/

void remove_if(const cv::Mat &mat, cv::Mat &res, remove_predicate pred, bool removeRows)
{
	res.release();
	int n = removeRows ? mat.rows : mat.cols;
	for (int i = 0; i < n; i++)
	{
		cv::Mat rc = removeRows ? mat.row(i) : mat.col(i);
		if (pred(rc)) continue; // remove element
		if (res.empty()) res = rc;
		else
		{
			if (removeRows)
				vconcat(res, rc, res);
			else
				hconcat(res, rc, res);
		}
	}
}


bool is_zero(const cv::Mat &rc)
{
	return (cv::sum(rc)[0] == 0);
}

/*
#################### CONVERSION UTILITY ####################
*/

glm::qua<float32> openGL2openCVRotation(glm::mat4& in_viewMat) {
	glm::qua tempQuat = glm::toQuat(in_viewMat);
	glm::vec3 eul = glm::eulerAngles(tempQuat);
	return glm::qua(glm::vec3(eul.x - M_PI / 2.0f, -eul.y, -eul.z));
}

bool fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat) {
	cv::Mat tempMat;
	cvmat.convertTo(tempMat, CV_32FC1);
	if (tempMat.cols != 4 || tempMat.rows != 4 || tempMat.type() != CV_32FC1) {
		std::cout << "CV to GLM Matrix conversion Error" << std::endl;
		return false;
	}
	memcpy(glm::value_ptr(*glmmat), tempMat.data, 16 * sizeof(float32));
	*glmmat = glm::transpose(*glmmat);
	return true;
}

bool fromCV2GLM(const cv::Mat& cvmat, glm::mat3* glmmat) {
	cv::Mat tempMat;
	cvmat.convertTo(tempMat, CV_32FC1);
	if (tempMat.cols != 3 || tempMat.rows != 3 || tempMat.type() != CV_32FC1) {
		std::cout << "CV to GLM Matrix conversion Error" << std::endl;
		return false;
	}
	memcpy(glm::value_ptr(*glmmat), tempMat.data, 9 * sizeof(float32));
	*glmmat = glm::transpose(*glmmat);
	return true;
}

bool fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat) {
	if (cvmat->cols != 4 || cvmat->rows != 4) {
		(*cvmat) = cv::Mat(4, 4, CV_32F);
	}
	memcpy(cvmat->data, glm::value_ptr(glmmat), 16 * sizeof(float));
	*cvmat = cvmat->t();
	return true;
}

bool fromGLM2CV(const glm::mat3& glmmat, cv::Mat* cvmat) {
	if (cvmat->cols != 3 || cvmat->rows != 3) {
		(*cvmat) = cv::Mat(3, 3, CV_32F);
	}
	memcpy(cvmat->data, glm::value_ptr(glmmat), 9 * sizeof(float));
	*cvmat = cvmat->t();
	return true;
}
bool fromGLM2CV(const glm::mat3& glmmat, cv::Matx33d* in_mat) {
	cv::Mat cvmat;
	if (cvmat.cols != 3 || cvmat.rows != 3) {
		cvmat = cv::Mat(3, 3, CV_32F);
	}
	memcpy(cvmat.data, glm::value_ptr(glmmat), 9 * sizeof(float));
	cvmat = cvmat.t();
	*in_mat = cvmat;
	return true;
}


/*
#################### FILE UTILITY ####################
*/



void filesInDirectory(std::vector<std::string>& in_filePathVector, std::string in_path, std::string in_extension) {
	in_filePathVector.clear();
	std::filesystem::current_path(in_path);
	for (auto& p : std::filesystem::directory_iterator(""))
		if (p.path().extension() == in_extension) {
			in_filePathVector.push_back(p.path().string());
		}
	if (in_filePathVector.empty()) {
		std::cout << "No " << in_extension << " files found in " << in_path << std::endl;
	}
	else {
		std::cout << "Found " << in_filePathVector.size() << " " << in_extension << " files in " << in_path << std::endl;

	}
	std::filesystem::current_path("../");

}

std::string fileToString(const char* filename) {

	std::ifstream t(filename);
	std::stringstream buffer;
	buffer << t.rdbuf();
	t.close();
	std::string fds = buffer.str();
	return buffer.str();
}


static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, std::vector<TemplatePosition> &in_templatePositions)
{
	std::string filename = "linemod_templates.yml";
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	detector->write(fs);

	std::vector<cv::String> ids = detector->classIds();
	fs << "classes" << "[";
	for (int i = 0; i < (int)ids.size(); ++i)
	{
		fs << "{";
		detector->writeClass(ids[i], fs);
		fs << "}";
	}
	fs << "]";

	std::ofstream templatePositionFile("linemod_tempPosFile.bin", std::ios::binary | std::ios::out);
	uint64 numTemp = in_templatePositions.size();
	templatePositionFile.write((char*)&numTemp, sizeof(uint64));
	for (uint64 i = 0; i < numTemp; i++)
	{
		templatePositionFile.write((char *)&in_templatePositions[i], sizeof(TemplatePosition));
	}
	templatePositionFile.close();

}

static cv::Ptr<cv::linemod::Detector> readLinemod(std::vector<TemplatePosition> &in_templatePositions)
{
	std::string filename = "linemod_templates.yml";
	cv::Ptr<cv::linemod::Detector> detector = cv::makePtr<cv::linemod::Detector>();
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	detector->read(fs.root());

	cv::FileNode fn = fs["classes"];
	for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i) {
		detector->readClass(*i);
	}

	std::ifstream input = std::ifstream("linemod_tempPosFile.bin", std::ios::in | std::ios::binary);
	if (!input.is_open()) {
		std::cout << "Error loading " << "linemod_tempPosFile.bin" << std::endl;
	}
	uint64 numTemp;
	input.read((char*)&numTemp, sizeof(uint64));
	TemplatePosition tp;
	for (size_t i = 0; i < numTemp; i++)
	{
		input.read((char*)&tp, sizeof(TemplatePosition));
		in_templatePositions.push_back(tp);
	}
	input.close();

	return detector;
}

cv::Mat loadDepth(std::string a_name)
{
	std::ifstream l_file(a_name, std::ios::in | std::ios::binary);

	if (l_file.fail() == true)
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
			uint16 depthVal;
			l_file.read((char*)&depthVal, sizeof(uint16));
			lp_image.at<uint16>(l_r, l_c) = depthVal;

		}
	}
	l_file.close();

	return lp_image;
}


/*
#################### CALC UTILITY ####################
*/

uint16 medianMat(cv::Mat in_mat, cv::Rect &in_bb, uint8 in_medianPosition) {
	cv::Mat invBinRot;
	cv::threshold(in_mat, invBinRot, 1, 65535, cv::THRESH_BINARY);
	invBinRot = 65535 - invBinRot;
	cv::Mat croppedDepth = in_mat + invBinRot;
	croppedDepth = croppedDepth(in_bb);

	std::vector<uint16>vecFromMat(croppedDepth.begin<uint16>(), croppedDepth.end<uint16>());
	std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 4, vecFromMat.end());
	return vecFromMat[vecFromMat.size() / in_medianPosition];
}

float32 length(cv::Vec3f &in_vecA) {
	return sqrt(in_vecA[0] * in_vecA[0] + in_vecA[1] * in_vecA[1] + in_vecA[2] * in_vecA[2]);
}


void readGroundTruthLinemodDataset(uint32 in_fileNumber, ObjectPose &in_objectPos) {
	glm::vec3 translation;
	cv::Vec3f angles;
	float64 tempNumber;

	std::ifstream fileStreamTranslation("data/tra" + std::to_string(in_fileNumber) + ".tra");
	if (!fileStreamTranslation.is_open()) {
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
	std::ifstream fileStreamRotation("data/rot" + std::to_string(in_fileNumber) + ".rot");
	if (!fileStreamRotation.is_open()) {
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
	glm::qua<float32> quaternions = glm::quat_cast(rotMatGlm);
	translation *= 10;
	in_objectPos = { translation,quaternions };
}


float32 matchingScoreParallel(Model &in_model, ObjectPose &in_groundTruth, ObjectPose &in_estimate) {
	cv::Mat rotMatGroundTruth;
	cv::Mat rotMatEstimate;
	fromGLM2CV(glm::toMat3(in_groundTruth.quaternions), &rotMatGroundTruth);
	fromGLM2CV(glm::toMat3(in_estimate.quaternions), &rotMatEstimate);

	cv::Mat difference(in_model.numVertices, 1, CV_32F);

	concurrency::parallel_for(uint32(0), (uint32)in_model.numVertices, [&](uint32 i) {
		cv::Matx31f vertModel(in_model.vertices[i].x, in_model.vertices[i].y, in_model.vertices[i].z);

		cv::Mat groundTruthRotatedTranslated = rotMatGroundTruth * vertModel + cv::Matx31f(in_groundTruth.translation.x, in_groundTruth.translation.y, in_groundTruth.translation.z);

		cv::Mat estimateTruthRotatedTranslated = rotMatEstimate * vertModel + cv::Matx31f(in_estimate.translation.x, in_estimate.translation.y, in_estimate.translation.z);

		cv::Mat differenceBetweenMat = groundTruthRotatedTranslated - estimateTruthRotatedTranslated;
		cv::Vec3f differenceBetweenVec(differenceBetweenMat.at<float32>(0, 0), differenceBetweenMat.at<float32>(1, 0), differenceBetweenMat.at<float32>(2, 0));
		difference.at<float32>(i, 0) = length(differenceBetweenVec);
	});
	float32 mean = cv::sum(difference)[0] / in_model.numVertices;
	return mean;
}



/*
#################### IMAGE UTILITY ####################
*/

void depthToBinary(cv::Mat &in_gray, cv::Mat &in_binary, int in_threshold) {
	cv::threshold(in_gray, in_binary, in_threshold, 65535, cv::THRESH_BINARY);
	in_binary.convertTo(in_binary, CV_8UC1);
}

void erodeMask(cv::Mat &in_mask, cv::Mat &in_erode, int in_numberIterations) {
	cv::erode(in_mask, in_erode, cv::Mat(), cv::Point(-1, -1), in_numberIterations);
}

void rotateCvMat(cv::Mat &in_mat, cv::Mat &in_dstMat, cv::Mat &in_rotMat) {
	cv::warpAffine(in_mat, in_dstMat, in_rotMat, in_mat.size());
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
	int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
	static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
										  CV_RGB(0, 255, 0),
										  CV_RGB(255, 255, 0),
										  CV_RGB(255, 140, 0),
										  CV_RGB(255, 0, 0) };

	for (int m = 0; m < num_modalities; ++m)
	{
		cv::Scalar color = COLORS[m];
		for (int i = 0; i < (int)templates[m].features.size(); ++i)
		{
			cv::linemod::Feature f = templates[m].features[i];
			cv::Point pt(f.x + offset.x, f.y + offset.y);
			circle(dst, pt, T / 2, color);
		}
	}
}


void drawCoordinateSystem(cv::Mat &in_srcDstImage, cv::Mat in_camMat, float32 in_coordinateSystemLength, ObjectPose &in_objPos) {
	cv::Mat rotMat;
	fromGLM2CV(glm::toMat3(in_objPos.quaternions), &rotMat);

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



void updatePosition(cv::Matx44d in_mat, ObjectPose &in_objPose) {

	glm::mat4 transMat;
	fromCV2GLM(cv::Mat(in_mat), &transMat);
	in_objPose.quaternions = glm::toQuat(transMat);
	in_objPose.translation = glm::vec3(in_mat(0, 3), in_mat(1, 3), in_mat(2, 3));
}
