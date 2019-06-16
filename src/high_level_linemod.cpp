#include "high_level_linemod.h"

namespace lineMOD{
HighLevelLinemod::HighLevelLinemod(bool in_onlyColor,CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings) :
	onlyColor(in_onlyColor),
	videoWidth(in_camParams.videoWidth),
	videoHeight(in_camParams.videoHeight),
	lowerAngleStop(in_templateSettings.angleStart),
	upperAngleStop(in_templateSettings.angleStop),
	angleStep(in_templateSettings.angleStep),
	stepSize(in_templateSettings.stepSize),
	cx(in_camParams.cx),
	cy(in_camParams.cy),
	fieldOfView(360.0f / M_PI * atanf(videoHeight / (2 * in_camParams.fy)))
{

	if (!in_onlyColor) {
		std::vector<cv::Ptr<cv::linemod::Modality>> modality;
		//modality.push_back(cv::makePtr<cv::linemod::ColorGradient>());
		modality.push_back(cv::makePtr<cv::linemod::ColorGradient>(10.0f, 30, 55.0f));
		//modality.push_back(cv::makePtr<cv::linemod::DepthNormal>());
		modality.push_back(cv::makePtr<cv::linemod::DepthNormal>(2000, 50, 30, 2));

		static const int T_DEFAULTS[] = { 5,8 };
		detector = cv::makePtr<cv::linemod::Detector>(modality, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
	}
	else {
		std::vector<cv::Ptr<cv::linemod::Modality>> modality;
		modality.push_back(cv::makePtr<cv::linemod::ColorGradient>());
		static const int T_DEFAULTS[] = { 5,8 };
		detector = cv::makePtr<cv::linemod::Detector>(modality, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
	}


	generateRotMatForInplaneRotation();

}

HighLevelLinemod::~HighLevelLinemod()
{
	detector.release();
}


std::vector<cv::String> HighLevelLinemod::getClassIds() {
	return detector->classIds();
}
uint16 HighLevelLinemod::getNumClasses() {
	return detector->numClasses();
}
uint32 HighLevelLinemod::getNumTemplates() {
	return detector->numTemplates();
}

bool HighLevelLinemod::addTemplate(std::vector<cv::Mat> in_images, std::string in_modelName, glm::vec3 in_cameraPosition) {
	cv::Mat mask;
	cv::Mat maskRotated;
	std::vector<cv::Mat> templateImgs;
	cv::Mat colorRotated;
	cv::Mat depthRotated;

	cv::threshold(in_images[1], mask, 1, 65535, cv::THRESH_BINARY);
	mask.convertTo(mask, CV_8UC1);


	for (size_t q = 0; q < inPlaneRotationMat.size(); q++)
	{
		cv::warpAffine(mask, maskRotated, inPlaneRotationMat[q], mask.size());
		cv::warpAffine(in_images[0], colorRotated, inPlaneRotationMat[q], in_images[0].size());
		cv::warpAffine(in_images[1], depthRotated, inPlaneRotationMat[q], in_images[1].size());

		cv::Rect boundingBox;
		templateImgs.push_back(colorRotated);
		if (!onlyColor) {
			templateImgs.push_back(depthRotated);
		}

		uint64 template_id = detector->addTemplate(templateImgs, in_modelName, maskRotated, &boundingBox);
		templateImgs.clear();

		if (template_id == -1) {
			std::cout << "ERROR::Cant create Template" << std::endl;
			return false;
		}
		else {
			uint16 medianDepth = medianMat(depthRotated, boundingBox, 4);
			glm::vec3 translation;
			glm::qua<float32> quaternions;
			float32 currentInplaneAngle = -(lowerAngleStop + q * angleStep);
			calculateTemplatePose(translation, quaternions, in_cameraPosition, currentInplaneAngle);
			templates.push_back(Template(in_modelName, translation, quaternions, boundingBox, medianDepth));
		}


	}
	return true;
}

bool HighLevelLinemod::detectTemplate(std::vector<cv::Mat>& in_imgs) {
	//TODO generate automatic Hue Check

	matches.clear();
	objectPoses.clear();
	detector->match(in_imgs, 90.0f, matches);
	if (matches.size() > 0) {
		applyPostProcessing(in_imgs);

		//DRAW FEATURES OF BEST LINEMOD MATCH
		if (objectPoses.size() != 0) {
			const std::vector<cv::linemod::Template>& templates = detector->getTemplates(matches[0].class_id, matches[0].template_id);
			drawResponse(templates, 2, in_imgs[0], cv::Point(matches[0].x, matches[0].y), detector->getT(0));
		}

	}
	else {
		return false;
	}
}

void HighLevelLinemod::writeLinemod() {
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
	uint64 numTemp = templates.size();
	templatePositionFile.write((char*)&numTemp, sizeof(uint64));
	for (uint64 i = 0; i < numTemp; i++)
	{
		templatePositionFile.write((char *)&templates[i], sizeof(Template));
	}
	templatePositionFile.close();
}

void HighLevelLinemod::readLinemod() {
	templates.clear();
	std::string filename = "linemod_templates.yml";
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	detector->read(fs.root());

	cv::FileNode fn = fs["classes"];
	for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i) {
		detector->readClass(*i);
	}

	std::ifstream input = std::ifstream("linemod_tempPosFile.bin", std::ios::in | std::ios::binary);
	if (!input.is_open()) {
		//TODO raise error
	}
	uint64 numTemp;
	input.read((char*)&numTemp, sizeof(uint64));
	Template tp;
	for (size_t i = 0; i < numTemp; i++)
	{
		input.read((char*)&tp, sizeof(Template));
		templates.push_back(tp);
	}
	input.close();
}


std::vector<ObjectPose> HighLevelLinemod::getObjectPoses() {
	return objectPoses;
}


void HighLevelLinemod::generateRotMatForInplaneRotation() {
	for (float32 angle = lowerAngleStop; angle <= upperAngleStop; angle = angle + angleStep)
	{
		cv::Point2f srcCenter(videoWidth / 2.0f, videoHeight / 2.0f);
		inPlaneRotationMat.push_back(cv::getRotationMatrix2D(srcCenter, angle, 1.0f));
	}
}

uint16 HighLevelLinemod::medianMat(cv::Mat in_mat, cv::Rect &in_bb, uint8 in_medianPosition) {
	cv::Mat invBinRot;
	cv::threshold(in_mat, invBinRot, 1, 65535, cv::THRESH_BINARY);
	invBinRot = 65535 - invBinRot;
	cv::Mat croppedDepth = in_mat + invBinRot;
	croppedDepth = croppedDepth(in_bb);

	std::vector<uint16>vecFromMat(croppedDepth.begin<uint16>(), croppedDepth.end<uint16>());
	std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 4, vecFromMat.end());
	return vecFromMat[vecFromMat.size() / in_medianPosition];
}

void HighLevelLinemod::calculateTemplatePose(glm::vec3& in_translation, glm::qua<float32>& in_quats, glm::vec3& in_cameraPosition, float32& in_inplaneRot) {
	in_translation.x = 0.0f;
	in_translation.y = 0.0f;
	in_translation.z = glm::length(in_cameraPosition);

	if (in_cameraPosition[0] == 0 && in_cameraPosition[2] == 0) { //Looking straight up or down fails the cross product
		in_cameraPosition[0] = 0.00000000001;
	}
	glm::vec3 camUp = glm::normalize(glm::cross(in_cameraPosition, glm::cross(in_cameraPosition, up)));
	glm::vec3 rotatedUp = glm::rotate(-camUp, glm::radians(in_inplaneRot), glm::normalize(in_cameraPosition));
	glm::mat4 view = glm::lookAt(in_cameraPosition, glm::vec3(0.f), rotatedUp);
	in_quats = openglCoordinatesystem2opencv(view);
}

glm::qua<float32> HighLevelLinemod::openglCoordinatesystem2opencv(glm::mat4& in_viewMat) {
	glm::qua<float32> tempQuat = glm::toQuat(in_viewMat);
	glm::vec3 eul = glm::eulerAngles(tempQuat);
	return glm::qua(glm::vec3(eul.x - M_PI / 2.0f, -eul.y, -eul.z));
}

bool HighLevelLinemod::applyPostProcessing(std::vector<cv::Mat>& in_imgs) {
	cv::Mat colorImgHue;
	cv::cvtColor(in_imgs[0], colorImgHue, cv::COLOR_BGR2HSV);
	cv::inRange(colorImgHue, cv::Scalar(10, 0, 0), cv::Scalar(30, 255, 255), colorImgHue); //TODO RANGE

	for (uint32 i = 0; i < matches.size(); i++)
	{

		if (!onlyColor) {
			if (colorCheck(colorImgHue, i, 50) && depthCheck(in_imgs[1], i)) {
				updateTranslationAndCreateObjectPose(i);
			}
		}
		else {
			if (colorCheck(colorImgHue, i, 50)) {
				updateTranslationAndCreateObjectPose(i);
			}
		}
		if (objectPoses.size() == 3) {//TODO NON HARDCODE
			return true;
		}
		if (i == (matches.size() - 1)) {
			if (!objectPoses.empty()) {
				return true;
			}
			else {
				//updateTranslationAndCreateObjectPose(0);
				return false;
			}
		}

	}
}

bool HighLevelLinemod::colorCheck(cv::Mat &in_colImg, uint32& in_numMatch, float32 in_percentCorrectColor) {
	cv::Mat croppedImage;
	croppedImage = in_colImg(cv::Rect(
		matches[in_numMatch].x,
		matches[in_numMatch].y,
		templates[matches[in_numMatch].template_id].boundingBox.width,
		templates[matches[in_numMatch].template_id].boundingBox.height));

	float32 nonZer = cv::countNonZero(croppedImage) * 100 /
		(templates[matches[in_numMatch].template_id].boundingBox.width *
			templates[matches[in_numMatch].template_id].boundingBox.height);
	if (nonZer > in_percentCorrectColor) {
		return true;
	}
	else {
		return false;
	}
}

bool HighLevelLinemod::depthCheck(cv::Mat &in_depth, uint32& in_numMatch) {
	cv::Rect bb(
		matches[in_numMatch].x,
		matches[in_numMatch].y,
		templates[matches[in_numMatch].template_id].boundingBox.width,
		templates[matches[in_numMatch].template_id].boundingBox.height);
	int32 diff = (int32)medianMat(in_depth, bb, 4) - (int32)templates[matches[in_numMatch].template_id].medianDepth;
	//templates[matches[in_numMatch].template_id].translation.z += (float32)diff;
	if (abs(diff) < stepSize / 2) {
		return true;
	}
	else {
		return false;
	}
}

void HighLevelLinemod::updateTranslationAndCreateObjectPose(uint32 in_numMatch) {
	glm::vec3 updatedTanslation;
	float32 alpha;

	updatedTanslation.z = templates[matches[in_numMatch].template_id].translation.z;
	alpha = ((1.0f - (matches[in_numMatch].y + cy -
		templates[matches[in_numMatch].template_id].boundingBox.y) / (cy)) * fieldOfView / 2.0f);

	updatedTanslation.y = -tan(alpha*M_PI / 180) * updatedTanslation.z;
	alpha = ((1.0f - (matches[in_numMatch].x + cx -
		templates[matches[in_numMatch].template_id].boundingBox.x) / (cx)) * fieldOfView * ((float32)videoWidth / (float32)videoHeight) / 2.0f);

	updatedTanslation.x = -tan(alpha*M_PI / 180) * updatedTanslation.z;

	cv::Rect boundingBox(
		matches[in_numMatch].x,
		matches[in_numMatch].y,
		templates[matches[in_numMatch].template_id].boundingBox.width,
		templates[matches[in_numMatch].template_id].boundingBox.height);

	objectPoses.push_back(ObjectPose(updatedTanslation, templates[matches[in_numMatch].template_id].quaternions, boundingBox));
}
}