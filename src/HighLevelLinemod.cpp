#include "HighLevelLinemod.h"


/**
 * @brief Construct a new High Level LINE-MOD object
 * @detail The class deals with the template generation and detection, wraps the opencv linemod class into a package and adds functionality
 * 
 * @param in_camParams Contains the Camera Parameters
 * @param in_templateSettings Contains the settings for template generation and detection
 */
HighLevelLineMOD::HighLevelLineMOD(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings) :
	onlyColorModality(in_templateSettings.onlyUseColorModality),
	videoWidth(in_camParams.videoWidth),
	videoHeight(in_camParams.videoHeight),
	cx(in_camParams.cx),
	cy(in_camParams.cy),
	fx(in_camParams.fx),
	fy(in_camParams.fy),
	fieldOfViewHeight(360.0f / CV_PI * atanf(videoHeight / (2 * in_camParams.fy))),
	lowerAngleStop(in_templateSettings.angleStart),
	upperAngleStop(in_templateSettings.angleStop),
	angleStep(in_templateSettings.angleStep),
	stepSize(in_templateSettings.stepSize),
	modelFolder(in_templateSettings.modelFolder),
	detectorThreshold(in_templateSettings.detectorThreshold),
	percentToPassCheck(in_templateSettings.percentToPassCheck),
	numberWantedPoses(in_templateSettings.numberWantedPoses),
	radiusThresholdNewObject(in_templateSettings.radiusThresholdNewObject),
	discardGroupRatio(in_templateSettings.discardGroupRatio),
	useDepthImprovement(in_templateSettings.useDepthImprovement),
	depthOffset(in_templateSettings.depthOffset)
{
	if (!onlyColorModality)
	{
		std::vector<cv::Ptr<cv::linemod::Modality>> modality;
		modality.emplace_back(cv::makePtr<cv::linemod::ColorGradient>());
		modality.emplace_back(cv::makePtr<cv::linemod::DepthNormal>());

		static const int T_DEFAULTS[] = { 5, 8 };
		detector = cv::makePtr<cv::linemod::Detector>(modality, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
	}
	else
	{
		std::vector<cv::Ptr<cv::linemod::Modality>> modality;
		modality.emplace_back(cv::makePtr<cv::linemod::ColorGradient>());
		static const int T_DEFAULTS[] = { 2, 8 };
		detector = cv::makePtr<cv::linemod::Detector>(modality, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
	}

	generateRotMatForInplaneRotation();
}

HighLevelLineMOD::~HighLevelLineMOD()
{
	detector.release();
}

/**
 * @brief Class Id getter
 * 
 * @return std::vector<cv::String>
 */
std::vector<cv::String> HighLevelLineMOD::getClassIds()
{
	return detector->classIds();
}

/**
 * @brief Number of classes getter
 * 
 * @return uint16_t 
 */
uint16_t HighLevelLineMOD::getNumClasses()
{
	return detector->numClasses();
}

/**
 * @brief Number of Templates getter
 * 
 * @return uint32_t 
 */
uint32_t HighLevelLineMOD::getNumTemplates()
{
	return detector->numTemplates();
}


/**
 * @brief Adding a template from input images
 * 
 * @param in_images 
 * @param in_modelName 
 * @param in_cameraPosition 
 * @return true Templates correctly extracted from image
 * @return false Returns false if the templates cant be created. Usually its because they are too small
 */
bool HighLevelLineMOD::addTemplate(std::vector<cv::Mat>& in_images, const std::string& in_modelName,
	glm::vec3 in_cameraPosition)
{
	cv::Mat mask;
	cv::Mat maskRotated;
	std::vector<cv::Mat> templateImgs;
	cv::Mat colorRotated;
	cv::Mat depthRotated;
	cv::Mat colorToBinary;
	cv::threshold(in_images[0], colorToBinary, 1, 255, cv::THRESH_BINARY);
	cv::threshold(in_images[1], mask, 1, 65535, cv::THRESH_BINARY);
	mask.convertTo(mask, CV_8UC1);

	for (size_t q = 0; q < inPlaneRotationMat.size(); q++)
	{
		cv::warpAffine(mask, maskRotated, inPlaneRotationMat[q], maskRotated.size());
		cv::warpAffine(colorToBinary, colorRotated, inPlaneRotationMat[q], colorToBinary.size());
		cv::warpAffine(in_images[1], depthRotated, inPlaneRotationMat[q], in_images[1].size());
		templateImgs.push_back(colorRotated);
		if (!onlyColorModality)
		{
			templateImgs.push_back(depthRotated);
		}
		erodeMask(maskRotated, maskRotated, 1);
		cv::Rect boundingBox;
		uint64_t template_id = detector->addTemplate(templateImgs, in_modelName, maskRotated, &boundingBox);
		templateImgs.clear();

		if (template_id == -1)
		{
			std::cout << "ERROR::Cant create Template" << std::endl;
			return false;
		}
		glm::vec3 translation;
		glm::qua<float> quaternions;
		uint16_t medianDepth = medianMat(depthRotated, boundingBox, 5);
		int16_t currentInplaneAngle = -(lowerAngleStop + q * angleStep);
		calculateTemplatePose(translation, quaternions, in_cameraPosition, currentInplaneAngle);
		templates.emplace_back(translation, quaternions, boundingBox, medianDepth);
	}
	return true;
}

/**
 * @brief Calculate a rough mask by estimating the convex hull of features
 * 
 * @param in_match 
 * @param[out] dst image with the mask 
 */
void HighLevelLineMOD::templateMask(cv::linemod::Match const& in_match, cv::Mat& dst)
{
	const std::vector<cv::linemod::Template>& templates = detector->getTemplates(
		in_match.class_id, in_match.template_id);
	cv::Point offset(in_match.x, in_match.y);
	std::vector<cv::Point> points;
	uint16_t num_modalities = detector->getModalities().size();
	for (int m = 0; m < num_modalities; ++m)
	{
		for (cv::linemod::Feature f : templates[m].features)
		{
			points.push_back(cv::Point(f.x, f.y) + offset);
		}
	}

	std::vector<cv::Point> hull;
	convexHull(points, hull);

	dst = cv::Mat::zeros(cv::Size(videoWidth, videoHeight), CV_8U);
	const auto hull_count = (int)hull.size();
	const cv::Point* hull_pts = &hull[0];
	fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
}

/**
 * @brief Detect templates in the given images with the given class number
 * 
 * @param in_imgs 
 * @param in_classNumber 
 * @return true 
 * @return false could not find a template
 */
bool HighLevelLineMOD::detectTemplate(std::vector<cv::Mat>& in_imgs, uint16_t in_classNumber)
{
	templates = modelTemplates[in_classNumber];
	posesMultipleObj.clear();
	cv::Mat tmpDepth;
	bool depthCheckForColorDetector = false;

	const std::vector<std::string> currentClass(1, detector->classIds()[in_classNumber]);
	if (onlyColorModality && in_imgs.size() == 2)
	{
		tmpDepth = in_imgs[1];
		in_imgs.pop_back();
		depthCheckForColorDetector = true;
	}
	detector->match(in_imgs, detectorThreshold, matches, currentClass);
	if (depthCheckForColorDetector)
	{
		in_imgs.push_back(tmpDepth);
	}
	if (!matches.empty())
	{
		cvtColor(in_imgs[0], colorImgHue, cv::COLOR_BGR2HSV);
		inRange(colorImgHue, modProps[in_classNumber].lowerColorRange, modProps[in_classNumber].upperColorRange, colorImgHue);

		groupSimilarMatches();
		discardSmallMatchGroups();
		for (auto& potentialMatch : potentialMatches)
		{
			groupedMatches = elementsFromListOfIndices(matches, potentialMatch.matchIndices);
			std::vector<ObjectPose> objPoses;
			applyPostProcessing(in_imgs, objPoses);
			if (!objPoses.empty())
			{
				posesMultipleObj.push_back(objPoses);
			}
		}

		//DRAW FEATURES OF BEST LINEMOD MATCH
		if (!posesMultipleObj.empty())
		{
			for (auto& potentialMatch : potentialMatches)
			{
				const std::vector<cv::linemod::Template>& templates = detector->getTemplates(
					matches[potentialMatch.matchIndices[0]].class_id,
					matches[potentialMatch.matchIndices[0]].template_id);
				drawResponse(templates, 1, in_imgs[0], potentialMatch.position, detector->getT(0));
			}
		}
		return true;
	}
	return false;
}


/**
 * @brief Function to pick out elements from vector with a list of indices
 * 
 * @param in_matches Vector to pick elements from
 * @param in_indices Vector of indices to pick from other vector
 * @return std::vector<cv::linemod::Match> 
 */
std::vector<cv::linemod::Match> HighLevelLineMOD::elementsFromListOfIndices(
	std::vector<cv::linemod::Match>& in_matches, const std::vector<uint32_t>& in_indices)
{
	std::vector<cv::linemod::Match> tmpMatches;
	tmpMatches.reserve(in_indices.size());
	for (unsigned int in_indice : in_indices)
	{
		tmpMatches.push_back(in_matches[in_indice]);
	}
	return tmpMatches;
}


/**
 * @brief Sort matches with a similar position in the image into groups
 * 
 */
void HighLevelLineMOD::groupSimilarMatches()
{
	potentialMatches.clear();
	for (size_t i = 0; i < matches.size(); i++)
	{
		cv::Point matchPosition = cv::Point(matches[i].x, matches[i].y);
		uint16_t numCurrentGroups = potentialMatches.size();
		bool foundGroupForMatch = false;

		for (size_t q = 0; q < numCurrentGroups; q++)
		{
			if (cv::norm(matchPosition - potentialMatches[q].position) < radiusThresholdNewObject)
			{
				potentialMatches[q].matchIndices.push_back(i);
				foundGroupForMatch = true;
				break;
			}
		}
		if (!foundGroupForMatch)
		{
			potentialMatches.emplace_back(cv::Point(matches[i].x, matches[i].y), i);
		}
	}
}


/**
 * @brief Remove groups from the sorted list of matches with a low percentage of elements
 * 
 */
void HighLevelLineMOD::discardSmallMatchGroups()
{
	uint32_t numMatchGroups = potentialMatches.size();
	uint32_t biggestGroup = 0;
	std::vector<PotentialMatch> tmp;
	for (size_t i = 0; i < numMatchGroups; i++)
	{
		if (potentialMatches[i].matchIndices.size() > biggestGroup)
		{
			biggestGroup = potentialMatches[i].matchIndices.size();
		}
	}
	for (size_t j = 0; j < numMatchGroups; j++)
	{
		float ratioToBiggestGroup = potentialMatches[j].matchIndices.size() * 100 / biggestGroup;
		if (ratioToBiggestGroup > discardGroupRatio)
		{
			//TODO Non magic number
			tmp.push_back(potentialMatches[j]);
		}
	}
	potentialMatches.swap(tmp);
}


/**
 * @brief Write the detecor and templates to a file called "linemod_templates.yml.gz" and "linemod_tempPosFile.bin"
 * 
 */
void HighLevelLineMOD::writeLinemod()
{
	std::string filename = "linemod_templates.yml.gz";
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	detector->write(fs);

	std::vector<cv::String> ids = detector->classIds();
	fs << "classes" << "[";
	for (const auto& id : ids)
	{
		fs << "{";
		detector->writeClass(id, fs);
		fs << "}";
	}
	fs << "]";

	std::ofstream templatePositionFile("linemod_tempPosFile.bin", std::ios::binary | std::ios::out);
	uint32_t numTempVecs = modelTemplates.size();
	templatePositionFile.write((char*)&numTempVecs, sizeof(uint32_t));
	for (const auto& modelTemplate : modelTemplates)
	{
		uint64_t numTemp = modelTemplate.size();
		templatePositionFile.write((char*)&numTemp, sizeof(uint64_t));
		for (uint64_t i = 0; i < numTemp; i++)
		{
			templatePositionFile.write((char *)&modelTemplate[i], sizeof(Template));
		}
	}
	templatePositionFile.close();
}


/**
 * @brief Reading the templates and the detector from the written files
 * 
 */
void HighLevelLineMOD::readLinemod()
{
	templates.clear();
	modelTemplates.clear();
	std::string filename = "linemod_templates.yml.gz";
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	detector->read(fs.root());

	cv::FileNode fn = fs["classes"];
	for (auto&& i : fn)
	{
		detector->readClass(i);
	}

	std::ifstream input = std::ifstream("linemod_tempPosFile.bin", std::ios::in | std::ios::binary);
	if (!input.is_open())
	{
		//TODO raise error
	}
	uint32_t numTempVecs;
	input.read((char*)&numTempVecs, sizeof(uint32_t));
	for (uint32_t numTempVec = 0; numTempVec < numTempVecs; numTempVec++)
	{
		uint64_t numTemp;
		input.read((char*)&numTemp, sizeof(uint64_t));
		Template tp;
		for (uint64_t i = 0; i < numTemp; i++)
		{
			input.read((char*)&tp, sizeof(Template));
			templates.push_back(tp);
		}
		modelTemplates.push_back(templates);
		templates.clear();
	}
	input.close();
	readColorRanges();
}

/**
 * @brief Reading the Object Poses after detection
 * 
 * @return std::vector<std::vector<ObjectPose>> 
 */
std::vector<std::vector<ObjectPose>> HighLevelLineMOD::getObjectPoses()
{
	return posesMultipleObj;
}

 /**
  * @brief Generation the matrices for image rotation
  * 
  */
void HighLevelLineMOD::generateRotMatForInplaneRotation()
{
	for (int16_t angle = lowerAngleStop; angle <= upperAngleStop; angle = angle + angleStep)
	{
		cv::Point2f srcCenter(videoWidth / 2, videoHeight / 2);
		inPlaneRotationMat.push_back(getRotationMatrix2D(srcCenter, angle, 1.0f));
	}
}

/**
 * @brief Calculate the median or quartile of an opencv matrice
 * 
 * @param in_mat The matrice to calculate
 * @param in_bb The bounding box describing the area to calculate the medain
 * @param in_medianPosition The wanted position of the sorted values. For median this should be 2. For lower quartile it is 4.
 * @return uint16_t 
 */
uint16_t HighLevelLineMOD::medianMat(cv::Mat const& in_mat, cv::Rect& in_bb, uint8_t in_medianPosition)
{
	cv::Mat invBinRot; //Turn depth values 0 into 65535
	threshold(in_mat, invBinRot, 1, 65535, cv::THRESH_BINARY);
	invBinRot = 65535 - invBinRot;
	cv::Mat croppedDepth = in_mat + invBinRot;
	croppedDepth = croppedDepth(in_bb);

	std::vector<uint16_t> vecFromMat(croppedDepth.begin<uint16_t>(), croppedDepth.end<uint16_t>());
	std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 4, vecFromMat.end());
	return vecFromMat[vecFromMat.size() / in_medianPosition];
}

/**
 * @brief Calculation position and rotation from the camera position and the in plane rotation angle
 * 
 * @param[out] in_translation 
 * @param[out] in_quats 
 * @param in_cameraPosition 
 * @param in_inplaneRot 
 */
void HighLevelLineMOD::calculateTemplatePose(glm::vec3& in_translation, glm::qua<float>& in_quats,
	glm::vec3& in_cameraPosition, int16_t& in_inplaneRot)
{
	in_translation.x = 0.0f;
	in_translation.y = 0.0f;
	in_translation.z = glm::length(in_cameraPosition);

	if (in_cameraPosition[0] == 0 && in_cameraPosition[2] == 0)
	{
		//Looking straight up or down fails the cross product
		in_cameraPosition[0] = 0.00000000001;
	}
	glm::vec3 camUp = normalize(cross(in_cameraPosition, cross(in_cameraPosition, up)));
	glm::vec3 rotatedUp = rotate(-camUp, glm::radians((float)in_inplaneRot), normalize(in_cameraPosition));
	glm::mat4 view = lookAt(in_cameraPosition, glm::vec3(0.0f), rotatedUp);
	in_quats = openglCoordinatesystem2opencv(view);
}


/**
 * @brief The function converts the rotation from the opengl coordinate system to the opencv one
 * 
 * @param in_viewMat 
 * @return glm::qua<float> 
 */
glm::qua<float> HighLevelLineMOD::openglCoordinatesystem2opencv(glm::mat4& in_viewMat)
{
	glm::mat4 coordinateTransform(1.0f);
	coordinateTransform[1][1] = -1.0f;
	coordinateTransform[2][2] = -1.0f;
	glm::qua<float> tempQuat = toQuat(glm::transpose(glm::transpose(in_viewMat)*coordinateTransform));
	return tempQuat;
}

/**
 * @brief Function that applies color and depth checks to the matches until a set number of matches pass
 * 
 * @param in_imgs 
 * @param[out] in_objPoses 
 * @return true True means that atleast one match passed the check
 * @return false False means that no match passed the checks 
 */
bool HighLevelLineMOD::applyPostProcessing(std::vector<cv::Mat>& in_imgs, std::vector<ObjectPose>& in_objPoses)
{
	for (uint32_t i = 0; i < groupedMatches.size(); i++)
	{
		if (!onlyColorModality)
		{
			if (colorCheck(colorImgHue, i, percentToPassCheck) && depthCheck(in_imgs[1], i))
			{
				updateTranslationAndCreateObjectPose(i, in_objPoses);
			}
		}
		else if (onlyColorModality && in_imgs.size() == 2)
		{
			if (colorCheck(colorImgHue, i, percentToPassCheck) && depthCheck(in_imgs[1], i))
			{
				updateTranslationAndCreateObjectPose(i, in_objPoses);
			}
		}
		else
		{
			if (colorCheck(colorImgHue, i, percentToPassCheck))
			{
				updateTranslationAndCreateObjectPose(i, in_objPoses);
			}
		}
		if (in_objPoses.size() == numberWantedPoses)
		{
			return true;
		}
		if (i == (groupedMatches.size() - 1))
		{
			if (!in_objPoses.empty())
			{
				return true;
			}
		}
	}
	return false;
}

/**
 * @brief Test if the color of a match is correct
 * 
 * @param in_colImg Binary image. White pixel are the correct color
 * @param in_numMatch Index of the match in the matches vector
 * @param in_percentToPassCheck How many percent of the pixel have to be correct
 * @return true 
 * @return false 
 */
bool HighLevelLineMOD::colorCheck(cv::Mat& in_colImg, uint32_t& in_numMatch, float in_percentToPassCheck)
{
	cv::Mat croppedImage;
	cv::Mat mask;
	templateMask(groupedMatches[in_numMatch], mask);
	bitwise_and(in_colImg, mask, croppedImage);

	float nonZer = countNonZero(croppedImage) * 100 / countNonZero(mask);
	return nonZer > in_percentToPassCheck;
}

/**
 * @brief Checking if the depth is as expected. The difference between the expected and the match depth is then adjusted
 * 
 * @param in_depth Depth image
 * @param in_numMatch Index of the match in the matches vector
 * @return true 
 * @return false 
 */
bool HighLevelLineMOD::depthCheck(cv::Mat& in_depth, uint32_t& in_numMatch)
{
	if (useDepthImprovement)
	{

		cv::Rect bb(
			groupedMatches[in_numMatch].x,
			groupedMatches[in_numMatch].y,
			templates[groupedMatches[in_numMatch].template_id].boundingBox.width,
			templates[groupedMatches[in_numMatch].template_id].boundingBox.height);
		int32_t depthDiff = (int32_t)medianMat(in_depth, bb, 5) - (int32_t)templates[groupedMatches[in_numMatch].template_id].
			medianDepth - depthOffset;
		tempDepth = templates[groupedMatches[in_numMatch].template_id].translation.z + depthDiff;
		return abs(depthDiff) < stepSize;
	}
	else {
		tempDepth = templates[groupedMatches[in_numMatch].template_id].translation.z;
		return true;
	}

}

/**
 * @brief Update the translation and rotation of a match depending on its 2D position and place it in the pose vector
 * 
 * @param in_numMatch Index of the match in the matches vector
 * @param[out] in_objPoses 
 */
void HighLevelLineMOD::updateTranslationAndCreateObjectPose(uint32_t const& in_numMatch,
	std::vector<ObjectPose>& in_objPoses)
{
	glm::vec3 updatedTanslation;
	glm::qua<float> updatedRotation;
	calcPosition(in_numMatch, updatedTanslation, tempDepth);
	calcRotation(in_numMatch, updatedTanslation, updatedRotation);
	cv::Rect boundingBox(
		groupedMatches[in_numMatch].x,
		groupedMatches[in_numMatch].y,
		templates[groupedMatches[in_numMatch].template_id].boundingBox.width,
		templates[groupedMatches[in_numMatch].template_id].boundingBox.height);
	in_objPoses.emplace_back(updatedTanslation, updatedRotation, boundingBox);
}

/**
 * @brief Calculate the translation 
 * 
 * @param in_numMatch Index of the match in the matches vector
 * @param[out] in_position translation of the object
 * @param in_directDepth depth check adjusted distance between camera and object
 */
void HighLevelLineMOD::calcPosition(uint32_t const& in_numMatch, glm::vec3& in_position, float const& in_directDepth)
{
	float pixelX, pixelY;
	matchToPixelCoord(in_numMatch, pixelX, pixelY);
	float offsetFromCenter = pixelDistToCenter(pixelX, pixelY);
	in_position.z = calcTrueZ(in_directDepth, offsetFromCenter);

	float mmOffsetFromCenter = in_position.z / fy;
	in_position.x = (pixelX - videoWidth / 2) * mmOffsetFromCenter;
	in_position.y = (pixelY - videoHeight / 2) * mmOffsetFromCenter;
}

/**
 * @brief Calculate the adjusted roatation in quaternions of the match
 * 
 * @param in_numMatch Index of the match in the matches vector
 * @param in_position Updated translation vector
 * @param[out] in_quats Quaternions of the object
 */
void HighLevelLineMOD::calcRotation(uint32_t const& in_numMatch, glm::vec3 const& in_position,
	glm::qua<float>& in_quats)
{
	glm::mat4 adjustRotation = lookAt(glm::vec3(-in_position.x, -in_position.y, in_position.z), glm::vec3(0.f), up);
	in_quats = toQuat(adjustRotation * toMat4(templates[groupedMatches[in_numMatch].template_id].quaternions));
}

/**
 * @brief Calculate the match origin position in pixel
 * 
 * @param in_numMatch 
 * @param[out] in_x 
 * @param[out] in_y 
 */
void HighLevelLineMOD::matchToPixelCoord(uint32_t const& in_numMatch, float& in_x, float& in_y)
{
	in_x = (groupedMatches[in_numMatch].x + videoWidth / 2 - templates[groupedMatches[in_numMatch].template_id].boundingBox.x);
	in_y = (groupedMatches[in_numMatch].y + videoHeight / 2 - templates[groupedMatches[in_numMatch].template_id].boundingBox.y);
}

/**
 * @brief Calculate the pixel distance from origin to image center
 * 
 * @param in_x 
 * @param in_y 
 * @return float 
 */
float HighLevelLineMOD::pixelDistToCenter(float in_x, float in_y)
{
	in_x -= videoWidth / 2;
	in_y -= videoHeight / 2;
	return sqrt(in_x * in_x + in_y * in_y);
}

/**
 * @brief Calculate the z-position of the object out of the direct distance between object and camera
 * 
 * @param in_directDist 
 * @param in_distFromCenter 
 * @return float 
 */
float HighLevelLineMOD::calcTrueZ(float const& in_directDist, float const& in_distFromCenter)
{
	return sqrt(in_directDist*in_directDist - (in_distFromCenter*in_distFromCenter));
}

/**
 * @brief Add the templates of the current class to a vector of templates
 * 
 */
void HighLevelLineMOD::pushBackTemplates()
{
	modelTemplates.push_back(templates);
	templates.clear();
}

/**
 * @brief Read the object color for the color check from corresponding file
 * 
 */
void HighLevelLineMOD::readColorRanges()
{
	modProps.clear();
	modelFiles = detector->classIds();
	for (uint16_t i = 0; i < detector->numClasses(); i++)
	{
		ModelProperties tmpModProp;
		std::string filename = modelFolder + modelFiles[i].substr(0, modelFiles[i].size() - 4) + ".yml";
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		cv::Vec3b tempVec;

		fs["lower color range"] >> tmpModProp.lowerColorRange;
		fs["upper color range"] >> tmpModProp.upperColorRange;
		fs["has rotational symmetry"] >> tmpModProp.rotationallySymmetrical;
		fs["planes of symmetry"] >> tempVec;
		tmpModProp.planesOfSymmetry = glm::vec3(tempVec[0], tempVec[1], tempVec[2]);

		modProps.emplace_back(tmpModProp);
	}
}