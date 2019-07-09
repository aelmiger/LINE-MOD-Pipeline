#include "High_Level_Linemod.h"

HighLevelLinemod::HighLevelLinemod(CameraParameters const& in_camParams, TemplateGenerationSettings const& in_templateSettings) :
	onlyColorModality(in_templateSettings.onlyUseColorModality),
	videoWidth(in_camParams.videoWidth),
	videoHeight(in_camParams.videoHeight),
	cx(in_camParams.cx),
	cy(in_camParams.cy),
	fx(in_camParams.fx),
	fy(in_camParams.fy),
	fieldOfViewHeight(360.0f / M_PI * atanf(videoHeight / (2 * in_camParams.fy))),
	lowerAngleStop(in_templateSettings.angleStart),
	upperAngleStop(in_templateSettings.angleStop),
	angleStep(in_templateSettings.angleStep),
	stepSize(in_templateSettings.stepSize),
	modelFolder(in_templateSettings.modelFolder)
{
	if (!onlyColorModality)
	{
		std::vector<cv::Ptr<cv::linemod::Modality>> modality;
		modality.emplace_back(cv::makePtr<cv::linemod::ColorGradient>());
		//modality.push_back(cv::makePtr<cv::linemod::ColorGradient>(10.0f, 30, 55.0f));
		modality.emplace_back(cv::makePtr<cv::linemod::DepthNormal>());
		//modality.push_back(cv::makePtr<cv::linemod::DepthNormal>(2000, 50, 30, 2));

		static const int T_DEFAULTS[] = {5, 8};
		detector = cv::makePtr<cv::linemod::Detector>(modality, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
	}
	else
	{
		std::vector<cv::Ptr<cv::linemod::Modality>> modality;
		modality.emplace_back(cv::makePtr<cv::linemod::ColorGradient>());
		static const int T_DEFAULTS[] = {4, 8};
		detector = cv::makePtr<cv::linemod::Detector>(modality, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
	}

	generateRotMatForInplaneRotation();
}

HighLevelLinemod::~HighLevelLinemod()
{
	detector.release();
}

std::vector<cv::String> HighLevelLinemod::getClassIds()
{
	return detector->classIds();
}

uint16 HighLevelLinemod::getNumClasses()
{
	return detector->numClasses();
}

uint32 HighLevelLinemod::getNumTemplates()
{
	return detector->numTemplates();
}

bool HighLevelLinemod::addTemplate(std::vector<cv::Mat> in_images, const std::string& in_modelName,
                                   glm::vec3 in_cameraPosition)
{
	cv::Mat mask;
	cv::Mat maskRotated;
	std::vector<cv::Mat> templateImgs;
	cv::Mat colorRotated;
	cv::Mat depthRotated;
	cv::threshold(in_images[1], mask, 1, 65535, cv::THRESH_BINARY);
	mask.convertTo(mask, CV_8UC1);

	for (size_t q = 0; q < inPlaneRotationMat.size(); q++)
	{
		cv::warpAffine(mask, maskRotated, inPlaneRotationMat[q], maskRotated.size());
		cv::warpAffine(in_images[0], colorRotated, inPlaneRotationMat[q], in_images[0].size());
		cv::warpAffine(in_images[1], depthRotated, inPlaneRotationMat[q], in_images[1].size());
		templateImgs.push_back(colorRotated);
		if (!onlyColorModality)
		{
			templateImgs.push_back(depthRotated);
		}
		erodeMask(maskRotated, maskRotated, 1);
		cv::Rect boundingBox;
		uint64 template_id = detector->addTemplate(templateImgs, in_modelName, maskRotated, &boundingBox);
		templateImgs.clear();

		if (template_id == -1)
		{
			std::cout << "ERROR::Cant create Template" << std::endl;
			return false;
		}
		glm::vec3 translation;
		glm::qua<float32> quaternions;
		uint16 medianDepth = medianMat(depthRotated, boundingBox, 4);
		float32 currentInplaneAngle = -(lowerAngleStop + q * angleStep);
		calculateTemplatePose(translation, quaternions, in_cameraPosition, currentInplaneAngle);
		templates.emplace_back(translation, quaternions, boundingBox, medianDepth);
	}
	return true;
}

void HighLevelLinemod::templateMask(cv::linemod::Match const& in_match, cv::Mat& dst)
{
	const std::vector<cv::linemod::Template>& templates = detector->getTemplates(
		in_match.class_id, in_match.template_id);
	cv::Point offset(in_match.x, in_match.y);
	std::vector<cv::Point> points;
	uint16 num_modalities = detector->getModalities().size();
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

bool HighLevelLinemod::detectTemplate(std::vector<cv::Mat>& in_imgs, uint16 in_classNumber)
{
	//TODO generate automatic Hue Check
	templates = modelTemplates[in_classNumber];
	currentColorRange = modelColors[in_classNumber];
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
	detector->match(in_imgs, 85.0f, matches, currentClass);
	if (depthCheckForColorDetector)
	{
		in_imgs.push_back(tmpDepth);
	}
	if (!matches.empty())
	{
		cvtColor(in_imgs[0], colorImgHue, cv::COLOR_BGR2HSV);
		inRange(colorImgHue, currentColorRange.lowerBoundary, currentColorRange.upperBoundary, colorImgHue);
		//TODO RANGE

		groupSimilarMatches();
		discardSmallMatchGroups();
		for (auto& potentialMatch : potentialMatches)
		{
			groupedMatches = elementsFromListOfIndices(matches, potentialMatch.matchIndices); //too slow TODO
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

std::vector<cv::linemod::Match> HighLevelLinemod::elementsFromListOfIndices(
	std::vector<cv::linemod::Match>& in_matches, const std::vector<uint32>& in_indices)
{
	std::vector<cv::linemod::Match> tmpMatches;
	tmpMatches.reserve(in_indices.size());
	for (unsigned int in_indice : in_indices)
	{
		tmpMatches.push_back(in_matches[in_indice]);
	}
	return tmpMatches;
}

void HighLevelLinemod::groupSimilarMatches()
{
	potentialMatches.clear();
	for (size_t i = 0; i < matches.size(); i++)
	{
		cv::Point matchPosition = cv::Point(matches[i].x, matches[i].y);
		uint16 numCurrentGroups = potentialMatches.size();
		bool foundGroupForMatch = false;

		for (size_t q = 0; q < numCurrentGroups; q++)
		{
			if (norm(matchPosition - potentialMatches[q].position) < 100)
			{
				//TODO Non magic number
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

void HighLevelLinemod::discardSmallMatchGroups()
{
	uint32 numMatchGroups = potentialMatches.size();
	uint32 biggestGroup = 0;
	std::vector<PotentialMatch> tmp;
	for (size_t i = 0; i < numMatchGroups; i++)
	{
		if (potentialMatches[i].matchIndices.size() > biggestGroup)
		{
			biggestGroup = potentialMatches[i].matchIndices.size();
		}
	}
	std::vector<size_t> eraseIndices;
	for (size_t j = 0; j < numMatchGroups; j++)
	{
		float32 ratioToBiggestGroup = potentialMatches[j].matchIndices.size() * 100 / biggestGroup;
		if (ratioToBiggestGroup > 25)
		{
			//TODO Non magic number
			tmp.push_back(potentialMatches[j]);
		}
	}
	potentialMatches.swap(tmp);
}

void HighLevelLinemod::writeLinemod()
{
	std::string filename = "linemod_templates.yml";
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
	uint32 numTempVecs = modelTemplates.size();
	templatePositionFile.write((char*)&numTempVecs, sizeof(uint32));
	for (uint32 numTemplateVec = 0; numTemplateVec < numTempVecs; numTemplateVec++)
	{
		uint64 numTemp = modelTemplates[numTemplateVec].size();
		templatePositionFile.write((char*)&numTemp, sizeof(uint64));
		for (uint64 i = 0; i < numTemp; i++)
		{
			templatePositionFile.write((char *)&modelTemplates[numTemplateVec][i], sizeof(Template));
		}
	}
	templatePositionFile.close();
}

void HighLevelLinemod::readLinemod()
{
	templates.clear();
	modelTemplates.clear();
	std::string filename = "linemod_templates.yml";
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
	uint32 numTempVecs;
	input.read((char*)&numTempVecs, sizeof(uint32));
	for (uint32 numTempVec = 0; numTempVec < numTempVecs; numTempVec++)
	{
		uint64 numTemp;
		input.read((char*)&numTemp, sizeof(uint64));
		Template tp;
		for (uint64 i = 0; i < numTemp; i++)
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

std::vector<std::vector<ObjectPose>> HighLevelLinemod::getObjectPoses()
{
	return posesMultipleObj;
}

void HighLevelLinemod::generateRotMatForInplaneRotation()
{
	for (uint16 angle = lowerAngleStop; angle <= upperAngleStop; angle = angle + angleStep)
	{
		cv::Point2f srcCenter(videoWidth / 2, videoHeight / 2);
		inPlaneRotationMat.push_back(getRotationMatrix2D(srcCenter, angle, 1.0f));
	}
}

uint16 HighLevelLinemod::medianMat(cv::Mat const& in_mat, cv::Rect& in_bb, uint8 in_medianPosition)
{
	cv::Mat invBinRot; //Turn depth values 0 into 65535
	threshold(in_mat, invBinRot, 1, 65535, cv::THRESH_BINARY);
	invBinRot = 65535 - invBinRot;
	cv::Mat croppedDepth = in_mat + invBinRot;
	croppedDepth = croppedDepth(in_bb);

	std::vector<uint16> vecFromMat(croppedDepth.begin<uint16>(), croppedDepth.end<uint16>());
	std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 4, vecFromMat.end());
	return vecFromMat[vecFromMat.size() / in_medianPosition];
}

void HighLevelLinemod::calculateTemplatePose(glm::vec3& in_translation, glm::qua<float32>& in_quats,
                                             glm::vec3& in_cameraPosition, float32& in_inplaneRot)
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
	glm::vec3 rotatedUp = rotate(-camUp, glm::radians(in_inplaneRot), normalize(in_cameraPosition));
	glm::mat4 view = lookAt(in_cameraPosition, glm::vec3(0.f), rotatedUp);
	in_quats = openglCoordinatesystem2opencv(view);
}

glm::qua<float32> HighLevelLinemod::openglCoordinatesystem2opencv(glm::mat4& in_viewMat)
{
	glm::qua<float32> tempQuat = toQuat(in_viewMat);
	glm::vec3 eul = eulerAngles(tempQuat);
	return glm::qua(glm::vec3(eul.x - M_PI / 2.0f, -eul.y, -eul.z));
}

bool HighLevelLinemod::applyPostProcessing(std::vector<cv::Mat>& in_imgs, std::vector<ObjectPose>& in_objPoses)
{
	for (uint32 i = 0; i < groupedMatches.size(); i++)
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

bool HighLevelLinemod::colorCheck(cv::Mat& in_colImg, uint32& in_numMatch, float32 in_percentToPassCheck)
{
	cv::Mat croppedImage;
	cv::Mat mask;
	templateMask(groupedMatches[in_numMatch], mask);
	bitwise_and(in_colImg, mask, croppedImage);

	float32 nonZer = countNonZero(croppedImage) * 100 / countNonZero(mask);
	return nonZer > in_percentToPassCheck;
}

bool HighLevelLinemod::depthCheck(cv::Mat& in_depth, uint32& in_numMatch)
{
	cv::Rect bb(
		groupedMatches[in_numMatch].x,
		groupedMatches[in_numMatch].y,
		templates[groupedMatches[in_numMatch].template_id].boundingBox.width,
		templates[groupedMatches[in_numMatch].template_id].boundingBox.height);
	int32 depthDiff = (int32)medianMat(in_depth, bb, 4) - (int32)templates[groupedMatches[in_numMatch].template_id].
		medianDepth;
	tempDepth = templates[groupedMatches[in_numMatch].template_id].translation.z + depthDiff;
	return abs(depthDiff) < stepSize;
}

void HighLevelLinemod::updateTranslationAndCreateObjectPose(uint32 const& in_numMatch,
                                                            std::vector<ObjectPose>& in_objPoses)
{
	glm::vec3 updatedTanslation;
	glm::qua<float32> updatedRotation;
	calcPosition(in_numMatch, updatedTanslation, tempDepth);
	calcRotation(in_numMatch, updatedTanslation, updatedRotation);
	cv::Rect boundingBox(
		groupedMatches[in_numMatch].x,
		groupedMatches[in_numMatch].y,
		templates[groupedMatches[in_numMatch].template_id].boundingBox.width,
		templates[groupedMatches[in_numMatch].template_id].boundingBox.height);
	in_objPoses.emplace_back(updatedTanslation, updatedRotation, boundingBox);
}

void HighLevelLinemod::calcPosition(uint32 const& in_numMatch, glm::vec3& in_position, float32 const& in_directDepth)
{
	float32 pixelX, pixelY;
	matchToPixelCoord(in_numMatch, pixelX, pixelY);
	float32 offsetFromCenter = pixelDistToCenter(pixelX, pixelY);
	float32 angleFromCenter = atan(offsetFromCenter / fy);
	in_position.z = calcTrueZ(in_directDepth, angleFromCenter);

	float32 mmOffsetFromCenter = in_position.z * (offsetFromCenter / fy);
	in_position.x = (pixelX - cx) / offsetFromCenter * mmOffsetFromCenter;
	in_position.y = (pixelY - cy) / offsetFromCenter * mmOffsetFromCenter;
}

void HighLevelLinemod::calcRotation(uint32 const& in_numMatch, glm::vec3 const& in_position,
                                    glm::qua<float32>& in_quats)
{
	glm::mat4 adjustRotation = lookAt(glm::vec3(-in_position.x, -in_position.y, in_position.z), glm::vec3(0.f), up);
	in_quats = toQuat(adjustRotation * toMat4(templates[groupedMatches[in_numMatch].template_id].quaternions));
}

void HighLevelLinemod::matchToPixelCoord(uint32 const& in_numMatch, float32& in_x, float32& in_y)
{
	in_x = (groupedMatches[in_numMatch].x + cx - templates[groupedMatches[in_numMatch].template_id].boundingBox.x);
	in_y = (groupedMatches[in_numMatch].y + cy - templates[groupedMatches[in_numMatch].template_id].boundingBox.y);
}

float32 HighLevelLinemod::pixelDistToCenter(float32 in_x, float32 in_y)
{
	in_x -= cx;
	in_y -= cy;
	return sqrt(in_x * in_x + in_y * in_y);
}

float32 HighLevelLinemod::calcTrueZ(float32 const& in_directDist, float32 const& in_angleFromCenter)
{
	return cos(in_angleFromCenter) * in_directDist;
}

void HighLevelLinemod::pushBackTemplates()
{
	modelTemplates.push_back(templates);
	templates.clear();
}

void HighLevelLinemod::readColorRanges()
{
	modelColors.clear();
	modelFiles = detector->classIds();
	for (uint16 i = 0; i < detector->numClasses(); i++)
	{
		std::string file = modelFolder + modelFiles[i].substr(0, modelFiles[i].size() - 4) + ".txt";
		std::ifstream fsColor(file);
		if (!fsColor.is_open())
		{
			std::cout << "ERROR:: Cant read color info" << std::endl;
		}
		uint16 lowerHue;
		uint16 lowerSat;
		uint16 lowerLum;
		uint16 upperHue;
		uint16 upperSat;
		uint16 upperLum;

		fsColor >> lowerHue;
		fsColor >> lowerSat;
		fsColor >> lowerLum;
		fsColor >> upperHue;
		fsColor >> upperSat;
		fsColor >> upperLum;
		fsColor.close();
		modelColors.emplace_back(cv::Scalar(lowerHue, lowerSat, lowerLum), cv::Scalar(upperHue, upperSat, upperLum));
	}
}
