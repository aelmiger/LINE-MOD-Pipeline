#pragma once

#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/imgproc.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <utility>

#include "defines.h"
#include "utility.h"

/**
 * @brief Class wraps template generation and detection. Builds on the opencv linemod class
 */
class HighLevelLineMOD
{
public:
	/**
	 * @brief Construct a new High Level LINE-MOD object
	 * 
	 * @param in_camParams Contains the Camera Parameters
	 * @param in_templateSettings Contains the settings for template generation and detection
	 */
	HighLevelLineMOD(CameraParameters const& in_camParams,
	                 TemplateGenerationSettings const& in_templateSettings);
	~HighLevelLineMOD();

	/**
	 * @brief Class Id getter
	 * 
	 * @return std::vector<cv::String>
	 */
	std::vector<cv::String> getClassIds();

	/**
	 * @brief Number of classes getter
	 * 
	 * @return uint16_t 
	 */
	uint16_t getNumClasses();
	/**
	 * @brief Number of Templates getter
	 * 
	 * @return uint32_t 
	 */
	uint32_t getNumTemplates();

	/**
	 * @brief Adding a template from input images
	 * 
	 * @param in_images 
	 * @param in_modelName 
	 * @param in_cameraPosition 
	 * @return true Templates correctly extracted from image
	 * @return false Returns false if the templates cant be created. Usually its because they are too small
	 */
	bool addTemplate(std::vector<cv::Mat>& in_images, const std::string& in_modelName,
	                 glm::vec3 in_cameraPosition);

	/**
	 * @brief Detect templates in the given images with the given class number
	 * 
	 * @param in_imgs 
	 * @param in_classNumber 
	 * @return true 
	 * @return false could not find a template
	 */
	bool detectTemplate(std::vector<cv::Mat>& in_imgs, uint16_t in_classNumber);

	/**
	 * @brief Write the detecor and templates to a file called "linemod_templates.yml.gz" and "linemod_tempPosFile.bin"
	 * 
	 */
	void writeLinemod();

	/**
	 * @brief Reading the templates and the detector from the written files
	 * 
	 */
	void readLinemod();

	/**
	 * @brief Add the templates of the current class to a vector of templates
	 * 
	 */
	void pushBackTemplates();

	/**
	 * @brief Reading the Object Poses after detection
	 * 
	 * @return std::vector<std::vector<ObjectPose>> 
	 */
	std::vector<std::vector<ObjectPose>> getObjectPoses();

private:
	cv::Ptr<cv::linemod::Detector> detector;

	bool onlyColorModality;

	uint16_t videoWidth;
	uint16_t videoHeight;
	float cx;
	float cy;
	float fx;
	float fy;
	float fieldOfViewHeight;

	std::vector<cv::Mat> inPlaneRotationMat;
	int16_t lowerAngleStop;
	int16_t upperAngleStop;
	uint16_t angleStep;
	uint16_t stepSize;
	float detectorThreshold;
	uint16_t percentToPassCheck;
	uint16_t numberWantedPoses;
	float radiusThresholdNewObject;
	float discardGroupRatio;
	bool useDepthImprovement;
	float depthOffset;

	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
	int32_t tempDepth;

	struct Template
	{
		Template()
		{
		}

		Template(glm::vec3 tra, glm::qua<float> qua, cv::Rect bb, uint16_t med) :
			translation(tra),
			quaternions(qua),
			boundingBox(std::move(bb)),
			medianDepth(med)
		{
		}

		glm::vec3 translation;
		glm::qua<float> quaternions;
		cv::Rect boundingBox;
		uint16_t medianDepth;
	};

	struct PotentialMatch
	{
		PotentialMatch(cv::Point in_point, size_t in_indices) :
			position(std::move(in_point))
		{
			matchIndices.push_back(in_indices);
		}

		cv::Point position;
		std::vector<uint32_t> matchIndices;
	};

	cv::Mat colorImgHue;
	std::vector<Template> templates;
	std::vector<std::vector<Template>> modelTemplates;
	std::vector<std::vector<ObjectPose>> posesMultipleObj;
	std::vector<cv::linemod::Match> matches;
	std::vector<cv::linemod::Match> groupedMatches;
	std::vector<PotentialMatch> potentialMatches;
	std::vector<ModelProperties> modProps;

	std::vector<std::string> modelFiles;
	std::string modelFolder;

	/**
	 * @brief Generation the matrices for image rotation
	 * 
 	*/
	void generateRotMatForInplaneRotation();

	/**
	 * @brief Calculate the median or quartile of an opencv matrice
	 * 
	 * @param in_mat The matrice to calculate
	 * @param in_bb The bounding box describing the area to calculate the medain
	 * @param in_medianPosition The wanted position of the sorted values. For median this should be 2. For lower quartile it is 4.
	 * @return uint16_t 
	 */
	uint16_t medianMat(cv::Mat const& in_mat, cv::Rect& in_bb, uint8_t in_medianPosition);

	/**
	 * @brief Calculation position and rotation from the camera position and the in plane rotation angle
	 * 
	 * @param[out] in_translation 
	 * @param[out] in_quats 
	 * @param in_cameraPosition 
	 * @param in_inplaneRot 
	 */
	void calculateTemplatePose(glm::vec3& in_translation, glm::qua<float>& in_quats,
	                           glm::vec3& in_cameraPosition, int16_t& in_inplaneRot);

	/**
	 * @brief The function converts the rotation from the opengl coordinate system to the opencv one
	 * 
	 * @param in_viewMat 
	 * @return glm::qua<float> 
	 */						   
	glm::qua<float> openglCoordinatesystem2opencv(glm::mat4& in_viewMat);

	/**
	 * @brief Function that applies color and depth checks to the matches until a set number of matches pass
	 * 
	 * @param in_imgs 
	 * @param[out] in_objPoses 
	 * @return true True means that atleast one match passed the check
	 * @return false False means that no match passed the checks 
	 */
	bool applyPostProcessing(std::vector<cv::Mat>& in_imgs, std::vector<ObjectPose>& in_objPoses);

	/**
	 * @brief Test if the color of a match is correct
	 * 
	 * @param in_colImg Binary image. White pixel are the correct color
	 * @param in_numMatch Index of the match in the matches vector
	 * @param in_percentToPassCheck How many percent of the pixel have to be correct
	 * @return true 
	 * @return false 
	 */
	bool colorCheck(cv::Mat& in_hueImg, uint32_t& in_numMatch, float in_percentCorrectColor);
	bool depthCheck(cv::Mat& in_depth, uint32_t& in_numMatch);

	/**
	 * @brief Update the translation and rotation of a match depending on its 2D position and place it in the pose vector
	 * 
	 * @param in_numMatch Index of the match in the matches vector
	 * @param[out] in_objPoses 
	 */
	void updateTranslationAndCreateObjectPose(uint32_t const& in_numMatch,
	                                          std::vector<ObjectPose>& in_objPoses);
	
	/**
	 * @brief Calculate the translation 
	 * 
	 * @param in_numMatch Index of the match in the matches vector
	 * @param[out] in_position translation of the object
	 * @param in_directDepth depth check adjusted distance between camera and object
	 */
	void calcPosition(uint32_t const& in_numMatch, glm::vec3& in_position,
	                  float const& in_directDepth);
	

	/**
	 * @brief Calculate the adjusted roatation in quaternions of the match
	 * 
	 * @param in_numMatch Index of the match in the matches vector
	 * @param in_position Updated translation vector
	 * @param[out] in_quats Quaternions of the object
	 */
	void calcRotation(uint32_t const& in_numMatch, glm::vec3 const& in_position,
	                  glm::qua<float>& in_quats);

	/**
	 * @brief Calculate the match origin position in pixel
	 * 
	 * @param in_numMatch 
	 * @param[out] in_x 
	 * @param[out] in_y 
	 */
	void matchToPixelCoord(uint32_t const& in_numMatch, float& in_x, float& in_y);

	/**
	 * @brief Calculate the pixel distance from origin to image center
	 * 
	 * @param in_x 
	 * @param in_y 
	 * @return float 
	 */
	float pixelDistToCenter(float in_x, float in_y);

	/**
	 * @brief Calculate the z-position of the object out of the direct distance between object and camera
	 * 
	 * @param in_directDist 
	 * @param in_distFromCenter 
	 * @return float 
	 */
	float calcTrueZ(float const& in_directDist, float const& in_angleFromCenter);

	 /**
	 * @brief Calculate a rough mask by estimating the convex hull of features
	 * 
	 * @param in_match 
	 * @param[out] dst image with the mask 
	 */
	void templateMask(cv::linemod::Match const& in_match, cv::Mat& dst);

	/**
	 * @brief Sort matches with a similar position in the image into groups
	 * 
	 */
	void groupSimilarMatches();

	/**
	 * @brief Remove groups from the sorted list of matches with a low percentage of elements
	 * 
	 */
	void discardSmallMatchGroups();

	/**
	 * @brief Function to pick out elements from vector with a list of indices
	 * 
	 * @param in_matches Vector to pick elements from
	 * @param in_indices Vector of indices to pick from other vector
	 * @return std::vector<cv::linemod::Match> 
	 */
	std::vector<cv::linemod::Match> elementsFromListOfIndices(
		std::vector<cv::linemod::Match>& in_matches,
		const std::vector<uint32_t>& in_indices);
	
	/**
	 * @brief Read the object color for the color check from corresponding file
	 * 
	 */
	void readColorRanges();

	/**
	 * @brief Draws the features of a match on the image for debuging
	 * 
	 * @param templates 
	 * @param num_modalities 
	 * @param dst
	 * @param offset 
	 * @param T 
	 */
	void drawResponse(const std::vector<cv::linemod::Template>& templates,
	                  int num_modalities, cv::Mat& dst, const cv::Point& offset, int T);
};
