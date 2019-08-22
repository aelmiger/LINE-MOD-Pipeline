#pragma once

#include <opencv2/core.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iterator>

#include "OpenglRender.h"
#include "HighLevelLinemod.h"
#include "HighLevelLinemodIcp.h"
#include "Kinect2.h"
#include "utility.h"
#include "Benchmark.h"

/**
 * @brief High level class for the detection process
 * 
 */
class PoseDetection
{
public:
	/**
	 * @brief Construct a new Pose Detection object
	 * 
	 */
	PoseDetection();
	~PoseDetection();
	/**
	 * @brief Function to cleanup the used classes
	 * 
	 */
	void cleanup();

	/**
	 * @brief Main funtion to detect an object in given pictures
	 * 
	 * @param in_imgs Input images either RGB or RGB and Depth
	 * @param in_className Name of the model to detect in the images
	 * @param in_numberOfObjects How many objects are in the image
	 * @param[out] in_objPose The detected Pose/Poses
	 * @param in_displayResults Display the RGB image with the detected poses?
	 */
	void detect(std::vector<cv::Mat>& in_imgs, std::string const& in_className,
	            uint16_t const& in_numberOfObjects, std::vector<ObjectPose>& in_objPose,
	            bool in_displayResults);

	/**
	 * @brief Call function once if benchmark is supposed to be used
	 * 
	 * @param in_className Name of the model to be benchmarked
	 */
	void setupBenchmark(std::string const& in_className);
private:
	OpenGLRender* opengl;
	HighLevelLineMOD* line;
	HighLevelLinemodIcp* icp;
	Benchmark* bench;

	std::vector<cv::Mat> inputImg;
	std::vector<std::vector<ObjectPose>> detectedPoses;
	std::vector<ObjectPose> finalObjectPoses;
	cv::Mat colorImg;
	cv::Mat depthImg;

	CameraParameters camParams;
	TemplateGenerationSettings templateSettings;
	std::vector<std::string> modelFiles;
	std::vector<cv::String> ids;

	/**
	 * @brief Reading the linemod file created by the template generator
	 * 
	 */
	void readLinemodFromFile();

	/**
	 * @brief Convert the model name to an index
	 * 
	 * @param in_stringToFind Model name
	 * @param in_vectorToLookIn Vector that holds all model names
	 * @return uint16_t Index of the element position of the model name
	 */
	uint16_t findIndexInVector(std::string const& in_stringToFind,
	                           std::vector<std::string>& in_vectorToLookIn);

	/**
	 * @brief Draw the coordinate system of a pose onto the image
	 * 
	 * @param[in/out] in_srcDstImage image to write to 
	 * @param in_camMat Camera matrix
	 * @param in_coordinateSystemLength Length of the coordinate system in mm
	 * @param in_objPos Pose to draw
	 */
	void drawCoordinateSystem(cv::Mat& in_srcDstImage, const cv::Mat& in_camMat,
	                          float in_coordinateSystemLength,
	                          ObjectPose& in_objPos);
	
	/**
	 * @brief Shifting an image pixelwise to adjust for a shiftied camera principal point
	 * 
	 * @param in_img Image to shift
	 * @param in_offsetx offset cx in pixel
	 * @param in_offsety offset cy in pixel
	 * @return cv::Mat shifted image
	 */
	cv::Mat translateImg(cv::Mat& in_img, int in_offsetx, int in_offsety);
};
