#pragma once
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <glm/gtx/string_cast.hpp>

#include "defines.h"
#include "Kinect2.h"
#include "utility.h"

/**
 * @brief Class can create a benchmark dataset
 * 
 */
class Aruco
{
public:
	/**
	 * @brief Construct a new Aruco object
	 * 
	 */
	Aruco();
	~Aruco();

	/**
	 * @brief Generates the aruco board image "aruco_board.jp"
	 * 
	 */
	void createArucoBoard();

	/**
	 * @brief Uses the Kinect V2 camera to detect the aruco board and save images
	 * @detail Pressing TAB will save RGB and Depth images to the "benchmark/" folder
	 * with a corresponding YAML file containing the board pose with the origin at the center
	 * 
	 */
	void detectBoard();

private:
	const int markersX = 5;
	const int markersY = 7;
	const int markerLength = 120;
	const int markerSeparation = 20;

	const int margins = markerSeparation;
	const int dictionaryId = cv::aruco::DICT_6X6_100;
};
