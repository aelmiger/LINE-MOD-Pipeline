#pragma once
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <glm/gtx/string_cast.hpp>

#include "defines.h"
#include "Kinect2.h"
#include "utility.h"

class Aruco
{
public:
	Aruco();
	~Aruco();

	void createArucoBoard();
	void detectBoard();

private:
	const int markersX = 5;
	const int markersY = 7;
	const int markerLength = 120;
	const int markerSeparation = 20;

	const int margins = markerSeparation;
	const int dictionaryId = cv::aruco::DICT_6X6_100;
};
