#pragma once
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

#include "defines.h"
#include "Kinect2.h"

class Aruco
{
public:
	Aruco();
	~Aruco();

	void createArucoBoard()
	{
		int borderBits = 1;

		cv::Size imageSize;
		imageSize.width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins;
		imageSize.height =
			markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins;

		cv::Ptr<cv::aruco::Dictionary> dictionary =
			getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

		cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, float(markerLength),
		                                                                   float(markerSeparation), dictionary);

		cv::Mat boardImage;
		board->draw(imageSize, boardImage, margins, borderBits);

		imshow("board", boardImage);
		cv::waitKey(0);

		imwrite("aruco_board.jpg", boardImage);
	}

	void detectBoard()
	{
		cv::Mat depthImg;
		Kinect2 kin2;
		CameraParameters camParam;
		cv::Mat cameraMatrix = camParam.cameraMatrix;
		cv::Mat distCoeffs = camParam.distortionCoefficients;
		cv::Ptr<cv::aruco::Dictionary> dictionary = getPredefinedDictionary(
			cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
		cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, float(markerLength),
		                                                                   float(markerSeparation), dictionary);
		while (true)
		{
			cv::Mat image, imageCopy;
			kin2.getKinectFrames(image, depthImg);
			image.copyTo(imageCopy);
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
			detectMarkers(image, dictionary, corners, ids, cv::aruco::DetectorParameters::create(), rejectedCandidates);
			refineDetectedMarkers(image, board, corners, ids, rejectedCandidates);

			// if at least one marker detected
			if (!ids.empty())
			{
				//cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
				cv::Vec3d rvec, tvec;
				int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);
				cv::Matx33d rotMat;
				Rodrigues(rvec, rotMat);
				tvec *= 0.283f;
				tvec += rotMat * cv::Vec3d(96, 136, 0);
				// if at least one board marker detected
				if (valid > 0)
					cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 100);
			}

			imshow("out", imageCopy);
			char key = (char)cv::waitKey(1);
			if (key == 27)
				break;
		}
	}

private:
	const int markersX = 5;
	const int markersY = 7;
	const int markerLength = 120;
	const int markerSeparation = 20;

	const int margins = markerSeparation;
	const int dictionaryId = cv::aruco::DICT_6X6_100;
};
