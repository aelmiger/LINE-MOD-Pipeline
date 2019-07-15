#include "Aruco.h"

Aruco::Aruco()
{
}

Aruco::~Aruco()
{
}

void Aruco::createArucoBoard()
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

void Aruco::detectBoard()
{
	cv::Mat depthImg;
	Kinect2 kin2;
	CameraParameters camParam;
	TemplateGenerationSettings tempSett;
	readSettings(camParam, tempSett);
	cv::Mat cameraMatrix = camParam.cameraMatrix;
	cv::Mat distCoeffs = camParam.distortionCoefficients;
	cv::Ptr<cv::aruco::Dictionary> dictionary = getPredefinedDictionary(
		cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, float(markerLength),
		float(markerSeparation), dictionary);
	uint16_t counter = 0;
	while (true)
	{
		cv::Mat image, imageCopy;
		kin2.getKinectFrames(image, depthImg);
		image.copyTo(imageCopy);
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
		detectMarkers(image, dictionary, corners, ids, cv::aruco::DetectorParameters::create(), rejectedCandidates);
		refineDetectedMarkers(image, board, corners, ids, rejectedCandidates);
		cv::Matx33d rotMat;
		cv::Vec3d rvec, tvec;


		// if at least one marker detected
		if (!ids.empty())
		{
			//cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
			int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);
			Rodrigues(rvec, rotMat);
			tvec *= 0.283f;
			tvec += rotMat * cv::Vec3d(96, 136, 0);
			// if at least one board marker detected
			if (valid > 0)
				cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 100);
		}

		imshow("out", imageCopy);
		auto key = (char)cv::waitKey(1);
		if (key == 9) {
			std::string picEnding = ".png";
			cv::imwrite("benchmark/img"+std::to_string(counter)+picEnding, image);
			cv::imwrite("benchmark/depth" + std::to_string(counter) + picEnding, depthImg);

			std::string filename = "benchmark/pose"+ std::to_string(counter);
			std::string fileEnding = ".yml";
			cv::FileStorage fs(filename+fileEnding, cv::FileStorage::WRITE);
			fs << "rotMat" << rotMat;
			fs << "position" << tvec;
			counter++;
		}
		if (key == 27)
			break;
	}
}

