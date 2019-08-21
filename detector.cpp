#include "PoseDetection.h"

int main()
{
	PoseDetection poseDetect;

	poseDetect.setupBenchmark("bigBearing.ply"); //Uncomment if Benchmark is not wanted

	int counter = 0;

	///////IMAGE SOURCES:
	cv::VideoCapture sequence("benchmark/img%0d.png");
	//cv::VideoCapture sequence("data/color%0d.jpg");
	//Kinect2 kin2;
	/////////////////////

	while (true)
	{
		std::vector<cv::Mat> imgs;
		cv::Mat colorImg;
		cv::Mat depthImg;

		///////IMAGE SOURCES:
		sequence >> colorImg;
		depthImg = cv::imread("benchmark/depth" + std::to_string(counter) + ".png",
		                      cv::IMREAD_ANYDEPTH);
		//Video Capture does not work with 16bit png on linux
		//depthImg = loadDepthLineModDataset("data/depth" + std::to_string(counter) + ".dpt");
		//kin2.getKinectFrames(colorImg, depthImg);
		////////////////////

		if (colorImg.empty())
		{
			std::cout << "End of Sequence" << std::endl;
			std::getchar();
		}

		imgs.push_back(colorImg);
		imgs.push_back(depthImg);

		std::vector<ObjectPose> objPose;
		poseDetect.detect(imgs, "bigBearing.ply", 2, objPose, true);

		counter++;
	}
}
