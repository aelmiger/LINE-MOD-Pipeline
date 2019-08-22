#include "PoseDetection.h"

int main()
{
	PoseDetection poseDetect;

	poseDetect.setupBenchmark("p1.ply"); //Uncomment if Benchmark is not wanted

	int counter = 0;

	///////IMAGE SOURCES:
	cv::VideoCapture sequence("benchmark/img%0d.png");
	//cv::VideoCapture sequence("benchmarkLINEMOD/color%0d.jpg");
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
		//depthImg = loadDepthLineModDataset("benchmarkLINEMOD/depth" + std::to_string(counter) + ".dpt");
		//kin2.getKinectFrames(colorImg, depthImg);
		////////////////////

		if (colorImg.empty())
		{
			std::cout << "End of Sequence" << std::endl;
			cv::waitKey(0);
		}

		imgs.push_back(colorImg);
		imgs.push_back(depthImg);

		std::vector<ObjectPose> objPose;
		poseDetect.detect(imgs, "p1.ply", 1, objPose, true);

		counter++;
	}
}
