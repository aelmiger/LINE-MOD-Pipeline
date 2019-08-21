#include "TemplateGenerator.h"
#include "PoseDetection.h"
#include "Aruco.h"

int main()
{
	//Aruco ar;
	//ar.detectBoard();
	TemplateGenerator templateGen;
	templateGen.run();
	//TODO 2 exe in cmake!
	PoseDetection poseDetect;
	poseDetect.setupBenchmark("bigBearing.ply");
	int counter = 0;
	//cv::VideoCapture sequence("data/color%0d.jpg");
	cv::VideoCapture sequence("benchmark/img%0d.png");
	//Kinect2 kin2;

	while (true) {

		std::vector<cv::Mat> imgs;
		cv::Mat colorImg;
		cv::Mat depthImg;

		sequence >> colorImg;
		depthImg = cv::imread("benchmark/depth" + std::to_string(counter) + ".png",cv::IMREAD_ANYDEPTH); //Video Capture does not work with 16bit png on linux
		std::cout<<depthImg.type()<<std::endl;

		//kin2.getKinectFrames(colorImg, depthImg);

		if (colorImg.empty())
		{			
			std::cout << "End of Sequence" << std::endl;
		}

		//depthImg = loadDepthLineModDataset("data/depth" + std::to_string(counter) + ".dpt");

		imgs.push_back(colorImg);
		imgs.push_back(depthImg);
		std::vector<ObjectPose> objPose;
		poseDetect.detect(imgs, "bigBearing.ply", 2, objPose);



		counter++;
	}
	//TODO run mit funktionen austauschen!
	//TODO vector nicht per value bergeben
	std::getchar();
}