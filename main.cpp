#include "TemplateGenerator.h"
#include "PoseDetection.h"
#include "Aruco.h"

#ifdef _DEBUG
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core410d.lib")
#pragma comment(lib, "opencv_imgproc410d.lib")
#pragma comment(lib, "opencv_rgbd410d.lib")
#pragma comment(lib, "opencv_imgcodecs410d.lib")
#pragma comment(lib, "opencv_highgui410d.lib")
#pragma comment(lib, "opencv_videoio410d.lib")
#pragma comment(lib, "opencv_ccalib410d.lib")
#pragma comment(lib, "opencv_calib3d410d.lib")
#pragma comment(lib, "opencv_surface_matching410d.lib")
#pragma comment(lib, "freenect2.lib")
#pragma comment(lib, "assimp-vc140-mt.lib")

#else
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core410.lib")
#pragma comment(lib, "opencv_imgproc410.lib")
#pragma comment(lib, "opencv_rgbd410.lib")
#pragma comment(lib, "opencv_imgcodecs410.lib")
#pragma comment(lib, "opencv_highgui410.lib")
#pragma comment(lib, "opencv_videoio410.lib")
#pragma comment(lib, "opencv_calib3d410.lib")
#pragma comment(lib, "opencv_surface_matching410.lib")
#pragma comment(lib, "opencv_aruco410.lib")
#pragma comment(lib, "freenect2.lib")
#pragma comment(lib, "assimp-vc140-mt.lib")

#endif

int main()t
{
	//Aruco ar;
	//ar.detectBoard();
	//TemplateGenerator templateGen;
	//templateGen.run();
	//TODO 2 exe in cmake!
	//templateGen.~Template_Generator();
	PoseDetection poseDetect; //TODO YML als settings datei
	poseDetect.setupBenchmark("bigBearing.ply");
	int counter = 0;
	//cv::VideoCapture sequence("data/color%0d.jpg");
	//cv::VideoCapture sequence("benchmark/img%0d.png");
	//cv::VideoCapture sequencedepth("benchmark/depth%0d.png");		
	Kinect2 kin2;

	while (true) {


		std::vector<cv::Mat> imgs;
		cv::Mat colorImg;
		cv::Mat depthImg;



		//sequence >> colorImg;
		//sequencedepth >> depthImg;

		kin2.getKinectFrames(colorImg, depthImg);

		if (colorImg.empty())
		{			std::cout << "End of Sequence" << std::endl;
		}

		//depthImg = loadDepth("data/depth" + std::to_string(counter) + ".dpt");

		imgs.push_back(colorImg);
		imgs.push_back(depthImg);
		std::vector<ObjectPose> objPose;
		poseDetect.run(imgs, "bigBearing.ply", 2, objPose); //TODO cleanup Funktion und nullptr abfrage



		counter++;
	}

	//TODO implement nach welcher klasse suchen und wieviele Objekte
	//TODO run mit funktionen austauschen!
	//TODO vector nicht per value bergeben
	//TODO doxygen
	//TODO Docker
	std::getchar();
}