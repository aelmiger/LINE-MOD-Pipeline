#include "kinect2.h"

Kinect2::Kinect2()
{
	libfreenect2::setGlobalLogger(NULL);
	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "ERROR:: No kinect connected!" << std::endl;
	}
	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	//pipeline = new libfreenect2::CpuPacketPipeline(); //Necessary to prevent issues with OpenGL but slows down alot
	if (pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}
	else {
		dev = freenect2.openDevice(serial);
	}
	if (dev == 0)
	{
		std::cout << "ERROR:: opening device!" << std::endl;
	}
	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);
	dev->start();
	registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

Kinect2::~Kinect2()
{
	dev->stop();
	dev->close();
	delete registration;
	delete listener;
}

void Kinect2::getKinectFrames(cv::Mat& in_rgb, cv::Mat& in_depth) {
	listener->waitForNewFrame(frames);
	libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

	cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
	registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
	cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
	rgbd2.convertTo(rgbd2, CV_16UC1);
	rgbd2(cv::Rect(cv::Point(320 + 320, 61 + 240), cv::Point(1600 - 320, 1021 - 240))).copyTo(croppedDepth);
	rgbmat(cv::Rect(cv::Point(320 + 320, 61 + 240), cv::Point(1600 - 320, 1021 - 240))).copyTo(croppedBgr);

	cv::cvtColor(croppedBgr, croppedBgr, cv::COLOR_BGRA2BGR);
	cv::flip(croppedBgr, in_rgb, 1);
	cv::flip(croppedDepth, in_depth, 1);
	listener->release(frames);
}