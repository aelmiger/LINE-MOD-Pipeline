#pragma once
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

class Kinect2
{
public:

	Kinect2();
	~Kinect2();
	void getKinectFrames(cv::Mat& in_rgb, cv::Mat& in_depth);
private:
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	libfreenect2::Registration* registration;
	libfreenect2::FrameMap frames;
	cv::Mat rgbmat, depthmat, depthmatUndistorted, rgbd, rgbd2, croppedBgr, croppedDepth;
	libfreenect2::SyncMultiFrameListener* listener;
};
