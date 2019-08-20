#pragma once
//#include <iostream>
//#include <cstdio>
//#include <iomanip>
//#include <ctime>
//#include <csignal>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

/**
 * @brief Class that wraps the communication with the Kinect V2 in a high level interface
 * 
 */
class Kinect2
{
public:
	/**
	 * @brief Construct a new Kinect 2 object
	 * 
	 */
	Kinect2();
	~Kinect2();
	/**
	 * @brief Get 640x480 aligned RGB (8bit) and depth(16bit) images from the camera
	 * 
	 * @param[out] in_rgb 
	 * @param[out] in_depth 
	 */
	void getKinectFrames(cv::Mat& in_rgb, cv::Mat& in_depth);
private:
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device* dev = nullptr;
	libfreenect2::PacketPipeline* pipeline = nullptr;
	libfreenect2::Registration* registration;
	libfreenect2::FrameMap frames;
	cv::Mat rgbmat, depthmat, depthmatUndistorted, rgbd, rgbd2, croppedBgr, croppedDepth;
	libfreenect2::SyncMultiFrameListener* listener;
};
