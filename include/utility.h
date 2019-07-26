#pragma once

#include <vector>
//#include <cmath>
//#include <numeric>
#include <string>
#include <filesystem>
#include <string>
#include <iostream>
#include <fstream>
//#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
//#include <glm/gtx/rotate_vector.hpp>

#include "defines.h"
/*
#################### CONVERSION UTILITY ####################
*/

glm::qua<float> openGL2openCVRotation(glm::mat4& in_viewMat);
bool fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat);
bool fromCV2GLM(const cv::Mat& cvmat, glm::mat3* glmmat);
bool fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat);
bool fromGLM2CV(const glm::mat3& glmmat, cv::Mat* cvmat);
bool fromGLM2CV(const glm::mat3& glmmat, cv::Matx33d* in_mat);

/*
#################### FILE UTILITY ####################
*/

void filesInDirectory(std::vector<std::string>& in_filePathVector, const std::string& in_path, const std::string&
	in_extension);
std::string fileToString(const char* filename);
cv::Mat loadDepth(const std::string& a_name);
void readSettings(CameraParameters& in_camParams, TemplateGenerationSettings& in_tempGenSettings);
/*
#################### CALC UTILITY ####################
*/

float length(cv::Vec3f& in_vecA);

/*
#################### IMAGE UTILITY ####################
*/

void erodeMask(cv::Mat& in_mask, cv::Mat& in_erode, int in_numberIterations);
void drawResponse(const std::vector<cv::linemod::Template>& templates,
	int num_modalities, cv::Mat& dst, const cv::Point& offset, int T);
void drawCoordinateSystem(cv::Mat& in_srcDstImage, const cv::Mat& in_camMat, float in_coordinateSystemLength,
	ObjectPose& in_objPos);
void updatePosition(cv::Matx44d in_mat, ObjectPose& in_objPose);
cv::Mat translateImg(cv::Mat &in_img, int in_offsetx, int in_offsety);