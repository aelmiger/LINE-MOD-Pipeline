#pragma once

#include <vector>
#include <math.h>
#include <numeric>
#include <string>
#include <filesystem>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <ppl.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>


#include "defines.h"
/*
#################### GENERAL UTILITY ####################
*/

std::vector<glm::vec3> zipVectors(const std::vector<glm::vec3> & a, const std::vector<glm::vec3> & b);
void printProgBar(int in_percent, std::string in_mfile);
typedef bool(*remove_predicate)(const cv::Mat &rc);
void remove_if(const cv::Mat &mat, cv::Mat &res, remove_predicate pred, bool removeRows = true);
bool is_zero(const cv::Mat &rc);

/*
#################### CONVERSION UTILITY ####################
*/

glm::qua<float32> openGL2openCVRotation(glm::mat4& in_viewMat);
bool fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat);
bool fromCV2GLM(const cv::Mat& cvmat, glm::mat3* glmmat);
bool fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat);
bool fromGLM2CV(const glm::mat3& glmmat, cv::Mat* cvmat);
bool fromGLM2CV(const glm::mat3& glmmat, cv::Matx33d* in_mat);


/*
#################### FILE UTILITY ####################
*/


void filesInDirectory(std::vector<std::string>& in_filePathVector, std::string in_path, std::string in_extension);
std::string fileToString(const char* filename);
static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, std::vector<TemplatePosition> &in_templatePositions);
static cv::Ptr<cv::linemod::Detector> readLinemod(std::vector<TemplatePosition> &in_templatePositions);
cv::Mat loadDepth(std::string a_name);



/*
#################### CALC UTILITY ####################
*/

uint16 medianMat(cv::Mat in_mat, cv::Rect &in_bb, uint8 in_medianPosition);
float32 length(cv::Vec3f &in_vecA);
void readGroundTruthLinemodDataset(uint32 in_fileNumber, ObjectPose &in_objectPos);
float32 matchingScoreParallel(Model &in_model, ObjectPose &in_groundTruth, ObjectPose &in_estimate);

/*
#################### IMAGE UTILITY ####################
*/

void depthToBinary(cv::Mat &in_gray, cv::Mat &in_binary, int in_threshold = 1);
void erodeMask(cv::Mat &in_mask, cv::Mat &in_erode, int in_numberIterations);
void rotateCvMat(cv::Mat &in_mat, cv::Mat &in_dstMat, cv::Mat &in_rotMat);
void drawResponse(const std::vector<cv::linemod::Template>& templates,
	int num_modalities, cv::Mat& dst, cv::Point offset, int T);
void drawCoordinateSystem(cv::Mat &in_srcDstImage, cv::Mat in_camMat, float32 in_coordinateSystemLength, ObjectPose &in_objPos);
void updatePosition(cv::Matx44d in_mat, ObjectPose &in_objPose);
