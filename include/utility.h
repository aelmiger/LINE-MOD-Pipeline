#pragma once

#include <vector>
#include <string>
#include <filesystem>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "defines.h"
/*
#################### CONVERSION UTILITY ####################
*/

bool fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat);
bool fromCV2GLM(const cv::Mat& cvmat, glm::mat3* glmmat);
bool fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat);
bool fromGLM2CV(const glm::mat3& glmmat, cv::Mat* cvmat);
bool fromGLM2CV(const glm::mat3& glmmat, cv::Matx33d* in_mat);

/*
#################### FILE UTILITY ####################
*/

void filesInDirectory(std::vector<std::string>& in_filePathVector, const std::string& in_path,
                      const std::string&
                      in_extension);
cv::Mat loadDepthLineModDataset(const std::string& a_name);
void readSettings(CameraParameters& in_camParams, TemplateGenerationSettings& in_tempGenSettings);
