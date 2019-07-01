#pragma once

#include <vector>
//#include <iostream>
#include <string>

#define SDL_MAIN_HANDLED
#include <SDL/SDL.h>
#define GLEW_STATIC
#include <GL/glew.h>
#include <opencv2/core.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "defines.h"
#include "utility.h"
#include "model_buffer.h"
#include "Model_Importer.h"

class OpenGLRender
{
public:
	OpenGLRender(CameraParameters const& in_camParams);
	~OpenGLRender();
	cv::Mat getColorImgFromBuff();
	cv::Mat getDepthImgFromBuff();
	void renderColorToFrontBuff(uint16 in_modelIndice, glm::vec3 camPositon, float32 in_rotate = 0.0f,
	                            float32 in_x = 0.0f, float32 in_y = 0.0f);
	void renderColorToFrontBuff(uint16 in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec);
	void renderDepthToFrontBuff(uint16 in_modelIndice, glm::vec3 camPositon, float32 in_rotate = 0.0f,
	                            float32 in_x = 0.0f, float32 in_y = 0.0f);
	void renderDepthToFrontBuff(uint16 in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec);
	void creatModBuffFromFiles(std::string const& in_modelLocation);
	void readModelFile(std::string const& in_file, Model& in_model);
	void calculateMatch3DPosition(ObjectPose& in_objectPos, TemplatePosition& in_templatePosition,
	                              cv::linemod::Match& in_match);

private:
	SDL_Window* window;
	std::vector<ModelBuffer> modBuff;
	cv::Mat renderedColorImg;
	cv::Mat renderedDepthImg;
	uint16 width;
	uint16 height;
	float32 fieldOfView;
	glm::vec3 position;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 modelMat;
	glm::mat4 viewProj;
	glm::vec3 up;
	GLuint colorShaderProgram;
	GLuint depthShaderProgram;
	GLuint fbo;
	GLuint modelViewProjMatrixLocationColor;
	GLuint modelViewProjMatrixLocationDepth;
	float32 cx;
	float32 cy;

	void setupSDLWindow();
	void setupOpenGL();
	void setupFramebuffer();
	void setupShader();
	std::vector<glm::vec3> zipVectors(const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b);
	void translateCam(glm::vec3 in_vec, float32 in_rotate, float32 in_x, float32 in_y);
};
