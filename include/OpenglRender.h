#pragma once

#include <vector>
//#include <iostream>
#include <string>

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#define GLEW_STATIC
#include <GL/glew.h>
#include <opencv2/core.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "defines.h"
#include "utility.h"
#include "ModelBuffer.h"
#include "ModelImporter.h"

/**
 * @brief Class to render RGB and Depth images with opengl
 * 
 */
class OpenGLRender
{
public:
	/**
	 * @brief Construct a new OpenGL Render object
	 * 
	 * @param in_camParams The camera paramters from the settings file
	 */
	OpenGLRender(CameraParameters const& in_camParams);
	~OpenGLRender();

	/**
	 * @brief Read 8bit color image from front buffer to the CPU RAM and convert to opencv mat
	 * 
	 * @return cv::Mat 
	 */
	cv::Mat getColorImgFromBuff();

	/**
	 * @brief Read 16bit depth image from front buffer to the CPU RAM and convert to opencv mat
	 * 
	 * @return cv::Mat 
	 */
	cv::Mat getDepthImgFromBuff();

	/**
	 * @brief Render color image to front buffer
	 * 
	 * @param in_modelIndice Index of model file
	 * @param camPositon Position of the camera in opengl coordinate space
	 */
	void renderColorToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon);

	/**
	 * @brief Render color image to front buffer
	 * 
	 * @param in_modelIndice Index of model file
	 * @param in_rotMat Rotation matrix of object
	 * @param in_traVec Translation vector of object
	 */
	void renderColorToFrontBuff(uint16_t in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec);

	/**
	 * @brief Render depth image to front buffer
	 * 
	 * @param in_modelIndice Index of model file
	 * @param camPositon Position of the camera in opengl coordinate space
	 */
	void renderDepthToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon);

	/**
	 * @brief Render depth image to front buffer
	 * 
	 * @param in_modelIndice Index of model file
	 * @param in_rotMat Rotation matrix of object
	 * @param in_traVec Translation vector of object
	 */
	void renderDepthToFrontBuff(uint16_t in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec);

	/**
	 * @brief Load model file into GPU Buffer
	 * 
	 * @param in_modelLocation 
	 */
	void creatModBuffFromFiles(std::string const& in_modelLocation);

	/**
	 * @brief Read a 3D-file and place into Model struct
	 * 
	 * @param in_file 
	 * @param[out] in_model 
	 */
	void readModelFile(std::string const& in_file, Model& in_model);

private:
	SDL_Window* window;
	std::vector<ModelBuffer> modBuff;
	cv::Mat renderedColorImg;
	cv::Mat renderedDepthImg;
	uint16_t width;
	uint16_t height;
	float fieldOfView;
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
	float cx;
	float cy;

	/**
	 * @brief Setting up the opengl context
	 * 
	 */
	void setupSDLWindow();

	/**
	 * @brief Setup opengl
	 * 
	 */
	void setupOpenGL();

	/**
	 * @brief Setting up the framebuffer
	 * 
	 */
	void setupFramebuffer();

	/**
	 * @brief Setting up the different shader for rgb and depth image
	 * 
	 */
	void setupShader();

	/**
	 * @brief Helper function to convert a shader file to a string
	 * 
	 * @param filename 
	 * @return std::string 
	 */
	std::string fileToString(const char* filename);

	/**
	 * @brief Helper function to combine two vector in a zip style fashion
	 * 
	 * @param a 
	 * @param b 
	 * @return std::vector<glm::vec3> 
	 */
	std::vector<glm::vec3> zipVectors(const std::vector<glm::vec3>& a,
	                                  const std::vector<glm::vec3>& b);

	/**
	 * @brief Function to calculate the view matrix depending on the camera position
	 * 
	 * @param in_vec 
	 */
	void translateCam(glm::vec3 in_vec);
};
