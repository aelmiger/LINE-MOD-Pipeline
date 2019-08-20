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
#include <glm/gtx/rotate_vector.hpp>

#include "defines.h"
#include "utility.h"
#include "model_buffer.h"
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
	 * @param in_rotate In plane camera rotation angle
	 * @param in_x Move camera on camera plane to the right/left
	 * @param in_y Move camera on camera plane down/up
	 */
	void renderColorToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon, float in_rotate = 0.0f,
		float in_x = 0.0f, float in_y = 0.0f);

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
	 * @param in_rotate In plane camera rotation angle
	 * @param in_x Move camera on camera plane to the right/left
	 * @param in_y Move camera on camera plane down/up
	 */
	void renderDepthToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon, float in_rotate = 0.0f,
		float in_x = 0.0f, float in_y = 0.0f);

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

	void setupSDLWindow();
	void setupOpenGL();
	void setupFramebuffer();
	void setupShader();
	std::vector<glm::vec3> zipVectors(const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b);
	void translateCam(glm::vec3 in_vec, float in_rotate, float in_x, float in_y);
};
