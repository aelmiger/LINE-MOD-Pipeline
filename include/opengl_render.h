#pragma once

#include <vector>
#include <iostream>
#include <string>

#define SDL_MAIN_HANDLED
#include <SDL/SDL.h>
#define GLEW_STATIC
#include <GL/glew.h>
#include <opencv2/core.hpp>

#include "defines.h"
#include "utility.h"
#include "model_buffer.h"

class OpenGLRender
{
public:
	OpenGLRender(CameraParameters const& in_camParams)
	{

		width = in_camParams.videoWidth;
		height = in_camParams.videoHeight;
		cx = in_camParams.cx;
		cy = in_camParams.cy;
		fieldOfView = 360.0f/M_PI * atanf(height / (2 * in_camParams.fy));
		projection = glm::perspective(glm::radians(fieldOfView), (float)width / (float)height, 100.0f, 10000.0f);


		view = glm::mat4(1.0f);
		position = glm::vec3(0.0f);
		up = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));

		renderedDepthImg.create(height,width,CV_16UC1);
		renderedColorImg.create(height, width, CV_8UC3);

		setupSDLWindow();
		setupOpenGL();
		setupFramebuffer();
		setupShader();


	}
	~OpenGLRender() {
		glDeleteProgram(depthShaderProgram);
		glDeleteProgram(colorShaderProgram);
		SDL_DestroyWindow(window);
	}

	cv::Mat getColorImgFromBuff() {
		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, renderedColorImg.data);
		cv::flip(renderedColorImg, renderedColorImg, 0); //TODO Wird flip wirklich benötigt?
		return renderedColorImg;
	}
	cv::Mat getDepthImgFromBuff() {
		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, renderedDepthImg.data);
		cv::flip(renderedDepthImg, renderedDepthImg, 0);
		return renderedDepthImg;
	}

	void renderColorToFrontBuff(uint16 in_modelIndice, glm::vec3 camPositon, float32 in_rotate, float32 in_x, float32 in_y) {
		ModelBuffer* modPointer = &modBuff[in_modelIndice];

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glUseProgram(colorShaderProgram);
		glPixelStorei(GL_PACK_ALIGNMENT, (renderedColorImg.step & 3) ? 1 : 4);
		glPixelStorei(GL_PACK_ROW_LENGTH, renderedColorImg.step / renderedColorImg.elemSize());

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		modelMat = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
		translateCam(camPositon, in_rotate, in_x, in_y);
		modPointer->bind();
		glUniformMatrix4fv(modelViewProjMatrixLocationColor, 1, GL_FALSE, &viewProj[0][0]);
		glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, 0);
		modPointer->unbind();
		SDL_GL_SwapWindow(window);
	}
	void renderColorToFrontBuff(uint16 in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec) {
		ModelBuffer* modPointer = &modBuff[in_modelIndice];

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glUseProgram(colorShaderProgram);
		glPixelStorei(GL_PACK_ALIGNMENT, (renderedColorImg.step & 3) ? 1 : 4);
		glPixelStorei(GL_PACK_ROW_LENGTH, renderedColorImg.step / renderedColorImg.elemSize());

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		view = in_rotMat;

		view[3][0] = in_traVec[0];
		view[3][1] = -in_traVec[1];
		view[3][2] = -in_traVec[2];
		view[3][3] = 1.0f;
		modelMat = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
		viewProj = projection * view*modelMat;
		modPointer->bind();
		glUniformMatrix4fv(modelViewProjMatrixLocationColor, 1, GL_FALSE, &viewProj[0][0]);
		glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, 0);
		modPointer->unbind();
		SDL_GL_SwapWindow(window);
	}



	void renderDepthToFrontBuff(uint16 in_modelIndice, glm::vec3 camPositon, float32 in_rotate, float32 in_x, float32 in_y) {
		ModelBuffer* modPointer = &modBuff[in_modelIndice];


		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glUseProgram(depthShaderProgram);
		glPixelStorei(GL_PACK_ALIGNMENT, (renderedDepthImg.step & 3) ? 1 : 4);
		glPixelStorei(GL_PACK_ROW_LENGTH, renderedDepthImg.step / renderedDepthImg.elemSize());

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		modelMat = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
		translateCam(camPositon, in_rotate, in_x, in_y);
		modPointer->bind();
		glUniformMatrix4fv(modelViewProjMatrixLocationDepth, 1, GL_FALSE, &viewProj[0][0]);
		glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, 0);
		modPointer->unbind();
		SDL_GL_SwapWindow(window);
	}

	

	void renderDepthToFrontBuff(uint16 in_modelIndice, glm::mat4 in_rotMat,glm::vec3 in_traVec) {
		ModelBuffer* modPointer = &modBuff[in_modelIndice];

		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glUseProgram(depthShaderProgram);
		glPixelStorei(GL_PACK_ALIGNMENT, (renderedDepthImg.step & 3) ? 1 : 4);
		glPixelStorei(GL_PACK_ROW_LENGTH, renderedDepthImg.step / renderedDepthImg.elemSize());

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		view = in_rotMat;

		view[3][0] = in_traVec[0];
		view[3][1] = -in_traVec[1];
		view[3][2] = -in_traVec[2];
		view[3][3] = 1.0f;
		modelMat = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
		viewProj = projection * view*modelMat;
		modPointer->bind();
		glUniformMatrix4fv(modelViewProjMatrixLocationDepth, 1, GL_FALSE, &viewProj[0][0]);
		glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, 0);
		modPointer->unbind();
		SDL_GL_SwapWindow(window);
	}


	void creatModBuffFromFiles(std::string in_modelLocation) {
		modBuff.clear();
		Model tmp;
		readModelFile(in_modelLocation, tmp);
		std::vector<glm::vec3> tempVert;
		tempVert = zipVectors(tmp.vertices, tmp.colors);
		modBuff.push_back(ModelBuffer(tempVert.data(), tmp.numVertices, tmp.indices.data(), tmp.numIndices, sizeof(tmp.indices[0])));
	}

	void readModelFile(std::string in_file,Model &in_model) {
		std::ifstream input = std::ifstream(in_file, std::ios::in | std::ios::binary);
		if (!input.is_open()) {
			std::cout << "Error loading " << in_file << " file" << std::endl;
		}

		input.read((char*)&in_model.numVertices, sizeof(uint64));
		input.read((char*)&in_model.numIndices, sizeof(uint64));

		for (uint64 i = 0; i < in_model.numVertices; i++) {
			glm::vec3 vertex;
			input.read((char*)&vertex.x, sizeof(float));
			input.read((char*)&vertex.y, sizeof(float));
			input.read((char*)&vertex.z, sizeof(float));
			in_model.vertices.push_back(vertex);
		}
		for (uint64 i = 0; i < in_model.numIndices; i++) {
			uint32 index;
			input.read((char*)&index, sizeof(uint32));
			in_model.indices.push_back(index);
		}
		if (input.peek() == EOF) {
			for (uint64 i = 0; i < in_model.numVertices; i++) {
				glm::vec3 color(1.0f,1.0f,1.0f);
				in_model.colors.push_back(color);
			}
		}
		else {
			for (uint64 i = 0; i < in_model.numVertices; i++) {
				glm::vec3 color;
				input.read((char*)&color.x, sizeof(float));
				input.read((char*)&color.y, sizeof(float));
				input.read((char*)&color.z, sizeof(float));
				in_model.colors.push_back(color);
			}
		}

		input.close();
	}



	void calculateMatch3DPosition(ObjectPose &in_objectPos, TemplatePosition &in_templatePosition, cv::linemod::Match &in_match) {
		glm::vec3 translation;
		float64 alpha;
		cv::Vec3f angles;
		

		translation.z = glm::length(in_templatePosition.positionCam);
		alpha = ((1.0f - (in_match.y + cy - in_templatePosition.boundingBox.y) / (cy)) * fieldOfView / 2.0f);
		translation.y = -tan(alpha*M_PI / 180) * translation.z;
		alpha = ((1.0f - (in_match.x + cx - in_templatePosition.boundingBox.x) / (cx)) * fieldOfView * ((float64)width / (float64)height) / 2.0f);
		translation.x = -tan(alpha*M_PI / 180) * translation.z;

		translateCam(in_templatePosition.positionCam, in_templatePosition.rotation, 0.0f, 0.0f);
		glm::qua quats = openGL2openCVRotation(view);
		in_objectPos = {translation, quats};

	}


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
	glm::vec3 eulAngles;
	GLuint colorShaderProgram;
	GLuint depthShaderProgram;
	GLuint fbo;
	GLuint modelViewProjMatrixLocationColor;
	GLuint modelViewProjMatrixLocationDepth;
	float32 cx;
	float32 cy;

	void setupSDLWindow() {
		SDL_Init(SDL_INIT_EVERYTHING);

		SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
		SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
		SDL_GL_SetSwapInterval(0);

		uint32 flags = SDL_WINDOW_OPENGL;
		window = SDL_CreateWindow("Render Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags);
		SDL_GLContext glContext = SDL_GL_CreateContext(window);
	}

	void setupOpenGL() {
		GLenum errorMsg = glewInit();
		if (errorMsg != GLEW_OK) {
			std::cout << "Error: " << glewGetErrorString(errorMsg) << std::endl;
			std::cin.get();
		}
		else {
			std::cout << "OpenGL successfully initiated. Version: " << glGetString(GL_VERSION) << std::endl;
		}
		glEnable(GL_DEPTH_TEST);


	}

	void setupFramebuffer() {
		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		unsigned int texture;
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, 640, 480, 0, GL_RED, GL_UNSIGNED_SHORT, NULL);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		unsigned int rbo;
		glGenRenderbuffers(1, &rbo);
		glBindRenderbuffer(GL_RENDERBUFFER, rbo);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, 640, 480);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void setupShader() {

		colorShaderProgram = glCreateProgram();

		GLuint vertShadId = glCreateShader(GL_VERTEX_SHADER);
		std::string srcString = fileToString("shader/basic.vs");
		const char* vertShadSrc = srcString.c_str();
		glShaderSource(vertShadId, 1, &vertShadSrc, 0);
		glCompileShader(vertShadId);

		int result;
		glGetShaderiv(vertShadId, GL_COMPILE_STATUS, &result);
		if (result != GL_TRUE) {
			std::cout << "Vertex Shader compilation error" << std::endl;
		}
		srcString.clear();


		GLuint fragShadId = glCreateShader(GL_FRAGMENT_SHADER);
		srcString = fileToString("shader/basic.fs");
		vertShadSrc = srcString.c_str();
		glShaderSource(fragShadId, 1, &vertShadSrc, 0);
		glCompileShader(fragShadId);

		glGetShaderiv(fragShadId, GL_COMPILE_STATUS, &result);
		if (result != GL_TRUE) {
			std::cout << "Fragment Shader compilation error" << std::endl;
		}

		glAttachShader(colorShaderProgram, vertShadId);
		glAttachShader(colorShaderProgram, fragShadId);
		glLinkProgram(colorShaderProgram);

		glGetProgramiv(colorShaderProgram, GL_LINK_STATUS, &result);
		if (!result) {
			std::cout << "Shader Program Linking error" << std::endl;
		}

		modelViewProjMatrixLocationColor = glGetUniformLocation(colorShaderProgram, "u_modelViewProj");
		glUseProgram(colorShaderProgram);
		glDeleteShader(vertShadId);
		glDeleteShader(fragShadId);

		depthShaderProgram = glCreateProgram();

		vertShadId = glCreateShader(GL_VERTEX_SHADER);
		srcString = fileToString("shader/basic.vs");
		vertShadSrc = srcString.c_str();
		glShaderSource(vertShadId, 1, &vertShadSrc, 0);
		glCompileShader(vertShadId);

		glGetShaderiv(vertShadId, GL_COMPILE_STATUS, &result);
		if (result != GL_TRUE) {
			std::cout << "Vertex Shader compilation error" << std::endl;
		}
		srcString.clear();


		fragShadId = glCreateShader(GL_FRAGMENT_SHADER);
		srcString = fileToString("shader/depth.fs");
		vertShadSrc = srcString.c_str();
		glShaderSource(fragShadId, 1, &vertShadSrc, 0);
		glCompileShader(fragShadId);

		glGetShaderiv(fragShadId, GL_COMPILE_STATUS, &result);
		if (result != GL_TRUE) {
			std::cout << "Fragment Shader compilation error" << std::endl;
		}

		glAttachShader(depthShaderProgram, vertShadId);
		glAttachShader(depthShaderProgram, fragShadId);
		glLinkProgram(depthShaderProgram);

		glGetProgramiv(depthShaderProgram, GL_LINK_STATUS, &result);
		if (!result) {
			std::cout << "Shader Program Linking error" << std::endl;
		}

		modelViewProjMatrixLocationDepth = glGetUniformLocation(depthShaderProgram, "u_modelViewProj");
		glUseProgram(depthShaderProgram);
		glDeleteShader(vertShadId);
		glDeleteShader(fragShadId);

	}


	std::vector<glm::vec3> zipVectors(const std::vector<glm::vec3> & a, const std::vector<glm::vec3> & b) {
		std::vector <glm::vec3> result;
		result.reserve(a.size() + b.size());

		for (size_t i = 0; i < a.size(); i++) {
			result.push_back(a[i]);
			result.push_back(b[i]);
		}
		return result;
	}

	void translateCam(glm::vec3 in_vec,float32 in_rotate, float32 in_x, float32 in_y) {
		position = in_vec;
		if (in_vec[0] == 0 && in_vec[2] == 0) { //Looking straight up or down fails the cross product
			position[0] = 0.000001;
			position[2] = 0.000001;
		}
		glm::vec3 fakeup = glm::normalize(glm::cross(position, glm::cross(position, up)));
		glm::vec3 mov = (glm::normalize(glm::cross(position, up)) * in_x);
		glm::vec3 movu = fakeup * in_y;

		glm::vec3 temporaryUp = glm::rotate(-fakeup, glm::radians(in_rotate),glm::normalize(position));
		view = glm::lookAt(position+mov+movu, mov + movu, temporaryUp);
		viewProj = projection * view * modelMat;
	}



};

