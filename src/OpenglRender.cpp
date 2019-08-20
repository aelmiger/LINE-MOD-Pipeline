#include "OpenglRender.h"

OpenGLRender::OpenGLRender(CameraParameters const& in_camParams)
{
	width = in_camParams.videoWidth;
	height = in_camParams.videoHeight;
	cx = in_camParams.cx;
	cy = in_camParams.cy;
	fieldOfView = 360.0f / M_PI * atanf(height / (2 * in_camParams.fy));
	projection = glm::perspective(glm::radians(fieldOfView), (float)width / (float)height, 100.0f, 10000.0f);

	view = glm::mat4(1.0f);
	position = glm::vec3(0.0f);
	up = normalize(glm::vec3(0.0f, 1.0f, 0.0f));

	renderedDepthImg.create(height, width, CV_16UC1);
	renderedColorImg.create(height, width, CV_8UC3);

	setupSDLWindow();
	setupOpenGL();
	setupFramebuffer();
	setupShader();
}

OpenGLRender::~OpenGLRender()
{
	glDeleteProgram(depthShaderProgram);
	glDeleteProgram(colorShaderProgram);
	SDL_DestroyWindow(window);
}

cv::Mat OpenGLRender::getColorImgFromBuff()
{
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, renderedColorImg.data);
	flip(renderedColorImg, renderedColorImg, 0); //TODO Wird flip wirklich ben�tigt?
	return renderedColorImg;
}

cv::Mat OpenGLRender::getDepthImgFromBuff()
{
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, renderedDepthImg.data);
	flip(renderedDepthImg, renderedDepthImg, 0);
	return renderedDepthImg;
}

void OpenGLRender::renderColorToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon, float in_rotate, float in_x,
	float in_y)
{
	ModelBuffer* modPointer = &modBuff[in_modelIndice];

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glUseProgram(colorShaderProgram);
	glPixelStorei(GL_PACK_ALIGNMENT, (renderedColorImg.step & 3) ? 1 : 4);
	glPixelStorei(GL_PACK_ROW_LENGTH, renderedColorImg.step / renderedColorImg.elemSize());

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	modelMat = rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
	translateCam(camPositon, in_rotate, in_x, in_y);
	modPointer->bind();
	glUniformMatrix4fv(modelViewProjMatrixLocationColor, 1, GL_FALSE, &viewProj[0][0]);
	glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, nullptr);
	modPointer->unbind();
	SDL_GL_SwapWindow(window);
}

void OpenGLRender::renderColorToFrontBuff(uint16_t in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec)
{
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
	modelMat = rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
	viewProj = projection * view; //* modelMat;
	modPointer->bind();
	glUniformMatrix4fv(modelViewProjMatrixLocationColor, 1, GL_FALSE, &viewProj[0][0]);
	glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, nullptr);
	modPointer->unbind();
	SDL_GL_SwapWindow(window);
}

void OpenGLRender::renderDepthToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon, float in_rotate, float in_x,
	float in_y)
{
	ModelBuffer* modPointer = &modBuff[in_modelIndice];

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glUseProgram(depthShaderProgram);
	glPixelStorei(GL_PACK_ALIGNMENT, (renderedDepthImg.step & 3) ? 1 : 4);
	glPixelStorei(GL_PACK_ROW_LENGTH, renderedDepthImg.step / renderedDepthImg.elemSize());

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	modelMat = rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
	translateCam(camPositon, in_rotate, in_x, in_y);
	modPointer->bind();
	glUniformMatrix4fv(modelViewProjMatrixLocationDepth, 1, GL_FALSE, &viewProj[0][0]);
	glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, nullptr);
	modPointer->unbind();
	SDL_GL_SwapWindow(window);
}

void OpenGLRender::renderDepthToFrontBuff(uint16_t in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec)
{
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
	modelMat = rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
	viewProj = projection * view;// *modelMat;
	modPointer->bind();
	glUniformMatrix4fv(modelViewProjMatrixLocationDepth, 1, GL_FALSE, &viewProj[0][0]);
	glDrawElements(GL_TRIANGLES, modPointer->numIndices, GL_UNSIGNED_INT, nullptr);
	modPointer->unbind();
	SDL_GL_SwapWindow(window);
}

void OpenGLRender::creatModBuffFromFiles(std::string const& in_modelLocation)
{
	Model tmp;
	readModelFile(in_modelLocation, tmp);
	std::vector<glm::vec3> tempVert;
	tempVert = zipVectors(tmp.vertices, tmp.colors);
	modBuff.emplace_back(tempVert.data(), tmp.numVertices, tmp.indices.data(), tmp.numIndices, sizeof(tmp.indices[0]));
}

void OpenGLRender::readModelFile(std::string const& in_file, Model& in_model)
{
	ModelImporter modImport;
	modImport.importModel(in_file, in_model);
}

void OpenGLRender::setupSDLWindow()
{
	SDL_Init(SDL_INIT_EVERYTHING);

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); 
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3); 
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); 
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetSwapInterval(0);

	uint32_t flags = SDL_WINDOW_OPENGL;
	window = SDL_CreateWindow("Render Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags);
	SDL_GLContext glContext = SDL_GL_CreateContext(window);
}

void OpenGLRender::setupOpenGL()
{
	GLenum errorMsg = glewInit();
	if (errorMsg != GLEW_OK)
	{
		std::cout << "Error: " << glewGetErrorString(errorMsg) << std::endl;
		std::cin.get();
	}
	else
	{
		std::cout << "OpenGL successfully initiated. Version: " << glGetString(GL_VERSION) << std::endl;
	}
	glEnable(GL_DEPTH_TEST);
}

void OpenGLRender::setupFramebuffer()
{
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	unsigned int texture;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, 640, 480, 0, GL_RED, GL_UNSIGNED_SHORT, nullptr);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	unsigned int rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, 640, 480);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLRender::setupShader()
{
	colorShaderProgram = glCreateProgram();

	GLuint vertShadId = glCreateShader(GL_VERTEX_SHADER);
	std::string srcString = fileToString("shader/basic.vs");
	const char* vertShadSrc = srcString.c_str();
	glShaderSource(vertShadId, 1, &vertShadSrc, nullptr);
	glCompileShader(vertShadId);

	int result;
	glGetShaderiv(vertShadId, GL_COMPILE_STATUS, &result);
	if (result != GL_TRUE)
	{
		std::cout << "Vertex Shader compilation error" << std::endl;
	}
	srcString.clear();

	GLuint fragShadId = glCreateShader(GL_FRAGMENT_SHADER);
	srcString = fileToString("shader/basic.fs");
	vertShadSrc = srcString.c_str();
	glShaderSource(fragShadId, 1, &vertShadSrc, nullptr);
	glCompileShader(fragShadId);

	glGetShaderiv(fragShadId, GL_COMPILE_STATUS, &result);
	if (result != GL_TRUE)
	{
		std::cout << "Fragment Shader compilation error" << std::endl;
	}

	glAttachShader(colorShaderProgram, vertShadId);
	glAttachShader(colorShaderProgram, fragShadId);
	glLinkProgram(colorShaderProgram);

	glGetProgramiv(colorShaderProgram, GL_LINK_STATUS, &result);
	if (!result)
	{
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
	glShaderSource(vertShadId, 1, &vertShadSrc, nullptr);
	glCompileShader(vertShadId);

	glGetShaderiv(vertShadId, GL_COMPILE_STATUS, &result);
	if (result != GL_TRUE)
	{
		std::cout << "Vertex Shader compilation error" << std::endl;
	}
	srcString.clear();

	fragShadId = glCreateShader(GL_FRAGMENT_SHADER);
	srcString = fileToString("shader/depth.fs");
	vertShadSrc = srcString.c_str();
	glShaderSource(fragShadId, 1, &vertShadSrc, nullptr);
	glCompileShader(fragShadId);

	glGetShaderiv(fragShadId, GL_COMPILE_STATUS, &result);
	if (result != GL_TRUE)
	{
		std::cout << "Fragment Shader compilation error" << std::endl;
	}

	glAttachShader(depthShaderProgram, vertShadId);
	glAttachShader(depthShaderProgram, fragShadId);
	glLinkProgram(depthShaderProgram);

	glGetProgramiv(depthShaderProgram, GL_LINK_STATUS, &result);
	if (!result)
	{
		std::cout << "Shader Program Linking error" << std::endl;
	}

	modelViewProjMatrixLocationDepth = glGetUniformLocation(depthShaderProgram, "u_modelViewProj");
	glUseProgram(depthShaderProgram);
	glDeleteShader(vertShadId);
	glDeleteShader(fragShadId);
}

std::string OpenGLRender::fileToString(const char* filename)
{
	std::ifstream t(filename);
	std::stringstream buffer;
	buffer << t.rdbuf();
	t.close();
	std::string fds = buffer.str();
	return buffer.str();
}

std::vector<glm::vec3> OpenGLRender::zipVectors(const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b)
{
	std::vector<glm::vec3> result;
	result.reserve(a.size() + b.size());

	for (size_t i = 0; i < a.size(); i++)
	{
		result.push_back(a[i]);
		result.push_back(b[i]);
	}
	return result;
}

void OpenGLRender::translateCam(glm::vec3 in_vec, float in_rotate, float in_x, float in_y)
{
	position = in_vec;
	if (in_vec[0] == 0 && in_vec[2] == 0)
	{
		//Looking straight up or down fails the cross product
		position[0] = 0.000001;
		position[2] = 0.000001;
	}
	glm::vec3 fakeup = normalize(cross(position, cross(position, up)));
	glm::vec3 mov = (normalize(cross(position, up)) * in_x);
	glm::vec3 movu = fakeup * in_y;

	glm::vec3 temporaryUp = rotate(-fakeup, glm::radians(in_rotate), normalize(position));
	view = lookAt(position + mov + movu, mov + movu, temporaryUp);
	viewProj = projection * view;
}