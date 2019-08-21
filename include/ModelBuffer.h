#pragma once
#define GLEW_STATIC
#include <GL/glew.h>
#include "defines.h"
#include <cstddef>

struct ModelBuffer
{
	ModelBuffer(void* in_vertData, uint32_t in_numVertices, void* in_indData,
	            uint32_t in_numIndices, uint8_t in_elementSize);

	void bind();

	void unbind();

public:
	GLuint numIndices;
private:
	GLuint indBufferId;
	GLuint vertBufferId;
	GLuint vao;
};
