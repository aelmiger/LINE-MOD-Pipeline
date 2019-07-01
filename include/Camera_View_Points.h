#pragma once
#include "defines.h"
#include <vector>
#include <math.h>
#include <ppl.h>

#include <glm/glm.hpp>

const float32 goldenRatio = 1.61803398875;

class CameraViewPoints {
	float32 radius;
	float32 icosahedronPointA;
	float32 icosahedronPointB;
	uint8 numSubdivisions;

	std::vector<glm::vec3> vertices;
	std::vector<Index> indices;
public:
	CameraViewPoints(float32 in_radius, uint8 in_subdivions);
	CameraViewPoints(float32 in_radius);
	uint32 getNumVertices();
	uint32 getNumIndices();
	std::vector<glm::vec3>& getVertices();
	std::vector<Index>& getIndices();

private:
	void icosahedronPointsFromRadius();
	void createVerticesForRotSym();
	void createIcosahedron();
	int32 checkForDuplicate(uint32 vertSize);
	void subdivide();
	void adjustVecToRadius(uint32 index);
};
