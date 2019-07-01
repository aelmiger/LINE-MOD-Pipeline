#include "camera_view_points.h"

CameraViewPoints::CameraViewPoints(float32 in_radius, uint8 in_subdivions) {
	radius = in_radius;
	numSubdivisions = in_subdivions;
	icosahedronPointsFromRadius();
	createIcosahedron();
	vertices.reserve((20 * pow(4, (uint32)numSubdivisions)) / 2 + 2);
	indices.reserve(20 * pow(4, (uint32)numSubdivisions));
	subdivide();
}
CameraViewPoints::CameraViewPoints(float32 in_radius) {
	radius = in_radius;
	createVerticesForRotSym();
}
uint32 CameraViewPoints::getNumVertices() {
	return vertices.size();
}

uint32 CameraViewPoints::getNumIndices() {
	return indices.size();
}

std::vector<glm::vec3>& CameraViewPoints::getVertices() { return vertices; }
std::vector<Index>& CameraViewPoints::getIndices() { return indices; }

void CameraViewPoints::icosahedronPointsFromRadius() {
	icosahedronPointA = sqrt((radius * radius) / (goldenRatio * goldenRatio + 1));
	icosahedronPointB = icosahedronPointA * goldenRatio;
}
void CameraViewPoints::createVerticesForRotSym() {
	for (float32 i = 0; i < 90; i = i + 10)
	{
		vertices.emplace_back( cos(i*M_PI / 180.0f)*radius,sin(i*M_PI / 180.0f)*radius,0.0f );
	}
}
void CameraViewPoints::createIcosahedron() {
	vertices.emplace_back( -icosahedronPointA,0.0f,icosahedronPointB );
	vertices.emplace_back( icosahedronPointA,0.0f,icosahedronPointB );
	vertices.emplace_back( -icosahedronPointA,0.0f,-icosahedronPointB );
	vertices.emplace_back( icosahedronPointA,0.0f,-icosahedronPointB );

	vertices.emplace_back( 0.0f,icosahedronPointB,icosahedronPointA );
	vertices.emplace_back( 0.0f,icosahedronPointB,-icosahedronPointA );
	vertices.emplace_back( 0.0f,-icosahedronPointB,icosahedronPointA );
	vertices.emplace_back( 0.0f,-icosahedronPointB,-icosahedronPointA );

	vertices.emplace_back( icosahedronPointB,icosahedronPointA,0.0f );
	vertices.emplace_back( -icosahedronPointB,icosahedronPointA,0.0f );
	vertices.emplace_back( icosahedronPointB,-icosahedronPointA,0.0f );
	vertices.emplace_back( -icosahedronPointB,-icosahedronPointA,0.0f );

	indices.emplace_back(0, 4, 1 );
	indices.emplace_back( 0,9,4 );
	indices.emplace_back( 9,5,4 );
	indices.emplace_back( 4,5,8 );
	indices.emplace_back( 4,8,1 );

	indices.emplace_back( 8,10,1);
	indices.emplace_back( 8,3,10);
	indices.emplace_back( 5,3,8 );
	indices.emplace_back( 5,2,3 );
	indices.emplace_back( 2,7,3 );

	indices.emplace_back( 7,10,3);
	indices.emplace_back( 7,6,10);
	indices.emplace_back( 7,11,6);
	indices.emplace_back( 11,0,6);
	indices.emplace_back( 0,1,6 );

	indices.emplace_back( 6,1,10);
	indices.emplace_back( 9,0,11);
	indices.emplace_back( 9,11,2);
	indices.emplace_back( 9,2,5 );
	indices.emplace_back( 7,2,11);
}

int32 CameraViewPoints::checkForDuplicate(uint32 vertSize) {
	uint32 index = -1;
	concurrency::parallel_for(uint32(0), (uint32)vertSize, [&](uint32 i) {
		if (vertices[vertSize].x == vertices[i].x && vertices[vertSize].y == vertices[i].y && vertices[vertSize].z == vertices[i].z) {
			index = i;
		}
	});
	return index;
}

void CameraViewPoints::subdivide() {
	for (uint8 j = 0; j < numSubdivisions; j++) {
		uint32 numFaces = indices.size();
		uint32 duplicateIndex = 0;
		uint32 currentVertSize = vertices.size();
		for (uint32 i = 0; i < numFaces; i++) {
			vertices.emplace_back(
				(vertices[indices[i].a].x + vertices[indices[i].b].x) / 2,
				(vertices[indices[i].a].y + vertices[indices[i].b].y) / 2,
				(vertices[indices[i].a].z + vertices[indices[i].b].z) / 2 );
			adjustVecToRadius(currentVertSize);

			duplicateIndex = checkForDuplicate(currentVertSize);
			uint32 abIndex = 0;
			if (duplicateIndex != -1) {
				vertices.pop_back();
				abIndex = duplicateIndex;
			}
			else {
				abIndex = currentVertSize;
				currentVertSize++;
			}

			vertices.emplace_back(
				(vertices[indices[i].c].x + vertices[indices[i].b].x) / 2,
				(vertices[indices[i].c].y + vertices[indices[i].b].y) / 2,
				(vertices[indices[i].c].z + vertices[indices[i].b].z) / 2 );
			adjustVecToRadius(currentVertSize);

			uint32 bcIndex = 0;
			duplicateIndex = checkForDuplicate(currentVertSize);
			if (duplicateIndex != -1) {
				vertices.pop_back();
				bcIndex = duplicateIndex;
			}
			else {
				bcIndex = currentVertSize;
				currentVertSize++;
			}

			vertices.emplace_back(
				(vertices[indices[i].a].x + vertices[indices[i].c].x) / 2,
				(vertices[indices[i].a].y + vertices[indices[i].c].y) / 2,
				(vertices[indices[i].a].z + vertices[indices[i].c].z) / 2 );
			adjustVecToRadius(currentVertSize);

			uint32 acIndex = 0;
			duplicateIndex = checkForDuplicate(currentVertSize);
			if (duplicateIndex != -1) {
				vertices.pop_back();
				acIndex = duplicateIndex;
			}
			else {
				acIndex = currentVertSize;
				currentVertSize++;
			}

			indices.emplace_back(indices[i].a,abIndex,acIndex );
			indices.emplace_back( indices[i].b,abIndex,bcIndex );
			indices.emplace_back( indices[i].c,bcIndex,acIndex );
			indices[i] = { abIndex, bcIndex, acIndex };
		}
	}
}

void CameraViewPoints::adjustVecToRadius(uint32 index) {
	float32 adjustValue = sqrt(vertices[index].x * vertices[index].x + vertices[index].y * vertices[index].y + vertices[index].z * vertices[index].z) / radius;
	vertices[index].x /= adjustValue;
	vertices[index].y /= adjustValue;
	vertices[index].z /= adjustValue;
}