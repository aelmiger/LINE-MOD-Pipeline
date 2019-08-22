#pragma once
#include "defines.h"
#include <vector>
#include <glm/glm.hpp>

const float goldenRatio = 1.61803398875;

/**
 * @brief Class to generate camera view points. Either from subdivided icosaheadron or circle
 * 
 */
class CameraViewPoints
{
	float radius;
	float icosahedronPointA;
	float icosahedronPointB;
	uint8_t numSubdivisions;

	std::vector<glm::vec3> vertices;
	std::vector<Index> indices;
public:
	/**
	 * @brief Construct a new Camera View Points object
	 * 
	 */
	CameraViewPoints();
	~CameraViewPoints();

	/**
	 * @brief Create a Camera View Points
	 * 
	 * @param in_radius Distance in millimeter between camera and object origin
	 * @param in_subdivions Number of subdivisions for icosaeder
	 */
	void createCameraViewPoints(float in_radius, uint8_t in_subdivions);

	/**
	 * @brief Read symmetry properties of the object file
	 * 
	 * @param in_modelFile Name of the model
	 */
	void readModelProperties(std::string in_modelFile);

	/**
	 * @brief Get the Vertices
	 * 
	 * @return std::vector<glm::vec3>& 
	 */
	std::vector<glm::vec3>& getVertices();
private:
	ModelProperties modProps;

	/**
	 * @brief Remove camera positions that generate an identical image for object
	 * 
	 */
	void removeSuperfluousVertices();

	/**
	 * @brief Estimating icosahedron points depending on given radius
	 * 
	 */
	void icosahedronPointsFromRadius();

	/**
	 * @brief Create vertices for a rotation symmetrical object
	 * @detail Degree distance between points is 60deg / number of subdivisons
	 * 
	 */
	void createVerticesForRotSym();

	/**
	 * @brief Create an Icosahedron
	 * 
	 */
	void createIcosahedron();

	/**
	 * @brief Remove duplicate vertices after subdivison
	 * 
	 * @param vertSize 
	 * @return int32_t 
	 */
	int32_t checkForDuplicate(uint32_t vertSize);

	/**
	 * @brief subdivide the created icosahedron
	 * 
	 */
	void subdivide();

	/**
	 * @brief Adjust the distance of a subdivided vertex to match the radius
	 * 
	 * @param index 
	 */
	void adjustVecToRadius(uint32_t index);
};
