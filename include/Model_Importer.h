#pragma once
#include <vector>
#include <string>
#include <cassert>
#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "defines.h"
class Model_Importer
{
public:
	Model_Importer();
	~Model_Importer();

	void importModel(std::string in_modelFile, Model& in_model);
private:
	Assimp::Importer* importer;

	void processNode(aiNode* in_node, const aiScene* in_scene, Model& in_model);
	void processMesh(aiMesh* mesh, const aiScene* scene, Model& in_model);
};
