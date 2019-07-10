#pragma once
//#include <vector>
#include <string>
//#include <cassert>
#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "defines.h"

class ModelImporter
{
public:
	ModelImporter();
	~ModelImporter();

	void importModel(std::string const& in_modelFile, Model& in_model);
private:
	Assimp::Importer* importer;

	void processNode(aiNode* in_node, const aiScene* in_scene, Model& in_model);
	void processMesh(aiMesh* mesh, const aiScene* scene, Model& in_model);
};
