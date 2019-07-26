#include "ModelImporter.h"

ModelImporter::ModelImporter()
{
	importer = new Assimp::Importer;
}

ModelImporter::~ModelImporter()
{
	delete importer;
}

void ModelImporter::importModel(std::string const& in_modelFile, Model& in_model)
{
	const aiScene* scene = importer->ReadFile(in_modelFile,
		aiProcess_Triangulate | aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph
		| aiProcess_JoinIdenticalVertices | aiProcess_ImproveCacheLocality);
	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE, !scene->mRootNode)
	{
		std::cout << "ERROR:: Reading file wih Assimp " << importer->GetErrorString() << std::endl;
	}
	processNode(scene->mRootNode, scene, in_model);
}

Assimp::Importer* importer;

void ModelImporter::processNode(aiNode* in_node, const aiScene* in_scene, Model& in_model)
{
	for (unsigned int i = 0; i < in_node->mNumMeshes; i++)
	{
		aiMesh* mesh = in_scene->mMeshes[in_node->mMeshes[i]];
		processMesh(mesh, in_scene, in_model);
	}

	for (unsigned int i = 0; i < in_node->mNumChildren; i++)
	{
		processNode(in_node->mChildren[i], in_scene, in_model);
	}
}

void ModelImporter::processMesh(aiMesh* mesh, const aiScene* scene, Model& in_model)
{
	for (unsigned int i = 0; i < mesh->mNumVertices; i++)
	{
		glm::vec3 vertex;
		vertex.x = mesh->mVertices[i].x;
		vertex.y = mesh->mVertices[i].y;
		vertex.z = mesh->mVertices[i].z;
		in_model.vertices.push_back(vertex);
	}
	if (mesh->HasVertexColors(0))
	{
		for (unsigned int i = 0; i < mesh->mNumVertices; i++)
		{
			glm::vec3 color;
			color.r = mesh->mColors[0][i].r;
			color.g = mesh->mColors[0][i].g;
			color.b = mesh->mColors[0][i].b;
			in_model.colors.push_back(color);
		}
	}
	else
	{
		for (unsigned int i = 0; i < mesh->mNumVertices; i++)
		{
			glm::vec3 color(1.0f, 1.0f, 1.0f);
			in_model.colors.push_back(color);
		}
	}
	for (unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		aiFace face = mesh->mFaces[i];
		assert(face.mNumIndices == 3);
		for (unsigned int j = 0; j < face.mNumIndices; j++)
		{
			in_model.indices.push_back(face.mIndices[j]);
		}
	}
	in_model.numIndices = in_model.indices.size();
	in_model.numVertices = in_model.vertices.size();
}