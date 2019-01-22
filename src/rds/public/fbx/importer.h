#pragma once

#include "coremin.h"
#include "renderer/mesh.h"
#include <fbxsdk.h>

class Importer
{
protected:
	/// Fbx manager and importer
	FbxManager * manager;
	FbxImporter * importer;

	/// Current fbx scene
	FbxScene * scene;

	/// Fbx geometry converter
	FbxGeometryConverter * geometryConverter;

public:
	/// Default constructor
	Importer();

	/// Load scene
	void loadScene(const String & filename);

protected:
	/// Find all nodes with mesh
	void fillMeshNodes(FbxNode * pNode, Array<FbxNode*> & meshNodes) const;

public:
	/// Import mesh
	bool importStaticMesh(MeshData & meshData);
};