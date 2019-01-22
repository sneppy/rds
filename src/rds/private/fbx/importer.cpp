#include "fbx/importer.h"

Importer::Importer()
{
	// Create Fbx utilities
	manager = FbxManager::Create();

	FbxIOSettings * settings = FbxIOSettings::Create(manager, IOSROOT);
	manager->SetIOSettings(settings);

	importer = FbxImporter::Create(manager, "RootImporter");
	scene = nullptr;

	geometryConverter = new FbxGeometryConverter(manager);
}

void Importer::loadScene(const String & filename)
{
	if (scene) scene->Destroy();

	// Import scene
	scene = FbxScene::Create(manager, *filename);

	importer->Initialize(*filename);
	importer->Import(scene);
}

void Importer::fillMeshNodes(FbxNode * pNode, Array<FbxNode*> & meshNodes) const
{
	const uint32 nodeCount = pNode->GetChildCount();
	for (uint32 i = 0; i < nodeCount; ++i)
	{
		const auto node = pNode->GetChild(i);
		if (node->GetMesh()) meshNodes.push(node);

		// Recursive walk
		fillMeshNodes(node, meshNodes);
	}
}

bool Importer::importStaticMesh(MeshData & meshData)
{
	if (!scene) return false;

	FbxNode * rootNode = scene->GetRootNode();

	// Find all nodes with meshes
	Array<FbxNode*> meshNodes;
	fillMeshNodes(rootNode, meshNodes);

	if (meshNodes.getCount() < 1) return false;

	// Quickly get total number of vertices
	uint32 numVertices = 0, vertexIndexOffset = 0;
	for (const auto node : meshNodes)
	{
		auto fbxMesh = node->GetMesh();
		numVertices += fbxMesh->GetControlPointsCount();
	}

	// Create meshData from scratch
	meshData = MeshData(numVertices);

	for (const auto node : meshNodes)
	{
		FbxMesh * mesh = node->GetMesh();

		// Triangulate mesh
		if (!mesh->IsTriangleMesh())
		{
			const auto convertedNode = geometryConverter->Triangulate(mesh, true);
			if (convertedNode) mesh = (FbxMesh*)convertedNode;
		}

		FbxLayer * baseLayer = mesh->GetLayer(0);

		// Get normal layer
		FbxLayerElementNormal * layerNormals = baseLayer->GetNormals();

		// Has normal data
		const bool bHasNormalData = layerNormals != nullptr;

		FbxLayerElement::EReferenceMode	normalsReferenceMode;
		FbxLayerElement::EMappingMode	normalsMappingMode;
		if (layerNormals)
		{
			normalsReferenceMode	= layerNormals->GetReferenceMode();
			normalsMappingMode		= layerNormals->GetMappingMode();
		}

		const uint32 polygonCount = mesh->GetPolygonCount();
		if (!polygonCount) continue;

		const uint32 vertexCount = mesh->GetControlPointsCount();
		if (!vertexCount) continue;

		// Add vertices
		Array<Vec3<float32, false>> controlPoints(numVertices);
		for (uint32 i = 0; i < vertexCount; ++i)
		{
			// Add vertex
			FbxVector4 controlPoint = mesh->GetControlPointAt(i);
			controlPoints.push(Vec3<float32, false>(controlPoint[0], controlPoint[2], -controlPoint[1]));
		}

		// By polygon
		uint32 vertexIndex = 0;
		for (uint32 i = 0; i < polygonCount; ++i)
		{
			const uint8 polygonVertexCount = mesh->GetPolygonSize(i);
			for (uint8 j = 0; j < polygonVertexCount; ++j, ++vertexIndex)
			{
				VertexData vertexData;

				// Get corner vertex
				const uint32 controlPointIndex = mesh->GetPolygonVertex(i, j);
				vertexData.position = controlPoints[controlPointIndex];
				
				if (layerNormals)
				{
					const uint32 normalMapIndex = normalsMappingMode == FbxLayerElement::eByControlPoint
						? controlPointIndex
						: vertexIndex;

					const uint32 normalValueIndex = normalsReferenceMode == FbxLayerElement::eDirect
						? normalMapIndex
						: layerNormals->GetIndexArray().GetAt(normalMapIndex);
					
					FbxVector4 normal = layerNormals->GetDirectArray().GetAt(normalValueIndex);
					vertexData.normal = Vec3<float32, false>(normal[0], normal[2], -normal[1]);
				}

				meshData.addVertex(vertexData);
			}
		}
	}
}