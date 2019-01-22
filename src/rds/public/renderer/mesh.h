#pragma once

#include "coremin.h"

struct VertexData
{
	Vec3<float32, false>	position;
	Vec3<float32, false>	normal;
	Vec3<float32, false>	tangent;
	Vec2<float32>			texCoords;
	Vec4<uint8, false>		color;
};

struct MeshData
{
	friend class RenderBatch;
protected:
	/// CPU buffers
	Array<VertexData>	vertexBuffer;
	Array<uint32>		indexBuffer;
	Array<uint32>		depthOnlyIndexBuffer;

public:
	/// Default constructor
	FORCE_INLINE MeshData(sizet numVertices = 1024) :
		vertexBuffer(numVertices),
		indexBuffer(numVertices),
		depthOnlyIndexBuffer(numVertices) {}
	
	/// Add vertex
	FORCE_INLINE void addVertex(const VertexData & v)
	{
		indexBuffer += vertexBuffer.getCount();
		vertexBuffer += v;
	};
};