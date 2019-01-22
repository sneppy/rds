#pragma once

#include "coremin.h"
#include "gldrv/gldrv.h"
#include "mesh.h"

struct RenderBatch
{
protected:
	/// Array of meshes in this render batch
	Array<MeshData*> meshes;

	/// Vertex buffer name
	uint32 vbo;

	/// Index buffer name;
	uint32 ibo;

public:
	/// Create empty
	FORCE_INLINE RenderBatch() :
		meshes(4),
		vbo(0),
		ibo(0) {}

	/// Register new mesh
	FORCE_INLINE void addMesh(MeshData * mesh, const String & name)
	{
		meshes.push(mesh);
	}

	/// Upload to server
	FORCE_INLINE void upload()
	{
		if (!vbo)	glGenBuffers(1, &vbo);
		if (!ibo)	glGenBuffers(1, &ibo);

		// Bind buffers
		glBindBuffer(GL_ARRAY_BUFFER, vbo),
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

		// Create unique buffer for batch
		uint32 vboCount = 0, iboCount = 0;
		for (const auto mesh : meshes)
		{
			vboCount += mesh->vertexBuffer.getCount(),
			iboCount += mesh->indexBuffer.getCount();
		}

		sizet
			vboSize = vboCount * sizeof(VertexData),
			iboSize = iboCount * sizeof(uint32);

		glBufferData(GL_ARRAY_BUFFER, vboSize, nullptr, GL_STATIC_DRAW);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, iboSize, nullptr, GL_STATIC_DRAW);

		// Submit data
		sizet vboOffset = 0, iboOffset = 0;
		for (const auto mesh : meshes)
		{
			const uint32
				vertexCount	= mesh->vertexBuffer.getCount(),
				indexCount	= mesh->indexBuffer.getCount();
				
			vboSize = vertexCount * sizeof(VertexData),
			iboSize = indexCount * sizeof(uint32);

			glBufferSubData(GL_ARRAY_BUFFER, vboOffset, vboSize, mesh->vertexBuffer.getData()),
			glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, iboOffset, iboSize, mesh->indexBuffer.getData());
		}
	}

	/// Bind buffers
	FORCE_INLINE void bind()
	{
		// Bind buffers
		glBindBuffer(GL_ARRAY_BUFFER, vbo),
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

		// Setup vertex attributes
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)offsetof(VertexData, position)),
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)offsetof(VertexData, normal));
		
		glEnableVertexAttribArray(0),
		glEnableVertexAttribArray(1);
	}

	FORCE_INLINE void draw()
	{
		uint64 indexOffset = 0;
		for (const auto mesh : meshes)
		{
			glDrawElements(GL_TRIANGLES, mesh->indexBuffer.getCount(), GL_UNSIGNED_INT, (void*)indexOffset);
			indexOffset += mesh->indexBuffer.getCount() * sizeof(uint32);
		}
	}
};