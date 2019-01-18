#version 430 core

// Transform matrix
uniform mat4 modelMatrix;
uniform mat4 viewProjectionMatrix;

/// Vertex data
layout(location = 0) in vec4 vPosition;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec3 vTangent;

/// Out vertex data
out vec3 wsPosition;
out vec3 wsNormal;
out vec3 wsTangent;

void main()
{
	wsPosition = (modelMatrix * vPosition).xyz;
	wsNormal = normalize(transpose(inverse(mat3(modelMatrix))) * vNormal);
	wsTangent = vTangent.xyz;

	gl_Position = viewProjectionMatrix * vec4(wsPosition, 1.f);
}