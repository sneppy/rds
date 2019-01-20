#version 430 core

// Vertex data
in vec3 wsPosition;
in vec3 wsNormal;
in vec3 wsTangent;

// Out color
out vec4 fColor;

void main()
{
	// Fake light
	vec3 ambientLight = vec3(0.14f, 0.12f, 0.12f);
	vec3 lightDir = normalize(vec3(0.5f, -1.f, 0.2f));
	float lightIntensity = 1.2f;
	vec3 lightColor = vec3(0.8f, 0.75f, 0.75f) * lightIntensity;

	// Scattering light
	vec3 scatteringLight = (dot(wsNormal, -lightDir) * 0.5f + 0.5f) * lightColor + ambientLight;

	// Fragment color
	fColor = vec4(min(scatteringLight, 1.f), 1.f);

	fColor = vec4(max(normalize(wsPosition), 0.1f), 1.f);
}
