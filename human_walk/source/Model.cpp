#include "Model.h"

void Model::addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord)
{
	v.push_back(vertex);
	n.push_back(normal);
	t.push_back(texCoord);
}

float *Model::getVertices()
{
	return &v[0].x;
}

float *Model::getNormals()
{
	return &n[0].x;
}

float *Model::getTexCoords()
{
	return &t[0].x;
}

int Model::getSize()
{
	return v.size();
}