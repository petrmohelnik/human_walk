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

void WeightedModel::addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord)
{
	v.push_back(vertex);
	n.push_back(normal);
	t.push_back(texCoord);
	weights.push_back(std::vector<float>());
	jointIndices.push_back(std::vector<int>());
}

void WeightedModel::addWeight(int i, float w, int j)
{
	weights[i].push_back(w);
	jointIndices[i].push_back(j);
}