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

void WeightedModel::addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord, glm::vec4 w, glm::ivec4 j)
{
	v.push_back(vertex);
	n.push_back(normal);
	t.push_back(texCoord);
	weights.push_back(w);
	jointIndices.push_back(j);
}

void WeightedModel::initWeightVectors()
{
	weights.reserve(getSize());
	jointIndices.reserve(getSize());
}

void WeightedModel::addWeight(int i, glm::vec4 w, glm::ivec4 j)
{
	weights.push_back(w);
	jointIndices.push_back(j);
}

float *WeightedModel::getWeights()
{
	return &weights[0].x;
}

int *WeightedModel::getJointIndices()
{
	return &jointIndices[0].x;
}

void WeightedModel::setBindMatrix(glm::mat4 m)
{
	bindMatrix = m;
}

glm::mat4 WeightedModel::getBindMatrix()
{
	return bindMatrix;
}