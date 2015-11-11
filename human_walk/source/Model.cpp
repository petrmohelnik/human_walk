#include "Mesh.h"

void Mesh::addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord)
{
	v.push_back(vertex);
	n.push_back(normal);
	t.push_back(texCoord);
}

float *Mesh::getVertices()
{
	return &v[0].x;
}

float *Mesh::getNormals()
{
	return &n[0].x;
}

float *Mesh::getTexCoords()
{
	return &t[0].x;
}

int Mesh::getSize()
{
	return v.size();
}

void WeightedMesh::addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord, glm::vec4 w, glm::ivec4 j)
{
	v.push_back(vertex);
	n.push_back(normal);
	t.push_back(texCoord);
	weights.push_back(w);
	jointIndices.push_back(j);
}

void WeightedMesh::initWeightVectors()
{
	weights.reserve(getSize());
	jointIndices.reserve(getSize());
}

void WeightedMesh::addWeight(int i, glm::vec4 w, glm::ivec4 j)
{
	weights.push_back(w);
	jointIndices.push_back(j);
}

float *WeightedMesh::getWeights()
{
	return &weights[0].x;
}

int *WeightedMesh::getJointIndices()
{
	return &jointIndices[0].x;
}

void WeightedMesh::setBindMatrix(glm::mat4 m)
{
	bindMatrix = m;
}

glm::mat4 WeightedMesh::getBindMatrix()
{
	return bindMatrix;
}