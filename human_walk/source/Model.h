#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <glm/glm.hpp>
#include <memory>

class Material
{
public:

private:

};

class Mesh
{
protected:
	std::vector<glm::vec3> v;
	std::vector<glm::vec3> n;
	std::vector<glm::vec2> t;
	std::shared_ptr<Material> m;
public:
	virtual void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord);
	float *getVertices();
	float *getNormals();
	float *getTexCoords();
	int getSize();
};

class WeightedMesh : public Mesh
{
private:
	std::vector<glm::vec4> weights;
	std::vector<glm::ivec4> jointIndices;
	glm::mat4 bindMatrix;
public:
	void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord, glm::vec4 w, glm::ivec4 j);
	void initWeightVectors();
	void addWeight(int i, glm::vec4 w, glm::ivec4 j);
	float *getWeights();
	int *getJointIndices();
	void setBindMatrix(glm::mat4 m);
	glm::mat4 getBindMatrix();
};

class Model
{
private:
	std::vector<std::shared_ptr<Mesh> > meshes;
public:

};

#endif //MODEL_H