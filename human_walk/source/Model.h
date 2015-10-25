#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <glm/glm.hpp>

class Model
{
protected:
	std::vector<glm::vec3> v;
	std::vector<glm::vec3> n;
	std::vector<glm::vec2> t;
public:
	virtual void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord);
	float *getVertices();
	float *getNormals();
	float *getTexCoords();
	int getSize();
};

class WeightedModel : public Model
{
private:
	std::vector<glm::vec4> weights;
	std::vector<glm::ivec4> jointIndices;
public:
	void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord, glm::vec4 w, glm::ivec4 j);
	void initWeightVectors();
	void addWeight(int i, glm::vec4 w, glm::ivec4 j);
	float *getWeights();
	int *getJointIndices();
};

#endif //MODEL_H