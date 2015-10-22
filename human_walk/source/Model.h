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
	std::vector<std::vector<float> > weights;
	std::vector<std::vector<int> > jointIndices;
public:
	void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord);
	void addWeight(int i, float w, int j);
};

#endif //MODEL_H