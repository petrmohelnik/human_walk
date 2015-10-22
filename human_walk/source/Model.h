#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <glm/glm.hpp>

class Model
{
private:
	std::vector<glm::vec3> v;
	std::vector<glm::vec3> n;
	std::vector<glm::vec2> t;
public:
	void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord);
	float *getVertices();
	float *getNormals();
	float *getTexCoords();
	int getSize();
};

#endif //MODEL_H