#ifndef LIGHT_H
#define LIGHT_H

#include <glm/glm.hpp>

//dodelat vic veci
struct Light
{
	glm::vec3 pos;
	Light(glm::vec3 p) : pos(p) {}
};

#endif //LIGHT_H