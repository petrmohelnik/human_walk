#ifndef RENDERED_OBJECT_H
#define RENDERED_OBJECT_H

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>
class Model;
class Camera;
struct Light;

class RenderedObject
{
protected:
public:
	virtual bool initRenderer(Model &m, GLuint p) = 0;
	virtual void display(Camera &cam, std::vector<Light> &lights, glm::vec3 ambientLight) = 0;
};

#endif //RENDERED_OBJECT_H