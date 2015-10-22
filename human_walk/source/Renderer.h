#ifndef RENDERER_H
#define RENDERER_H

#include <GL/glew.h>
#include <glm/glm.hpp>
class Model;

class Renderer
{
protected:
	GLuint vao;
	int indices; //amount of indices in vao
	GLuint program;
public:
	virtual void init(Model &m, GLuint p) = 0;
	virtual void render() = 0;
};

#endif //RENDERER_H