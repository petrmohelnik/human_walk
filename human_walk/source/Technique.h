#ifndef TECHNIQUE_H
#define TECHNIQUE_H

#include <GL/glew.h>
#include <glm/glm.hpp>
class Mesh;

class Technique
{
protected:
	GLuint vao;
	int indices; //amount of indices in vao
	GLuint program;
public:
	virtual void draw(Mesh &m, GLuint p) = 0;
	virtual void render() = 0;
};

#endif //TECHNIQUE_H