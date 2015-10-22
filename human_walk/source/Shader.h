#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <string>
#include <vector>
#include <iostream>

class Shader
{
private:
	std::vector<std::string> names;
	std::vector<GLuint> programs;
public:
	bool compileShader(const char *file, GLuint shaderType, const char *name, GLuint &s);
	bool linkProgram(GLuint vertexShader, GLuint fragmentShader, const char *name);
	GLuint getProgram(const char *name);
};

#endif //SHADER_H