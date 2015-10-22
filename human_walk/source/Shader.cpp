#include "Shader.h"

bool Shader::compileShader(const char *file, GLuint shaderType, const char *name, GLuint &s)
{
	GLuint shader;

	shader = glCreateShader(shaderType);
	glShaderSource(shader, 1, &file, NULL);
	glCompileShader(shader);

	int compiled;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	int log_length;
	glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);

	if (log_length > 1) {
		char *info_log;
		if (!(info_log = new(std::nothrow) char[log_length + 1]))
			return false;

		glGetShaderInfoLog(shader, log_length, &log_length, info_log);
		std::cout << "GLSL compiler " << name << ": " << info_log << std::endl;
	}

	s = shader;

	return compiled != 0;
}

bool Shader::linkProgram(GLuint vertexShader, GLuint fragmentShader, const char *name)
{
	GLuint program = glCreateProgram();
	
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	glLinkProgram(program);

	int linked;
	glGetProgramiv(program, GL_LINK_STATUS, &linked);
	int length;
	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);

	if (length > 1) {
		char *info_log;
		if (!(info_log = new(std::nothrow) char[length + 1]))
			return false;

		glGetProgramInfoLog(program, length, &length, info_log);
		std::cout << "GLSL linker " << name << ": " << info_log << std::endl;
		delete[] info_log;
	}
	
	programs.push_back(program);
	names.push_back(name);

	return linked != 0;
}

GLuint Shader::getProgram(const char *name)
{
	for (unsigned int i = 0; i < names.size(); i++) {
		if (names[i].compare(name) == 0)
			return programs[i];
	}

	return -1;
}