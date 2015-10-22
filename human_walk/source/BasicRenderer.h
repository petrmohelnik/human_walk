#ifndef BASIC_RENDERER_H
#define BASIC_RENDERER_H

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include "Renderer.h"
#include "Model.h"

class BasicRenderer : public Renderer
{
private:
	GLuint mvpUniform;
	GLuint mvUniform;
	GLuint ti_mvUniform;
	GLuint viewPosUniform;
	GLuint lightPosUniform;
	GLuint ambientLightUniform;
	glm::mat4 p;
	glm::mat4 mv;
	glm::mat4 ti_mv;
	glm::vec3 viewPos;
	glm::vec3 lightPos;
	glm::vec3 ambientLight;
public:
	void init(Model &m, GLuint p);
	void render();
	void setMv(glm::mat4 m);
	void setP(glm::mat4 m);
	void setViewPos(glm::vec3 v);
	void setLightPos(glm::vec3 p);
	void setAmbientLight(glm::vec3 a);
};

#endif //BASIC_RENDERER_H