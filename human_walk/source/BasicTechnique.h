#ifndef BASIC_TECHNIQUE_H
#define BASIC_TECHNIQUE_H

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include "Technique.h"
#include "Model.h"

class BasicTechnique : public Technique
{
protected:
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
	void init(Mesh &m, GLuint p);
	virtual void draw();
	void setMv(glm::mat4 m);
	void setP(glm::mat4 m);
	void setViewPos(glm::vec3 v);
	void setLightPos(glm::vec3 p);
	void setAmbientLight(glm::vec3 a);
};

#endif //BASIC_TECHNIQUE_H