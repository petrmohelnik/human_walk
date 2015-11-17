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
	GLuint mUniform;
	GLuint ti_mUniform;
	GLuint viewPosUniform;
	GLuint lightPosUniform;
	GLuint ambientLightUniform;
	GLuint texDifSamplerUniform;
	GLuint texDif;
	glm::mat4 p;
	glm::mat4 m;
	glm::mat4 v;
	glm::mat4 ti_m;
	glm::vec3 viewPos;
	glm::vec3 lightPos;
	glm::vec3 ambientLight;
	int texDifSampler;
public:
	void init(Mesh &m, GLuint p);
	virtual void draw();
	void setM(glm::mat4 mat);
	void setV(glm::mat4 mat);
	void setP(glm::mat4 mat);
	void setViewPos(glm::vec3 v);
	void setLightPos(glm::vec3 p);
	void setAmbientLight(glm::vec3 a);
	void bindTexDif(int t);
};

#endif //BASIC_TECHNIQUE_H