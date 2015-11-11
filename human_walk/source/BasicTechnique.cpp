#include "BasicTechnique.h"

void BasicTechnique::init(Mesh &m, GLuint p)
{
	program = p;

	GLuint vbo[3];
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(3, vbo);

	indices = m.getSize();

	mvpUniform = glGetUniformLocation(program, "mvp");
	mvUniform = glGetUniformLocation(program, "mv");
	ti_mvUniform = glGetUniformLocation(program, "ti_mv");
	viewPosUniform = glGetUniformLocation(program, "viewPos");
	lightPosUniform = glGetUniformLocation(program, "lightPos");
	ambientLightUniform = glGetUniformLocation(program, "ambientLight");

	GLint attr = glGetAttribLocation(program, "v_pos");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float) * 3, m.getVertices(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	attr = glGetAttribLocation(program, "v_norm");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float) * 3, m.getNormals(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	attr = glGetAttribLocation(program, "v_texCoord");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float) * 2, m.getTexCoords(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	glBindVertexArray(0);
}

void BasicTechnique::draw()
{
	glUseProgram(program);

	glm::mat4 mvp = p * mv;

	glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, glm::value_ptr(mvp));
	glUniformMatrix3fv(mvUniform, 1, GL_FALSE, glm::value_ptr(glm::mat3(mv)));
	glUniformMatrix3fv(ti_mvUniform, 1, GL_FALSE, glm::value_ptr(glm::mat3(ti_mv)));
	glUniform3f(viewPosUniform, -viewPos.x, -viewPos.y, -viewPos.z);
	glUniform3f(lightPosUniform, lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(ambientLightUniform, ambientLight.x, ambientLight.y, ambientLight.z);
	
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, indices);
	glBindVertexArray(0);
}

void BasicTechnique::setMv(glm::mat4 m)
{
	mv = m;
	ti_mv = glm::transpose(glm::inverse(m));
}

void BasicTechnique::setP(glm::mat4 m)
{
	p = m;
}

void BasicTechnique::setViewPos(glm::vec3 v)
{
	viewPos = v;
}

void BasicTechnique::setLightPos(glm::vec3 p)
{
	lightPos = p;
}

void BasicTechnique::setAmbientLight(glm::vec3 a)
{
	ambientLight = a;
}