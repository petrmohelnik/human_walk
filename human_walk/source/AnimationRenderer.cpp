#include "AnimationRenderer.h"

void AnimationRenderer::init(WeightedModel &m, GLuint p)
{
	program = p;

	GLuint vbo[5];
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(5, vbo);

	indices = m.getSize();

	mvpUniform = glGetUniformLocation(program, "mvp");
	mvUniform = glGetUniformLocation(program, "mv");
	ti_mvUniform = glGetUniformLocation(program, "ti_mv");
	viewPosUniform = glGetUniformLocation(program, "viewPos");
	lightPosUniform = glGetUniformLocation(program, "lightPos");
	ambientLightUniform = glGetUniformLocation(program, "ambientLight");
	skinningMatrixUniform = glGetUniformLocation(program, "skinningMatrix");

	GLint attr = glGetAttribLocation(program, "v_pos");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float)* 3, m.getVertices(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	attr = glGetAttribLocation(program, "v_norm");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float)* 3, m.getNormals(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	attr = glGetAttribLocation(program, "v_texCoord");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float)* 2, m.getTexCoords(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	attr = glGetAttribLocation(program, "v_weights");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(float)* 4, m.getWeights(), GL_STATIC_DRAW);
	glVertexAttribPointer(attr, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(attr);

	attr = glGetAttribLocation(program, "v_joints");
	glBindBuffer(GL_ARRAY_BUFFER, vbo[4]);
	glBufferData(GL_ARRAY_BUFFER, m.getSize() * sizeof(int)* 4, m.getJointIndices(), GL_STATIC_DRAW);
	glVertexAttribIPointer(attr, 4, GL_INT, 0, 0);
	glEnableVertexAttribArray(attr);

	glBindVertexArray(0);
}

void AnimationRenderer::render()
{
	glUseProgram(program);

	glm::mat4 mvp = p * mv;

	glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, glm::value_ptr(mvp));
	glUniformMatrix3fv(mvUniform, 1, GL_FALSE, glm::value_ptr(glm::mat3(mv)));
	glUniformMatrix3fv(ti_mvUniform, 1, GL_FALSE, glm::value_ptr(glm::mat3(ti_mv)));
	glUniform3f(viewPosUniform, -viewPos.x, -viewPos.y, -viewPos.z);
	glUniform3f(lightPosUniform, lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(ambientLightUniform, ambientLight.x, ambientLight.y, ambientLight.z);
	glUniformMatrix4fv(skinningMatrixUniform, skinningMatrices.size(), GL_FALSE, glm::value_ptr(skinningMatrices[0]));

	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, indices);
	glBindVertexArray(0);
}

void AnimationRenderer::setSkinningMatrices(std::vector<glm::mat4> &m)
{
	skinningMatrices = m;
}