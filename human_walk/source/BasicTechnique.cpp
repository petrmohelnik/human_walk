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
	mUniform = glGetUniformLocation(program, "m");
	ti_mUniform = glGetUniformLocation(program, "ti_m");
	viewPosUniform = glGetUniformLocation(program, "viewPos");
	lightPosUniform = glGetUniformLocation(program, "lightPos");
	ambientLightUniform = glGetUniformLocation(program, "ambientLight");
	texDifSamplerUniform = glGetUniformLocation(program, "texDifSampler");

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

	glGenTextures(1, &texDif);
	glBindTexture(GL_TEXTURE_2D, texDif);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m.getMaterial()->getDifTex().width, m.getMaterial()->getDifTex().height, 0, GL_RGBA, GL_UNSIGNED_BYTE, &(m.getMaterial()->getDifTex().tex[0]));
	glGenerateMipmap(GL_TEXTURE_2D);
}

void BasicTechnique::draw()
{
	glUseProgram(program);

	glm::mat4 mvp = p * v * m;

	glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, glm::value_ptr(mvp));
	glUniformMatrix3fv(mUniform, 1, GL_FALSE, glm::value_ptr(glm::mat3(m)));
	glUniformMatrix3fv(ti_mUniform, 1, GL_FALSE, glm::value_ptr(glm::mat3(ti_m)));
	glUniform3f(viewPosUniform, -viewPos.x, -viewPos.y, -viewPos.z);
	glUniform3f(lightPosUniform, lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(ambientLightUniform, ambientLight.x, ambientLight.y, ambientLight.z);
	glUniform1i(texDifSamplerUniform, texDifSampler);
	
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, indices);
	glBindVertexArray(0);
}

void BasicTechnique::setM(glm::mat4 mat)
{
	m = mat;
	ti_m = glm::transpose(glm::inverse(mat));
}

void BasicTechnique::setV(glm::mat4 mat)
{
	v = mat;
}

void BasicTechnique::setP(glm::mat4 mat)
{
	p = mat;
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

void BasicTechnique::bindTexDif(int t)
{
	glActiveTexture(GL_TEXTURE0 + t);
	glBindTexture(GL_TEXTURE_2D, texDif);
	texDifSampler = t;
}