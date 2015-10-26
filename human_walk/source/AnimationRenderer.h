#ifndef ANIMATION_RENDERER_H
#define ANIMATION_RENDERER_H

#include <GL/glew.h>
#include <vector>
#include "BasicRenderer.h"

class AnimationRenderer : public BasicRenderer
{
private:
	GLuint skinningMatrixUniform;
	std::vector<glm::mat4> skinningMatrices;
public:
	void init(WeightedModel &m, GLuint p);
	void render();
	void setSkinningMatrices(std::vector<glm::mat4> &m);
};

#endif //ANIMATION_RENDERER_H