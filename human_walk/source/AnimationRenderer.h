#ifndef ANIMATION_RENDERER_H
#define ANIMATION_RENDERER_H

#include <GL/glew.h>
#include <vector>
#include "BasicRenderer.h"

class AnimationRenderer : public BasicRenderer
{
private:
	GLuint skeletonInverseUniform;
	GLuint skeletonGlobalUniform;
	std::vector<glm::mat4> skeletonInverse;
	std::vector<glm::mat4> skeletonGlobal;
public:
	void init(WeightedModel &m, GLuint p);
	void render();
	void setSkeletonMatrices(std::vector<glm::mat4> &inverse, std::vector<glm::mat4> &global);
};

#endif //ANIMATION_RENDERER_H