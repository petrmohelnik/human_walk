#ifndef ANIMATION_TECHNIQUE_H
#define ANIMATION_TECHNIQUE_H

#include <GL/glew.h>
#include <vector>
#include "BasicTechnique.h"

class AnimationTechnique : public BasicTechnique
{
private:
	GLuint skinningMatrixUniform;
	std::vector<glm::mat4> skinningMatrices;
public:
	void init(WeightedMesh &m, GLuint p);
	void draw();
	void setSkinningMatrices(std::vector<glm::mat4> &m);
};

#endif //ANIMATION_TECHNIQUE_H