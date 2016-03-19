#ifndef RIGGED_MODEL_RENDERER_H
#define RIGGED_MODEL_RENDERER_H

#include <memory>
#include <iostream>
#include <vector>
#include "Renderer.h"
#include "Model.h"
#include "Skeleton.h"
#include "AnimationTechnique.h"
#include "Camera.h"
#include "Light.h"

class RiggedModelRenderer : public Renderer
{
private:
	std::shared_ptr<Skeleton> skeleton;
	std::vector<std::shared_ptr<AnimationTechnique> > technique;
	std::vector<glm::mat4> bindMatrix;
	glm::vec3 pos;
	std::vector<glm::mat4> skinningMat;
	std::vector<glm::mat3> TISkinningMat;
public:
	RiggedModelRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s);
	bool initRenderer(Model &m, GLuint p);
	void render(Camera &cam, const std::vector<Light> &lights, const glm::vec3 ambientLight);
};

#endif //RIGGED_MODEL_RENDERER_H