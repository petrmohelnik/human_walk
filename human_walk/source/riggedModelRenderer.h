#ifndef RIGGED_MODEL_RENDERER_H
#define RIGGED_MODEL_RENDERER_H

#include <memory>
#include <iostream>
#include "RenderedObject.h"
#include "Model.h"
#include "Skeleton.h"
#include "AnimationRenderer.h"
#include "Scene.h"
#include "Camera.h"

class RiggedModelRenderer : public RenderedObject
{
private:
	std::shared_ptr<Skeleton> skeleton;
	std::shared_ptr<AnimationRenderer> renderer;
	glm::vec3 pos;
	glm::mat4 bindMatrix;
public:
	RiggedModelRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s);
	bool initRenderer(Model &m, GLuint p);
	void display(Camera &cam, std::vector<Light> &lights, glm::vec3 ambientLight);
};

#endif //RIGGED_MODEL_RENDERER_H