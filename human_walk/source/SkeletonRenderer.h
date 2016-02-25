#ifndef SKELETON_RENDERER_H
#define SKELETON_RENDERER_H

#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include "Renderer.h"
#include "BasicTechnique.h"
#include "Model.h"
#include "Camera.h"
#include "Light.h"
#include "Skeleton.h"

class SkeletonRenderer : public Renderer
{
private:
	std::shared_ptr<BasicTechnique> technique;
	glm::vec3 pos;
	std::shared_ptr<Skeleton> skeleton;
public:
	SkeletonRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s);
	bool initRenderer(Model &m, GLuint p);
	void render(Camera &cam, const std::vector<Light> &lights, const glm::vec3 ambientLight);
};

#endif //SKELETON_RENDERER_H