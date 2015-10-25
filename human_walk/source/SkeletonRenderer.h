#ifndef SKELETON_RENDERER_H
#define SKELETON_RENDERER_H

#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include "RenderedObject.h"
#include "BasicRenderer.h"
#include "Model.h"
#include "Camera.h"
#include "Light.h"
#include "Skeleton.h"

class SkeletonRenderer : public RenderedObject
{
private:
	std::shared_ptr<BasicRenderer> renderer;
	glm::vec3 pos;
	std::shared_ptr<Skeleton> skeleton;
public:
	SkeletonRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s);
	bool initRenderer(Model &m, GLuint p);
	void display(Camera &cam, std::vector<Light> &lights, glm::vec3 ambientLight);
};

#endif //SKELETON_RENDERER_H