#ifndef BASIC_RENDERER_H
#define BASIC_RENDERER_H

#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include "BasicTechnique.h"
#include "Renderer.h"
#include "Model.h"
#include "Camera.h"
#include "light.h"

class BasicRenderer : public Renderer
{
private:
	std::shared_ptr<BasicTechnique> technique;
	glm::vec3 pos;
public:
	BasicRenderer(const glm::vec3 &position);
	bool initRenderer(Model &m, GLuint p);
	void render(Camera &cam, const std::vector<Light> &lights, const glm::vec3 ambientLight);
};

#endif //BASIC_RENDERER_H