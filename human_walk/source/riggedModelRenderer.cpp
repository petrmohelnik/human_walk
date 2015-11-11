#include "RiggedModelRenderer.h"

RiggedModelRenderer::RiggedModelRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s)
{
	technique.reset(new AnimationTechnique);
	pos = p;
	skeleton = s;
}

bool RiggedModelRenderer::initRenderer(Model &m, GLuint p)
{
	if (m.getSize() < 1) {
		std::cout << "ERROR: Model is empty!";
		return false;
	}
	WeightedModel &wm = (WeightedModel &)m;
	bindMatrix = wm.getBindMatrix();
	renderer->init(wm, p);
	return true;
}

void RiggedModelRenderer::display(Camera &cam, std::vector<Light> &lights, glm::vec3 ambientLight)
{
	skeleton->countGlobalMatrices();

	if (lights.size() > 0)
		renderer->setLightPos(lights[0].pos);
	renderer->setAmbientLight(ambientLight);
	renderer->setP(cam.getProjection());
	renderer->setViewPos(cam.getPos());
	renderer->setSkinningMatrices(skeleton->getSkinningMatrices());

	glm::mat4 skeletonMv = skeleton->getRootTransformMatrix();
	skeletonMv = glm::translate(skeletonMv, pos);// * bindMatrix;

	renderer->setMv(skeletonMv);

	renderer->render();
}