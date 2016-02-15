#include "RiggedModelRenderer.h"

RiggedModelRenderer::RiggedModelRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s)
{
	pos = p;
	skeleton = s;
}

bool RiggedModelRenderer::initRenderer(Model &m, GLuint p)
{
	if (m.getMeshesSize() < 1) {
		std::cout << "ERROR: Model is empty!";
		return false;
	}

	std::vector<std::shared_ptr<Mesh> > meshes = m.getMeshes();
	for (int i = 0; i < m.getMeshesSize(); i++)
	{
		std::shared_ptr<AnimationTechnique> ptr(new AnimationTechnique);
		technique.push_back(ptr);
		WeightedMesh *wm = (WeightedMesh *)(meshes[i].get());
//		bindMatrix.push_back(wm->getBindMatrix());
		technique[i]->init(*wm, p);
	}

	return true;
}

void RiggedModelRenderer::render(Camera &cam, std::vector<Light> &lights, glm::vec3 ambientLight)
{
	//skeleton->countGlobalMatrices();

	for (unsigned int i = 0; i < technique.size(); i++)
	{
		if (lights.size() > 0)
			technique[i]->setLightPos(lights[0].pos);
		technique[i]->setAmbientLight(ambientLight);
		technique[i]->setP(cam.getProjection());
		technique[i]->setV(cam.getView());
		technique[i]->setViewPos(cam.getPos());
		technique[i]->setSkinningMatrices(skeleton->getSkinningMatrices(), skeleton->getTISkinningMatrices());
		technique[i]->bindTexDif(0);

		glm::mat4 skeletonM = skeleton->getRootTransformMatrix();
		skeletonM = glm::translate(skeletonM, pos);// * bindMatrix;

		technique[i]->setM(skeletonM);

		technique[i]->draw();
	}
}