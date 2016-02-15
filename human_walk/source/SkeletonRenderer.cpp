#include "SkeletonRenderer.h"

SkeletonRenderer::SkeletonRenderer(glm::vec3 p, std::shared_ptr<Skeleton> s)
{
	technique.reset(new BasicTechnique);
	pos = p;
	skeleton = s;
}

bool SkeletonRenderer::initRenderer(Model &m, GLuint p)
{
	if (m.getMeshesSize() < 1) {
		std::cout << "ERROR: Model is empty!";
			return false;
	}

	technique->init(*(m.getMeshes()[0]), p);
	return true;
}

void SkeletonRenderer::render(Camera &cam, std::vector<Light> &lights, glm::vec3 ambientLight)
{
	glClear(GL_DEPTH_BUFFER_BIT);

	if (lights.size() > 0)
		technique->setLightPos(lights[0].pos);
	technique->setAmbientLight(ambientLight);
	technique->setP(cam.getProjection());
	technique->setV(cam.getView());
	technique->setViewPos(cam.getPos());
	technique->bindTexDif(0);

	glm::mat4 skeletonM = skeleton->getRootTransformMatrix();
	skeletonM = glm::translate(skeletonM, pos);

	//skeleton->countGlobalMatrices();
	std::vector<glm::mat4> bones = skeleton->getScaledGlobalMatrices();
	//glm::mat4 modelview = glm::mat4(1.0f);

	/*for (unsigned int i = 0; i < bones.size(); i++)
	{
		bones[i] = glm::transpose(bones[i]);
	}*/

	for (unsigned int i = 0; i < bones.size(); i++)
	{
		//glm::mat4 modelview = glm::mat4(1.0f);
		//modelview = glm::translate(modelview, pos);
		//modelview = glm::translate(modelview, bones[i].pos);
		//modelview = glm::rotate(modelview, bones[i].rot.w, glm::vec3(bones[i].rot));
		//modelview = glm::scale(modelview, bones[i].scale);
		//modelview *= (bones[i]);
		//glm::mat4 m;
		//if (i < bones.size() - 1)
		//	m = glm::scale(modelview, glm::vec3(glm::length(glm::vec3((modelview * bones[i + 1])[3] - modelview[3]))));
		//else
		//	m = modelview;
		technique->setM(skeletonM * bones[i]);

		technique->draw();
	}
}

