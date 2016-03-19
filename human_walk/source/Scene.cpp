#include "Scene.h"

void Scene::setName(const char *n)
{
	name.append(n);
}

void Scene::addObject(std::shared_ptr<Renderer> o)
{
	objects.push_back(o);
}

void Scene::addUpdate(std::shared_ptr<Update> u)
{
	updates.push_back(u);
}

void Scene::addLight(Light l)
{
	lights.push_back(l);
}

void Scene::setAmbientLight(glm::vec3 a)
{
	ambientLight = a;
}

void Scene::initCamera(float fov, int width, int height, float nearPlane, float farPlane, int mode)
{
	camera.init(fov, width, height, nearPlane, farPlane, mode);
}

Camera* Scene::getCamera()
{
	return &camera;
}

void Scene::render()
{
	for (unsigned int i = 0; i < objects.size(); i++)
	{
		objects[i]->render(camera, lights, ambientLight);
	}
}

void Scene::update(float dt)
{
	for (auto &u : updates)
	{
		u->onUpdate(dt);
	}
}

const char* Scene::getName()
{
	return name.c_str();
}

void Scene::handleSdlEvent(SDL_Event &event)
{

}

void MainScene::handleSdlEvent(SDL_Event &event)
{
	//Call proper event handlers
	switch (event.type)
	{
	case SDL_KEYDOWN:
		onKeyDown(event.key.keysym.sym);
		break;
	case SDL_MOUSEMOTION:
		onMouseMove(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel, event.motion.state);
		break;		
	case SDL_MOUSEWHEEL:
		onMouseWheel(event.wheel.x, event.wheel.y);
		break;
	}
}

void MainScene::onKeyDown(SDL_Keycode key)
{
	glm::vec3 dir;
	switch (key) {
	case SDLK_UP:
		if (cameraMode == CAM_ROT_TRANS) {
			dir = glm::normalize(glm::rotate(glm::vec3(0.0f, 0.0f, 1.0f), camera.getRotation().y, glm::vec3(1.0f, 0.0f, 0.0f)));
			dir = glm::normalize(glm::rotate(dir, camera.getRotation().x, glm::vec3(0.0f, 1.0f, 0.0f)));
			camera.translate(glm::vec3(dir.x, dir.y, -dir.z));
		}
		break;
	case SDLK_DOWN:
		if (cameraMode == CAM_ROT_TRANS) {
			dir = glm::normalize(glm::rotate(glm::vec3(0.0f, 0.0f, -1.0f), camera.getRotation().y, glm::vec3(1.0f, 0.0f, 0.0f)));
			dir = glm::normalize(glm::rotate(dir, camera.getRotation().x, glm::vec3(0.0f, 1.0f, 0.0f)));
			camera.translate(glm::vec3(dir.x, dir.y, -dir.z));
		}
		break;
	case SDLK_LEFT:
		if (cameraMode == CAM_ROT_TRANS) {
			dir = glm::normalize(glm::rotate(glm::vec3(1.0f, 0.0f, 0.0f), camera.getRotation().y, glm::vec3(1.0f, 0.0f, 0.0f)));
			dir = glm::normalize(glm::rotate(dir, camera.getRotation().x, glm::vec3(0.0f, 1.0f, 0.0f)));
			camera.translate(glm::vec3(-dir.x, dir.y, dir.z));
		}
		break;
	case SDLK_RIGHT:
		if (cameraMode == CAM_ROT_TRANS) {
			dir = glm::normalize(glm::rotate(glm::vec3(-1.0f, 0.0f, 0.0f), camera.getRotation().y, glm::vec3(1.0f, 0.0f, 0.0f)));
			dir = glm::normalize(glm::rotate(dir, camera.getRotation().x, glm::vec3(0.0f, 1.0f, 0.0f)));
			camera.translate(glm::vec3(-dir.x, dir.y, dir.z));
		}
		break;
	case SDLK_t:
		skeleton->setPelvicTiltCoeff(skeleton->getPelvicTiltCoeff() + 1.0f);
		std::cout << "coeff: " << skeleton->getPelvicTiltCoeff() << std::endl;
		break;
	case SDLK_g:
		skeleton->setPelvicTiltCoeff(skeleton->getPelvicTiltCoeff() - 1.0f);
		std::cout << "coeff: " << skeleton->getPelvicTiltCoeff() << std::endl;
		break;
	case SDLK_y:
		skeleton->setPelvicTiltForwardCoeff(skeleton->getPelvicTiltForwardCoeff() + 1.0f);
		std::cout << "coeff: " << skeleton->getPelvicTiltForwardCoeff() << std::endl;
		break;
	case SDLK_h:
		skeleton->setPelvicTiltForwardCoeff(skeleton->getPelvicTiltForwardCoeff() - 1.0f);
		std::cout << "coeff: " << skeleton->getPelvicTiltForwardCoeff() << std::endl;
		break;
	case SDLK_r:
		skeleton->setPelvicRotationCoeff(skeleton->getPelvicRotationCoeff() + 1.0f);
		std::cout << "coeff: " << skeleton->getPelvicRotationCoeff() << std::endl;
		break;
	case SDLK_f:
		skeleton->setPelvicRotationCoeff(skeleton->getPelvicRotationCoeff() - 1.0f);
		std::cout << "coeff: " << skeleton->getPelvicRotationCoeff() << std::endl;
		break;
	case SDLK_e:
		skeleton->setPelvisLateralCoeff(skeleton->getPelvisLateralCoeff() + 1.0f);
		std::cout << "coeff: " << skeleton->getPelvisLateralCoeff() << std::endl;
		break;
	case SDLK_d:
		skeleton->setPelvisLateralCoeff(skeleton->getPelvisLateralCoeff() - 1.0f);
		std::cout << "coeff: " << skeleton->getPelvisLateralCoeff() << std::endl;
		break;
	case SDLK_w:
		skeleton->setPelvisVerticalCoeff(skeleton->getPelvisVerticalCoeff() + 1.0f);
		std::cout << "coeff: " << skeleton->getPelvisVerticalCoeff() << std::endl;
		break;
	case SDLK_s:
		skeleton->setPelvisVerticalCoeff(skeleton->getPelvisVerticalCoeff() - 1.0f);
		std::cout << "coeff: " << skeleton->getPelvisVerticalCoeff() << std::endl;
		break;
	case SDLK_q:
		skeleton->increaseMaxPelvisHeight(0.01f);
		std::cout << "height up" << std::endl;
		break;
	case SDLK_a:
		skeleton->increaseMaxPelvisHeight(-0.01f);
		std::cout << "height down" << std::endl;
		break;
	case SDLK_u:
		skeleton->increaseStepWidth(0.01f);
		std::cout << "step width up" << std::endl;
		break;
	case SDLK_j:
		skeleton->increaseStepWidth(-0.01f);
		std::cout << "step width down" << std::endl;
		break;
	case SDLK_i:
		skeleton->setShoulderCoeff(skeleton->getShoulderCoeff() + 0.1f);
		std::cout << "shoulder coeff up" << skeleton->getShoulderCoeff() << std::endl;
		break;
	case SDLK_k:
		skeleton->setShoulderCoeff(skeleton->getShoulderCoeff() - 0.1f);
		std::cout << "shoulder coeff down" << skeleton->getShoulderCoeff() << std::endl;
		break;
	case SDLK_o:
		skeleton->setElbowCoeff(skeleton->getElbowCoeff() + 0.1f);
		std::cout << "elbow coeff up" << skeleton->getElbowCoeff() << std::endl;
		break;
	case SDLK_l:
		skeleton->setElbowCoeff(skeleton->getElbowCoeff() - 0.1f);
		std::cout << "elbow coeff down" << skeleton->getElbowCoeff() << std::endl;
		break;
	case SDLK_p:
		skeleton->increaseArmWidth(0.05f);
		std::cout << "arm width up" << std::endl;
		break;
	case SDLK_SEMICOLON:
		skeleton->increaseArmWidth(-0.05f);
		std::cout << "arm width down" << std::endl;
		break;
	case SDLK_1:
		displaySkeleton = !displaySkeleton;
		break;
	case SDLK_2:
		displayRiggedModel = !displayRiggedModel;
		break;
	}
}

void MainScene::onMouseMove(Sint32 x, Sint32 y, Sint32 xrel, Sint32 yrel, Uint32 state)
{
	if (state == SDL_BUTTON_RMASK) {
		camera.rotateX(xrel * 0.02f);
		camera.rotateY(yrel * 0.02f);
	}
}

void MainScene::onMouseWheel(Sint32 x, Sint32 y)
{
	if (cameraMode == CAM_TRANS_ROT) {
		camera.translateRelative(glm::vec3(0.0, 0.0, y * 0.1));
	}
}

void MainScene::setCameraMode(int mode)
{
	camera.setMode(mode);
	cameraMode = mode;
}

void MainScene::initCamera(float fov, int width, int height, float nearPlane, float farPlane, int mode)
{
	camera.init(fov, width, height, nearPlane, farPlane, mode);
	cameraMode = mode;
}

void MainScene::render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	for (unsigned int i = 0; i < objects.size(); i++)
	{
		if ((!displaySkeleton && objects[i] == skeletonRenderer) || (!displayRiggedModel && objects[i] == riggedModelRenderer))
			continue;

		objects[i]->render(camera, lights, ambientLight);
	}
}

void MainScene::update(float dt)
{
	for (auto &u : updates)
	{
		u->onUpdate(dt);
	}

	camera.translate(skeleton->getStaticRootPos() - prevCamPos);
	prevCamPos = skeleton->getStaticRootPos();
}