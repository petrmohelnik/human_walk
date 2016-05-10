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


void Scene::handleGui()
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
	/*case SDLK_t:
		skeleton->setPelvicTiltCoeff(skeleton->getPelvicTiltCoeff() + 0.2f);
		std::cout << "coeff: " << skeleton->getPelvicTiltCoeff() << std::endl;
		break;
	case SDLK_g:
		skeleton->setPelvicTiltCoeff(skeleton->getPelvicTiltCoeff() - 0.2f);
		std::cout << "coeff: " << skeleton->getPelvicTiltCoeff() << std::endl;
		break;
	case SDLK_y:
		skeleton->setPelvicTiltForwardCoeff(skeleton->getPelvicTiltForwardCoeff() + 0.2f);
		std::cout << "coeff: " << skeleton->getPelvicTiltForwardCoeff() << std::endl;
		break;
	case SDLK_h:
		skeleton->setPelvicTiltForwardCoeff(skeleton->getPelvicTiltForwardCoeff() - 0.2f);
		std::cout << "coeff: " << skeleton->getPelvicTiltForwardCoeff() << std::endl;
		break;
	case SDLK_r:
		skeleton->setPelvicRotationCoeff(skeleton->getPelvicRotationCoeff() + 0.2f);
		std::cout << "coeff: " << skeleton->getPelvicRotationCoeff() << std::endl;
		break;
	case SDLK_f:
		skeleton->setPelvicRotationCoeff(skeleton->getPelvicRotationCoeff() - 0.2f);
		std::cout << "coeff: " << skeleton->getPelvicRotationCoeff() << std::endl;
		break;
	case SDLK_e:
		skeleton->setPelvisLateralCoeff(skeleton->getPelvisLateralCoeff() + 0.2f);
		std::cout << "coeff: " << skeleton->getPelvisLateralCoeff() << std::endl;
		break;
	case SDLK_d:
		skeleton->setPelvisLateralCoeff(skeleton->getPelvisLateralCoeff() - 0.2f);
		std::cout << "coeff: " << skeleton->getPelvisLateralCoeff() << std::endl;
		break;
	case SDLK_w:
		skeleton->setPelvisVerticalCoeff(skeleton->getPelvisVerticalCoeff() + 0.2f);
		std::cout << "coeff: " << skeleton->getPelvisVerticalCoeff() << std::endl;
		break;
	case SDLK_s:
		skeleton->setPelvisVerticalCoeff(skeleton->getPelvisVerticalCoeff() - 0.2f);
		std::cout << "coeff: " << skeleton->getPelvisVerticalCoeff() << std::endl;
		break;*/
	/*case SDLK_q:
		skeleton->increaseMaxPelvisHeight(0.01f);
		std::cout << "height up" << std::endl;
		break;
	case SDLK_a:
		skeleton->increaseMaxPelvisHeight(-0.01f);
		std::cout << "height down" << std::endl;
		break;*/
	/*case SDLK_u:
		skeleton->increaseStepWidth(0.01f);
		std::cout << "step width up" << std::endl;
		break;
	case SDLK_j:
		skeleton->increaseStepWidth(-0.01f);
		std::cout << "step width down" << std::endl;
		break;
	case SDLK_i:
		skeleton->setShoulderCoeff(skeleton->getShoulderCoeff() + 0.2f);
		std::cout << "shoulder coeff up" << skeleton->getShoulderCoeff() << std::endl;
		break;
	case SDLK_k:
		skeleton->setShoulderCoeff(skeleton->getShoulderCoeff() - 0.2f);
		std::cout << "shoulder coeff down" << skeleton->getShoulderCoeff() << std::endl;
		break;
	case SDLK_o:
		skeleton->setElbowCoeff(skeleton->getElbowCoeff() + 0.2f);
		std::cout << "elbow coeff up" << skeleton->getElbowCoeff() << std::endl;
		break;
	case SDLK_l:
		skeleton->setElbowCoeff(skeleton->getElbowCoeff() - 0.2f);
		std::cout << "elbow coeff down" << skeleton->getElbowCoeff() << std::endl;
		break;*/
/*	case SDLK_p:
		skeleton->increaseArmWidth(0.05f);
		std::cout << "arm width up" << std::endl;
		break;
	case SDLK_SEMICOLON:
		skeleton->increaseArmWidth(-0.05f);
		std::cout << "arm width down" << std::endl;
		break;*/
	case SDLK_1:
		displaySkeleton = !displaySkeleton;
		break;
	case SDLK_2:
		displayRiggedModel = !displayRiggedModel;
		break;
	case SDLK_KP_PLUS:
		if (cameraMode == CAM_TRANS_ROT) {
			camera.translateRelative(glm::vec3(0.0, 0.0, -0.2));
		}
		break;
	case SDLK_KP_MINUS:
		if (cameraMode == CAM_TRANS_ROT) {
			camera.translateRelative(glm::vec3(0.0, 0.0, 0.2));
		}
		break;
	case SDLK_SPACE:
		pause = !pause;
		break;
	/*case SDLK_3:
		speedCoeff += 0.1f;
		break;
	case SDLK_4:
		speedCoeff -= 0.1f;
		if (speedCoeff < 0.0f)
			speedCoeff = 0.0f;
		break;*/
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
		camera.translateRelative(glm::vec3(0.0, 0.0, y * 0.2));
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
	if (walkingSpeed != prevWalkingSpeed) {
		stepLength = walkingSpeed / 3.0f + 0.3666666f;
		speedCoeff = walkingSpeed / (stepLength * 2.0f);
		pelvisLateralDisp = (-walkingSpeed / 50.0f + 0.049f) / 0.02f;
		pelvisVerticalDisp = (walkingSpeed / 50.0f + 0.016f) / 0.05f;
		stepWidth = -walkingSpeed / 32.0f + 0.135f;
		maxPelvisHeight = (-2.0f * stepLength / 20.0f + 0.055f);

		prevWalkingSpeed = walkingSpeed;
	}

	float minMaxPelvisHeight = (-2.0f * stepLength / 20.0f + 0.055f);
	minMaxPelvisHeight = (minMaxPelvisHeight > 0.0f) ? 0.0f : minMaxPelvisHeight;
	if (maxPelvisHeight > minMaxPelvisHeight)
		maxPelvisHeight = minMaxPelvisHeight;

	skeleton->setPelvicTiltCoeff(pelvicTilt);
	skeleton->setPelvicTiltForwardCoeff(pelvicTiltForward);
	skeleton->setPelvicRotationCoeff(pelvicRotation);
	skeleton->setPelvisLateralCoeff(pelvisLateralDisp);
	skeleton->setPelvisVerticalCoeff(pelvisVerticalDisp);
	skeleton->setShoulderCoeff(shoulderSwing);
	skeleton->setElbowCoeff(elbowSwing);
	skeleton->setStepLength(2.0f * stepLength);
	skeleton->setToeOutAngle(toeOutAngle);
	skeleton->setPelvisMidStanceOffset(pelvisMidStanceDisp);
	skeleton->setJointWeights(jointWeights);
	skeleton->setTimeSpeedCoeff(speedCoeff);
	skeleton->setRotationForward(rotationForward);
	skeleton->setStepWidth(stepWidth);
	skeleton->setArmWidth(armWidth);
	skeleton->setMaxPelvisHeight(maxPelvisHeight);
	skeleton->setMaxElbowExtension(maxElbowExtension);
	skeleton->setFootUpCoeff(footUpCoeff);

	if (!pause) {
		for (auto &u : updates)
		{
			u->onUpdate(dt);
		}
	}

	camera.translate(skeleton->getCameraPos() - prevCamPos);
	prevCamPos = skeleton->getCameraPos();
}

void MainScene::handleGui()
{
	ImGui::Begin("Predefined walks:");
	if (ImGui::Button("Wide")) {
		pelvicRotation = 1.5f;
		pelvicTilt = 2.0f;
		pelvisLateralDisp = 5.0f;
		pelvisVerticalDisp = 2.5f;
		pelvicTiltForward = 2.0f;
		stepWidth = 0.2f;
		footUpCoeff = 0.0f;
		elbowSwing = 1.0f;
		shoulderSwing = 1.0f;
	} ImGui::SameLine();
	if (ImGui::Button("Narrow")) {
		pelvicTilt = 2.5f;
		stepWidth = 0.0f;
		pelvisLateralDisp = 1.8f;
		pelvicRotation = 0.8f;
		rotationForward = -0.05f;
		elbowSwing = 1.3f;
		shoulderSwing = 1.0f;
		maxElbowExtension = 0.2f;
		footUpCoeff = 0.0f;
	} ImGui::SameLine();
	if (ImGui::Button("Careful")) {
		pelvicRotation = 2.5f;
		pelvisLateralDisp = 2.5f;
		maxPelvisHeight = -0.04f;
		pelvisVerticalDisp = 2.5f;
		pelvicTiltForward = 5.0f;
		pelvisMidStanceDisp = -0.075f;
		rotationForward = -0.09f;
		stepLength = 0.9f;
		footUpCoeff = 0.2f;
		speedCoeff = 0.6f;
		elbowSwing = 1.5f;
		shoulderSwing = 1.5f;
		maxElbowExtension = 0.68f;
	} ImGui::SameLine();
	if (ImGui::Button("Zombie")) {
		pelvicRotation = 1.5f;
		pelvicTilt = 0.4f;
		pelvisLateralDisp = 3.0f;
		maxPelvisHeight = -0.03f;
		pelvisVerticalDisp = 0.3f;
		pelvicTiltForward = 3.0f;
		pelvisMidStanceDisp = -0.1f;
		rotationForward = 0.15f;
		stepLength = 0.4f;
		stepWidth = 0.15f;
		toeOutAngle = -0.1f;
		footUpCoeff = 0.0f;
		speedCoeff = 0.47f;
		shoulderSwing = 0.75f;
		elbowSwing = 0.15f;
		armWidth = 0.15f;
		maxElbowExtension = 0.8f;
	} ImGui::SameLine();
	if (ImGui::Button("Astronaut")) {
		pelvicRotation = 0.24f;
		pelvicTilt = 0.12f;
		pelvisLateralDisp = 1.5f;
		maxPelvisHeight = -0.00f;
		pelvisVerticalDisp = 0.9f;
		pelvicTiltForward = 1.0f;
		pelvisMidStanceDisp = 0.00f;
		rotationForward = 0.08f;
		stepLength = 0.48f;
		stepWidth = 0.16f;
		toeOutAngle = 0.0f;
		footUpCoeff = 0.0f;
		speedCoeff = 0.25f;
		shoulderSwing = 0.425f;
		elbowSwing = 0.69f;
		armWidth = 0.2f;
		maxElbowExtension = 0.3f;
	} ImGui::SameLine();
	if (ImGui::Button("reset")) {
		prevWalkingSpeed = 0.0f;
		walkingSpeed = 1.0f;
		speedCoeff = 0.7f;
		pelvicTilt = 1.0f;
		pelvicTiltForward = 1.0f;
		pelvicRotation = 1.0f;
		pelvisLateralDisp = 1.0f;
		pelvisVerticalDisp = 1.0f;
		shoulderSwing = 1.0f;
		elbowSwing = 1.0f;
		toeOutAngle = 0.1f;
		stepLength = 0.7f;
		pelvisMidStanceDisp = 0.0f;
		rotationForward = 0.0f;
		stepWidth = 0.1f;
		armWidth = 0.0f;
		maxPelvisHeight = 0.0f;
		maxElbowExtension = 0.5f;
		footUpCoeff = 0.0f;
	} ImGui::SameLine();
	ImGui::End();

	ImGui::Begin("Set parameters based on walking speed:");
	ImGui::SliderFloat("Walking speed", &walkingSpeed, 0.4f, 1.6f);
	ImGui::End();

	ImGui::Begin("Determinants of gait:");
	ImGui::SliderFloat("Pelvic rotation", &pelvicRotation, 0.0f, 5.0f);
	ImGui::SliderFloat("Pelvic tilt", &pelvicTilt, 0.0f, 5.0f);
	ImGui::SliderFloat("Lateral displacement", &pelvisLateralDisp, 0.0f, 5.0f);
	ImGui::SliderFloat("Pelvic height", &maxPelvisHeight, -0.5f, 0.0f);
	ImGui::End();

	ImGui::Begin("Secondary parameters:");
	ImGui::SliderFloat("Vertical displacement", &pelvisVerticalDisp, 0.0f, 5.0f);
	ImGui::SliderFloat("Trunk flexion", &pelvicTiltForward, 0.0f, 5.0f);
	ImGui::SliderFloat("Midstance displacement", &pelvisMidStanceDisp, -0.1f, 0.1f);
	ImGui::SliderFloat("Trunk bend", &rotationForward, -0.25f, 0.25f);
	ImGui::Separator();
	ImGui::SliderFloat("Step length", &stepLength, 0.3f, 0.9f);
	ImGui::SliderFloat("Step width", &stepWidth, 0.0f, 0.3f);
	ImGui::SliderFloat("Toe out angle", &toeOutAngle, -0.25f, 0.25f);
	ImGui::SliderFloat("Knee lifting", &footUpCoeff, 0.0f, 0.25f);
	ImGui::SliderFloat("Step speed", &speedCoeff, 0.0f, 2.0f);
	ImGui::Separator();
	ImGui::SliderFloat("Shoulder swing", &shoulderSwing, 0.0f, 2.0f);
	ImGui::SliderFloat("Elbow swing", &elbowSwing, 0.0f, 2.0f);
	ImGui::SliderFloat("Arm width", &armWidth, -0.2f, 0.5f);
	ImGui::SliderFloat("Elbow extension", &maxElbowExtension, 0.0f, 0.8f);
	ImGui::Separator();
	ImGui::SliderFloat("Hip weight", &jointWeights.x, 0.0f, 10.0f);
	ImGui::SliderFloat("Knee weight", &jointWeights.y, 0.0f, 10.0f);
	ImGui::SliderFloat("Ankle weight", &jointWeights.z, 0.0f, 10.0f);
	ImGui::End();

	ImGui::Begin("Display and scene settings:");
	ImGui::Checkbox("Display Skeleton", &displaySkeleton); ImGui::SameLine();
	ImGui::Checkbox("Display Model", &displayRiggedModel);
	ImGui::Checkbox("Pause", &pause);

	if (ImGui::Button("Next Terrain")) {
		int prev = activeTerrain;
		activeTerrain++;
		activeTerrain = activeTerrain >= terrain.size() ? 0 : activeTerrain;
		for (unsigned int i = 0; i < objects.size(); i++) {
			if (objects[i] == terrainRenderer[prev])
				objects[i] = terrainRenderer[activeTerrain];
		}
		skeleton->setTerrain(terrain[activeTerrain]);
	}
	ImGui::End();

	ImGui::Begin("Interesting curves:");
	skeleton->getHeelSwingCurvePoints(heelSwingCurve);
	std::vector<float>::iterator min = std::min_element(heelSwingCurve.begin(), heelSwingCurve.end());
	std::vector<float>::iterator max = std::max_element(heelSwingCurve.begin(), heelSwingCurve.end());
	ImGui::PlotLines("Heel swing height", &heelSwingCurve[0], heelSwingCurve.size(), 0, "", *min, *max, ImVec2(0, 80));

	skeleton->getPelvisVerticalCurvePoints(pelvisVerticalCurve);
	min = std::min_element(pelvisVerticalCurve.begin(), pelvisVerticalCurve.end());
	max = std::max_element(pelvisVerticalCurve.begin(), pelvisVerticalCurve.end());
	ImGui::PlotLines("Pelvis vertical trajectory", &pelvisVerticalCurve[0], pelvisVerticalCurve.size(), 0, "", *min, *max, ImVec2(0, 80));

	skeleton->getPelvisSpeedCurvePoints(pelvisSpeed1Curve);
	min = std::min_element(pelvisSpeed1Curve.begin(), pelvisSpeed1Curve.end());
	max = std::max_element(pelvisSpeed1Curve.begin(), pelvisSpeed1Curve.end());
	ImGui::PlotLines("Pelvis displacement", &pelvisSpeed1Curve[0], pelvisSpeed1Curve.size(), 0, "", *min, *max, ImVec2(0, 200));
}