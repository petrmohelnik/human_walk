//#define _CRT_SECURE_NO_WARNINGS
#include <GL/glew.h>
#include <windows.h>
#include <iostream>
#include <memory>
#include "SDLHandler.h"
#include "FileSystem.h"
#include "SkeletonRenderer.h"
#include "Shader.h"
#include "Scene.h"
#include "Application.h"
#include "Model.h"
#include "riggedModelRenderer.h"
#include "BasicRenderer.h"
#ifdef main
#undef main //remove SDL's main() hook if it exists
#endif

#define W_WIDTH 1280
#define W_HEIGHT 720

using namespace std;

/* Use nvidia graphics card */
extern "C" {
	_declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}

int main(int argc, char **argv)
{
	//init SDL
	SDLHandler sdl(W_WIDTH, W_HEIGHT);
	if (!sdl.init())
		return -1;

	//init glew
	glewExperimental = GL_TRUE;
	GLenum result = glewInit();
	if (result != GLEW_OK) {
		cout << "ERROR " << result << ": Initializing glew\n";
		return -1;
	}

	if (GLEW_VERSION_1_1) {
		cout << "----------------------------------------------------------------\n";
		cout << "Graphics Successfully Initialized\n";
		cout << "OpenGL Info\n";
		cout << "    Version: " << glGetString(GL_VERSION) << endl;
		cout << "     Vendor: " << glGetString(GL_VENDOR) << endl;
		cout << "   Renderer: " << glGetString(GL_RENDERER) << endl;
		cout << "    Shading: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
		cout << "----------------------------------------------------------------\n";
	}
	else {
		printf("Unable to get any OpenGL version from GLEW!");
	}

	Application app;
	FileSystem f;

	//int maxUniformVectors;
	//glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &maxUniformVectors);

	Shader s;
	string strVs, strFs, strAnimationVs;
	if (!f.loadFile("resource/basic.vs", strVs)) { cin.get(); return -1; }
	if (!f.loadFile("resource/basic.fs", strFs)) { cin.get(); return -1; }
	if (!f.loadFile("resource/animation.vs", strAnimationVs))  { cin.get(); return -1; }
	GLuint vs, fs, animationVs;
	if (!s.compileShader(strVs.c_str(), GL_VERTEX_SHADER, "basic_vs", vs)) { cin.get(); return -1; }
	if (!s.compileShader(strAnimationVs.c_str(), GL_VERTEX_SHADER, "animation_vs", animationVs)) { cin.get(); return -1; }
	if (!s.compileShader(strFs.c_str(), GL_FRAGMENT_SHADER, "basic_fs", fs)) { cin.get(); return -1; }
	if (!s.linkProgram(vs, fs, "basic_program")) { cin.get(); return -1; }
	if (!s.linkProgram(animationVs, fs, "animation_program")) { cin.get(); return -1; }

	std::cout << "\nLoading models......\n" << std::endl;

	std::shared_ptr<Terrain> terrain3(new Terrain(0.0f, 1000.0f, 1.0f, 5.0f, 5.0f));
	std::shared_ptr<Terrain> terrain2(new Terrain(0.35f, 1000.0f, 1.0f, 5.0f, 0.35f));
	std::shared_ptr<Terrain> terrain(new Terrain(0.0f, 1000.0f, 1.0f, 5.0f, 0.5f));
	terrain->midPoint(1.0f, 0.3f);
	terrain2->stairs(10, 0.15f);
	
	Model m;
	if (!f.parseObj("resource/bone.obj", m)) { cin.get(); return -1; }
	Model ground, ground2, ground3;
	//if (!f.parseObj("resource/groundFlat.obj", ground)) { cin.get(); return -1; }
	Texture grassTex;
	if (!f.loadTexture("resource/grass.png", grassTex))
		return false;
	terrain3->fillModel(ground3, grassTex);
	terrain2->fillModel(ground2, grassTex);
	terrain->fillModel(ground, grassTex);
	//ground.getMeshes()[0]->getMaterial()->setDifTex(grassTex);

	std::shared_ptr<BasicRenderer> groundRenderer(new BasicRenderer(glm::vec3(0.0, 0.0f, 0.0)));
	if (!groundRenderer->initRenderer(ground, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<BasicRenderer> ground2Renderer(new BasicRenderer(glm::vec3(0.0, 0.0f, 0.0)));
	if (!ground2Renderer->initRenderer(ground2, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<BasicRenderer> ground3Renderer(new BasicRenderer(glm::vec3(0.0, 0.0f, 0.0)));
	if (!ground3Renderer->initRenderer(ground3, s.getProgram("basic_program"))) { cin.get(); return -1; }

	int scenesNum = 6;
	std::shared_ptr<MainScene> *scenes = new std::shared_ptr<MainScene>[scenesNum];
	for (int i = 0; i < scenesNum; i++)
		scenes[i] = std::shared_ptr<MainScene>(new MainScene);

	Model main, lara, venom, astronaut, deadpool, witch, bolter;
	std::shared_ptr<Skeleton> skeleton(new Skeleton(terrain));
	if (!f.loadModelAndSkeletonDae("resource/male_mesh.dae", main, *skeleton.get())) { cin.get(); return -1; }
	std::shared_ptr<SkeletonRenderer> skeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), skeleton));
	if (!skeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<RiggedModelRenderer> riggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), skeleton));
	if (!riggedModelRenderer->initRenderer(main, s.getProgram("animation_program"))) { cin.get(); return -1; }

	scenes[0]->addObject(groundRenderer);
	scenes[0]->addSkeleton(skeleton);
	scenes[0]->addRiggedModelRenderer(riggedModelRenderer);
	scenes[0]->addSkeletonRenderer(skeletonRenderer);
	scenes[0]->addTerrain(terrain, groundRenderer);
	scenes[0]->addTerrain(terrain2, ground2Renderer);
	scenes[0]->addTerrain(terrain3, ground3Renderer);

	skeleton->init();

	if (scenesNum >= 2) {
		std::shared_ptr<Skeleton> venomSkeleton(new Skeleton(terrain));
		if (!f.loadModelAndSkeletonDae("resource/venom.dae", venom, *venomSkeleton.get())) { cin.get(); return -1; }
		std::shared_ptr<SkeletonRenderer> venomSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), venomSkeleton));
		if (!venomSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
		std::shared_ptr<RiggedModelRenderer> venomRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), venomSkeleton));
		if (!venomRiggedModelRenderer->initRenderer(venom, s.getProgram("animation_program"))) { cin.get(); return -1; }

		scenes[1]->addObject(groundRenderer);
		scenes[1]->addSkeleton(venomSkeleton);
		scenes[1]->addRiggedModelRenderer(venomRiggedModelRenderer);
		scenes[1]->addSkeletonRenderer(venomSkeletonRenderer);
		scenes[1]->addTerrain(terrain, groundRenderer);
		scenes[1]->addTerrain(terrain2, ground2Renderer);
		scenes[1]->addTerrain(terrain3, ground3Renderer);

		venomSkeleton->init();
	}
	
	if (scenesNum >= 3) {
		std::shared_ptr<Skeleton> laraSkeleton(new Skeleton(terrain));
		if (!f.loadModelAndSkeletonDae("resource/lara.dae", lara, *laraSkeleton.get())) { cin.get(); return -1; }
		std::shared_ptr<SkeletonRenderer> laraSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), laraSkeleton));
		if (!laraSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
		std::shared_ptr<RiggedModelRenderer> laraRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), laraSkeleton));
		if (!laraRiggedModelRenderer->initRenderer(lara, s.getProgram("animation_program"))) { cin.get(); return -1; }

		scenes[2]->addObject(groundRenderer);
		scenes[2]->addSkeleton(laraSkeleton);
		scenes[2]->addRiggedModelRenderer(laraRiggedModelRenderer);
		scenes[2]->addSkeletonRenderer(laraSkeletonRenderer);
		scenes[2]->addTerrain(terrain, groundRenderer);
		scenes[2]->addTerrain(terrain2, ground2Renderer);
		scenes[2]->addTerrain(terrain3, ground3Renderer);

		laraSkeleton->init();
	}

	if (scenesNum >= 4) {
		std::shared_ptr<Skeleton> deadpoolSkeleton(new Skeleton(terrain));
		if (!f.loadModelAndSkeletonDae("resource/deadpool.dae", deadpool, *deadpoolSkeleton.get())) { cin.get(); return -1; }
		std::shared_ptr<SkeletonRenderer> deadpoolSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), deadpoolSkeleton));
		if (!deadpoolSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
		std::shared_ptr<RiggedModelRenderer> deadpoolRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), deadpoolSkeleton));
		if (!deadpoolRiggedModelRenderer->initRenderer(deadpool, s.getProgram("animation_program"))) { cin.get(); return -1; }

		scenes[3]->addObject(groundRenderer);
		scenes[3]->addSkeleton(deadpoolSkeleton);
		scenes[3]->addRiggedModelRenderer(deadpoolRiggedModelRenderer);
		scenes[3]->addSkeletonRenderer(deadpoolSkeletonRenderer);
		scenes[3]->addTerrain(terrain, groundRenderer);
		scenes[3]->addTerrain(terrain2, ground2Renderer);
		scenes[3]->addTerrain(terrain3, ground3Renderer);

		deadpoolSkeleton->init();
	}

	if (scenesNum >= 5) {
		std::shared_ptr<Skeleton> bolterSkeleton(new Skeleton(terrain));
		if (!f.loadModelAndSkeletonDae("resource/bolter.dae", bolter, *bolterSkeleton.get())) { cin.get(); return -1; }
		std::shared_ptr<SkeletonRenderer> bolterSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), bolterSkeleton));
		if (!bolterSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
		std::shared_ptr<RiggedModelRenderer> bolterRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), bolterSkeleton));
		if (!bolterRiggedModelRenderer->initRenderer(bolter, s.getProgram("animation_program"))) { cin.get(); return -1; }

		scenes[4]->addObject(groundRenderer);
		scenes[4]->addSkeleton(bolterSkeleton);
		scenes[4]->addRiggedModelRenderer(bolterRiggedModelRenderer);
		scenes[4]->addSkeletonRenderer(bolterSkeletonRenderer);
		scenes[4]->addTerrain(terrain, groundRenderer);
		scenes[4]->addTerrain(terrain2, ground2Renderer);
		scenes[4]->addTerrain(terrain3, ground3Renderer);

		bolterSkeleton->init();
	}

	if (scenesNum >= 6) {
		std::shared_ptr<Skeleton> astronautSkeleton(new Skeleton(terrain));
		if (!f.loadModelAndSkeletonDae("resource/astronaut.dae", astronaut, *astronautSkeleton.get())) { cin.get(); return -1; }
		std::shared_ptr<SkeletonRenderer> astronautSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), astronautSkeleton));
		if (!astronautSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
		std::shared_ptr<RiggedModelRenderer> astronautRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), astronautSkeleton));
		if (!astronautRiggedModelRenderer->initRenderer(astronaut, s.getProgram("animation_program"))) { cin.get(); return -1; }

		scenes[5]->addObject(groundRenderer);
		scenes[5]->addSkeleton(astronautSkeleton);
		scenes[5]->addRiggedModelRenderer(astronautRiggedModelRenderer);
		scenes[5]->addSkeletonRenderer(astronautSkeletonRenderer);
		scenes[5]->addTerrain(terrain, groundRenderer);
		scenes[5]->addTerrain(terrain2, ground2Renderer);
		scenes[5]->addTerrain(terrain3, ground3Renderer);

		astronautSkeleton->init();
	}

	if (scenesNum >= 7) {
		std::shared_ptr<Skeleton> witchSkeleton(new Skeleton(terrain));
		if (!f.loadModelAndSkeletonDae("resource/witch.dae", witch, *witchSkeleton.get())) { cin.get(); return -1; }
		std::shared_ptr<SkeletonRenderer> witchSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), witchSkeleton));
		if (!witchSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
		std::shared_ptr<RiggedModelRenderer> witchRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), witchSkeleton));
		if (!witchRiggedModelRenderer->initRenderer(witch, s.getProgram("animation_program"))) { cin.get(); return -1; }

		scenes[6]->addObject(groundRenderer);
		scenes[6]->addSkeleton(witchSkeleton);
		scenes[6]->addRiggedModelRenderer(witchRiggedModelRenderer);
		scenes[6]->addSkeletonRenderer(witchSkeletonRenderer);
		scenes[6]->addTerrain(terrain, groundRenderer);
		scenes[6]->addTerrain(terrain2, ground2Renderer);
		scenes[6]->addTerrain(terrain3, ground3Renderer);

		witchSkeleton->init();
	}

	/*std::shared_ptr<Skeleton> bloodwingSkeleton(new Skeleton);
	if (!f.loadModelAndSkeletonDae("resource/bloodwing.dae", bloodwing, *bloodwingSkeleton.get())) { cin.get(); return -1; }
	std::shared_ptr<SkeletonRenderer>  bloodwingSkeletonRenderer(new SkeletonRenderer(glm::vec3(2.0, 0.0f, 0.0), bloodwingSkeleton));
	if (!bloodwingSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<RiggedModelRenderer>  bloodwingRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(2.0, 0.0f, 0.0), bloodwingSkeleton));
	if (!bloodwingRiggedModelRenderer->initRenderer(bloodwing, s.getProgram("animation_program"))) { cin.get(); return -1; }

	std::shared_ptr<Skeleton> tedSkeleton(new Skeleton);
	if (!f.loadModelAndSkeletonDae("resource/ted.dae", ted, *tedSkeleton.get())) { cin.get(); return -1; }
	std::shared_ptr<SkeletonRenderer>  tedSkeletonRenderer(new SkeletonRenderer(glm::vec3(-2.0, 0.0f, 0.0), tedSkeleton));
	if (!tedSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<RiggedModelRenderer>  tedRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(-2.0, 0.0f, 0.0), tedSkeleton));
	if (!tedRiggedModelRenderer->initRenderer(ted, s.getProgram("animation_program"))) { cin.get(); return -1; }*/
	
	scenes[0]->setName("mainScene");
	//scenes[1]->setName("secondScene");
	//scenes[2]->setName("thirdScene");
	

/*	scene->addUpdate(bloodwingSkeleton);
	scene->addObject(bloodwingRiggedModelRenderer);
	scene->addObject(bloodwingSkeletonRenderer);

	scene->addUpdate(tedSkeleton);
	scene->addObject(tedRiggedModelRenderer);
	scene->addObject(tedSkeletonRenderer);*/
	
	for (int i = 0; i < scenesNum; i++) {
		scenes[i]->initCamera(45.0f, W_WIDTH, W_HEIGHT, 0.1f, 1000.0f, CAM_TRANS_ROT);
		scenes[i]->getCamera()->translateRelative(glm::vec3(0.0f, 0.0f, 2.0f));
		Light light(glm::vec3(1000.0, 1000.0, 1000.0));
		scenes[i]->addLight(light);
		scenes[i]->setAmbientLight(glm::vec3(0.2, 0.2, 0.2));
	}

	/*bloodwingSkeleton->init();
	tedSkeleton->init();*/
	
	for (int i = 0; i < scenesNum; i++)
		app.addScene(scenes[i]);
	app.setActiveScene("mainScene");

	sdl.mainLoop(app);

	sdl.destroy();

	return 0;
}