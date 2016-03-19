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

#define W_WIDTH 800
#define W_HEIGHT 600

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

	Model m;
	if (!f.parseObj("resource/bone.obj", m)) { cin.get(); return -1; }
	Model ground;
	if (!f.parseObj("resource/groundFlat.obj", ground)) { cin.get(); return -1; }
	Texture grassTex;
	if (!f.loadTexture("resource/grass.png", grassTex))
		return false;
	ground.getMeshes()[0]->getMaterial()->setDifTex(grassTex);

	Model main, lara, venom, bloodwing, ted;
	std::shared_ptr<Skeleton> skeleton(new Skeleton);
	if (!f.loadModelAndSkeletonDae("resource/riggedFixed2.dae", main, *skeleton.get())) { cin.get(); return -1; }
	std::shared_ptr<SkeletonRenderer> skeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), skeleton));
	if (!skeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<RiggedModelRenderer> riggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), skeleton));
	if (!riggedModelRenderer->initRenderer(main, s.getProgram("animation_program"))) { cin.get(); return -1; }

	std::shared_ptr<Skeleton> venomSkeleton(new Skeleton);
	if (!f.loadModelAndSkeletonDae("resource/venom_ref.dae", venom, *venomSkeleton.get())) { cin.get(); return -1; }
	std::shared_ptr<SkeletonRenderer> venomSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), venomSkeleton));
	if (!venomSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<RiggedModelRenderer> venomRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), venomSkeleton));
	if (!venomRiggedModelRenderer->initRenderer(venom, s.getProgram("animation_program"))) { cin.get(); return -1; }
	
	std::shared_ptr<Skeleton> laraSkeleton(new Skeleton);
	if (!f.loadModelAndSkeletonDae("resource/lara.dae", lara, *laraSkeleton.get())) { cin.get(); return -1; }
	std::shared_ptr<SkeletonRenderer> laraSkeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, 0.0), laraSkeleton));
	if (!laraSkeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) { cin.get(); return -1; }
	std::shared_ptr<RiggedModelRenderer> laraRiggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, 0.0), laraSkeleton));
	if (!laraRiggedModelRenderer->initRenderer(lara, s.getProgram("animation_program"))) { cin.get(); return -1; }

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
	
	std::shared_ptr<BasicRenderer> groundRenderer(new BasicRenderer(glm::vec3(0.0, 0.0f, 0.0)));
	if (!groundRenderer->initRenderer(ground, s.getProgram("basic_program"))) { cin.get(); return -1; }

	std::shared_ptr<MainScene> scenes[3];
	for (int i = 0; i < 3; i++)
		scenes[i] = std::shared_ptr<MainScene>(new MainScene);

	scenes[0]->setName("mainScene");
	scenes[1]->setName("secondScene");
	scenes[2]->setName("thirdScene");

	scenes[0]->addObject(groundRenderer);
	scenes[0]->addSkeleton(skeleton);
	scenes[0]->addRiggedModelRenderer(riggedModelRenderer);
	scenes[0]->addSkeletonRenderer(skeletonRenderer);
	
	scenes[1]->addObject(groundRenderer);
	scenes[1]->addSkeleton(laraSkeleton);
	scenes[1]->addRiggedModelRenderer(laraRiggedModelRenderer);
	scenes[1]->addSkeletonRenderer(laraSkeletonRenderer);

	scenes[2]->addObject(groundRenderer);
	scenes[2]->addSkeleton(venomSkeleton);
	scenes[2]->addRiggedModelRenderer(venomRiggedModelRenderer);
	scenes[2]->addSkeletonRenderer(venomSkeletonRenderer);

/*	scene->addUpdate(bloodwingSkeleton);
	scene->addObject(bloodwingRiggedModelRenderer);
	scene->addObject(bloodwingSkeletonRenderer);

	scene->addUpdate(tedSkeleton);
	scene->addObject(tedRiggedModelRenderer);
	scene->addObject(tedSkeletonRenderer);*/
	
	for (int i = 0; i < 3; i++) {
		scenes[i]->initCamera(45.0f, W_WIDTH, W_HEIGHT, 0.1f, 1000.0f, CAM_TRANS_ROT);
		scenes[i]->getCamera()->translateRelative(glm::vec3(0.0f, 0.0f, 2.0f));
		Light light(glm::vec3(1000.0, 1000.0, 1000.0));
		scenes[i]->addLight(light);
		scenes[i]->setAmbientLight(glm::vec3(0.2, 0.2, 0.2));
	}

	skeleton->init();
	laraSkeleton->init();
	venomSkeleton->init();
	/*bloodwingSkeleton->init();
	tedSkeleton->init();*/
	
	for (int i = 0; i < 3; i++)
		app.addScene(scenes[i]);
	app.setActiveScene("mainScene");

	sdl.mainLoop(app);

	sdl.destroy();

	return 0;
}