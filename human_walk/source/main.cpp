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
	if (!f.loadFile("resource/basic.vs", strVs)) {
		cin.get();
		return -1;
	}
	if (!f.loadFile("resource/basic.fs", strFs)) {
		cin.get();
		return -1;
	}
	if (!f.loadFile("resource/animation.vs", strAnimationVs)) {
		cin.get();
		return -1;
	}
	GLuint vs, fs, animationVs;
	if (!s.compileShader(strVs.c_str(), GL_VERTEX_SHADER, "basic_vs", vs)) {
		cin.get();
		return -1;
	}
	if (!s.compileShader(strAnimationVs.c_str(), GL_VERTEX_SHADER, "animation_vs", animationVs)) {
		cin.get();
		return -1;
	}
	if (!s.compileShader(strFs.c_str(), GL_FRAGMENT_SHADER, "basic_fs", fs)) {
		cin.get();
		return -1;
	}
	if (!s.linkProgram(vs, fs, "basic_program")) {
		cin.get();
		return -1;
	}
	if (!s.linkProgram(animationVs, fs, "animation_program")) {
		cin.get();
		return -1;
	}

	Model m;
	if (!f.parseObj("resource/bone.obj", m)) {
		cin.get();
		return -1;
	}
	Model m2;
	std::shared_ptr<Skeleton> skeleton(new Skeleton);
	if (!f.loadModelAndSkeletonDae("resource/venom_joker_mat.dae", m2, *skeleton.get())) {
		cin.get();
		return -1;
	}
	std::shared_ptr<SkeletonRenderer> skeletonRenderer(new SkeletonRenderer(glm::vec3(0.0, 0.0f, -2.0), skeleton));
	if (!skeletonRenderer->initRenderer(m, s.getProgram("basic_program"))) {
		cin.get();
		return -1;
	}
	std::shared_ptr<RiggedModelRenderer> riggedModelRenderer(new RiggedModelRenderer(glm::vec3(0.0, 0.0f, -2.0), skeleton));
	if (!riggedModelRenderer->initRenderer(m2, s.getProgram("animation_program"))) {
		cin.get();
		return -1;
	}

	std::shared_ptr<MainScene> scene(new MainScene);
	scene->setName("mainScene");
	scene->addObject(riggedModelRenderer);
	scene->addObject(skeletonRenderer);
	scene->initCamera(45.0f, W_WIDTH, W_HEIGHT, 0.1f, 1000.0f, CAM_TRANS_ROT);
	scene->getCamera()->translate(glm::vec3(0.0f, -0.5f, 2.0f));
	Light light(glm::vec3(10.0, -10.0, -10.0));
	scene->addLight(light);
	scene->setAmbientLight(glm::vec3(0.1, 0.1, 0.1));
	
	app.addScene(scene);
	app.setActiveScene("mainScene");

	sdl.mainLoop(app);

	sdl.destroy();

	return 0;
}