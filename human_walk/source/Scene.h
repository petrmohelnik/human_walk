#ifndef SCENE_H
#define SCENE_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include <SDL/SDL.h>
#include "imgui.h"
#include "Camera.h"
#include "Update.h"
#include "Light.h"
#include "BasicRenderer.h"
#include "SkeletonRenderer.h"
#include "RiggedModelRenderer.h"

class Scene
{
protected:
	std::string name;
	std::vector<std::shared_ptr<Renderer> > objects;
	std::vector<std::shared_ptr<Update> > updates;
	Camera camera;
	std::vector<Light> lights;
	glm::vec3 ambientLight;
public:
	void setName(const char *n);
	void addObject(std::shared_ptr<Renderer> o);
	void addUpdate(std::shared_ptr<Update> u);
	void addLight(Light l);
	void setAmbientLight(glm::vec3 a);
	virtual void initCamera(float fov, int width, int height, float nearPlane, float farPlane, int mode = CAM_ROT_TRANS);
	Camera* getCamera();
	virtual void render();
    virtual void update(float dt);
	const char* getName();
	virtual void handleSdlEvent(SDL_Event &event);
	virtual void handleGui();
};

class MainScene : public Scene
{
private:
	int cameraMode;
	std::shared_ptr<Skeleton> skeleton;
	std::shared_ptr<SkeletonRenderer> skeletonRenderer;
	std::shared_ptr<RiggedModelRenderer> riggedModelRenderer;
	std::vector<std::shared_ptr<Terrain>> terrain;
	std::vector<std::shared_ptr<BasicRenderer>> terrainRenderer;
	bool displaySkeleton = true;
	bool displayRiggedModel = true;
	glm::vec3 prevCamPos;
	bool pause = false;
	
	float prevWalkingSpeed = 0.0f;
	float walkingSpeed = 1.0f;
	float speedCoeff = 0.7f;
	float pelvicTilt = 1.0f;
	float pelvicTiltForward = 1.0f;
	float pelvicRotation = 1.0f;
	float pelvisLateralDisp = 1.0f;
	float pelvisVerticalDisp = 1.0f;
	float shoulderSwing = 1.0f;
	float elbowSwing = 1.0f;
	unsigned int activeTerrain = 0;
	float toeOutAngle = 0.1f;
	float stepLength = 0.7f;
	float pelvisMidStanceDisp = 0.0f;
	float rotationForward = 0.0f;
	float stepWidth = 0.1f;
	float armWidth = 0.0f;
	float maxPelvisHeight = 0.0f;
	float maxElbowExtension = 0.5f;
	float footUpCoeff = 0.0f;

	glm::vec3 jointWeights;
	std::vector<float> heelSwingCurve;
	std::vector<float> pelvisVerticalCurve;
	std::vector<float> pelvisSpeed1Curve;
	std::vector<float> pelvisSpeed2Curve;
public:
	MainScene() : heelSwingCurve(50, 0.0f), pelvisVerticalCurve(25, 0.0f), pelvisSpeed1Curve(50, 0.0f), pelvisSpeed2Curve(25, 0.0f),
		jointWeights(1.0) {}
	void handleSdlEvent(SDL_Event &event);
	void onKeyDown(SDL_Keycode key);
	void onMouseMove(Sint32 x, Sint32 y, Sint32 xrel, Sint32 yrel, Uint32 state);
	void onMouseWheel(Sint32 x, Sint32 y);
	void setCameraMode(int mode);
	void initCamera(float fov, int width, int height, float nearPlane, float farPlane, int mode = CAM_ROT_TRANS);
	void render();
	void update(float dt);
	void addSkeleton(std::shared_ptr<Skeleton> s) { updates.push_back(s); skeleton = s; }
	void addSkeletonRenderer(std::shared_ptr<SkeletonRenderer> s) { objects.push_back(s); skeletonRenderer = s; }
	void addRiggedModelRenderer(std::shared_ptr<RiggedModelRenderer> s) { objects.push_back(s); riggedModelRenderer = s; }
	void handleGui();
	void addTerrain(std::shared_ptr<Terrain> t, std::shared_ptr<BasicRenderer> r) { terrain.push_back(t); terrainRenderer.push_back(r); }
};

#endif //SCENE_H