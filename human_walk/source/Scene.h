#ifndef SCENE_H
#define SCENE_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <memory>
#include <SDL/SDL.h>
#include "Camera.h"
#include "Renderer.h"
#include "Update.h"
#include "Light.h"

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
};

class MainScene : public Scene
{
private:
	int cameraMode;
public:
	void handleSdlEvent(SDL_Event &event);
	void onKeyDown(SDL_Keycode key);
	void onMouseMove(Sint32 x, Sint32 y, Sint32 xrel, Sint32 yrel, Uint32 state);
	void onMouseWheel(Sint32 x, Sint32 y);
	void setCameraMode(int mode);
	void initCamera(float fov, int width, int height, float nearPlane, float farPlane, int mode = CAM_ROT_TRANS);
	void render();
	void update(float dt);
};

#endif //SCENE_H