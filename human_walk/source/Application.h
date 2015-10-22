#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>
#include <memory>
#include <SDL/SDL.h>
#include "Scene.h"

class Application
{
private:
	std::vector<std::shared_ptr<Scene> > scenes;
	std::shared_ptr<Scene> activeScene;
public:
	void addScene(std::shared_ptr<Scene> s);
	bool setActiveScene(const char *name);
	std::shared_ptr<Scene> getScene(const char *name);
	void display();
	void sdlEvent(SDL_Event &event);
};

#endif //APPLICATION_H