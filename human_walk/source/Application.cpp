#include "Application.h"

void Application::addScene(std::shared_ptr<Scene> s)
{
	scenes.push_back(s);
}

bool Application::setActiveScene(const char *name)
{
	for (unsigned int i = 0; i < scenes.size(); i++)	{
		if (strcmp(scenes[i]->getName(), name) == 0) {
			activeScene = scenes[i];
			return true;
		}
	}

	return false;
}

std::shared_ptr<Scene> Application::getScene(const char *name)
{
	for (unsigned int i = 0; i < scenes.size(); i++) {
		if (strcmp(scenes[i]->getName(), name) == 0) {
			return scenes[i];
		}
	}

	return nullptr;
}

void Application::display()
{
	activeScene->render();
}

void Application::update(float dt)
{
	activeScene->update(dt);
}

void Application::sdlEvent(SDL_Event &event)
{
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_TAB)
		setActiveNextScene();
	else
		activeScene->handleSdlEvent(event);
}

void Application::setActiveNextScene()
{
	int i = 0;
	for (; i < scenes.size(); i++) {
		if (scenes[i] == activeScene)
			break;
	}
	i++;
	i = i >= scenes.size() ? 0 : i;

	activeScene = scenes[i];
}

void Application::onWindowResize(int width, int height)
{
	for (auto &s : scenes)
		s->getCamera()->onWindowResize(width, height);
}
