#ifndef SDL_HANDLER_H
#define SDL_HANDLER_H

#include <SDL/SDL.h>
#include <iostream>
#include "imgui.h"
#include "imgui_impl_sdl_gl3.h"
#include "Application.h"

class SDLHandler
{
private:
	SDL_Window *mainwindow; //window handle
	SDL_GLContext maincontext; //context handle
	int width, height;
	Uint32 lastTics = 0;
public:
	SDLHandler(int w, int h);
	bool init();
	void destroy();
	void mainLoop(Application &app);
	void swap() { SDL_GL_SwapWindow(mainwindow); }
};

#endif //SDL_HANDLER_H