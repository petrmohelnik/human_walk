#include "SDLHandler.h"

SDLHandler::SDLHandler(int w, int h)
{
	width = w;
	height = h;
}

bool SDLHandler::init()
{
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
		std::cout << "ERROR: Cannot initialize SDL\nSDL ERROR: " << SDL_GetError() << "\n";
		return false;
	}

	//request 4.5 OpenGL context nad core funcionality
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);

	//create window
	mainwindow = SDL_CreateWindow("human_walk", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
	if (!mainwindow) {
		std::cout << "ERROR: Cannot create window\nSDL ERROR: " << SDL_GetError() << "\n";
		return false;
	}

	//create opengl context and attach it to window
	maincontext = SDL_GL_CreateContext(mainwindow);
	if (!maincontext) {
		std::cout << "ERROR: Cannot create opengl context\nSDL ERROR: " << SDL_GetError() << "\n";
		return false;
	}

	//vertical synchronization
	SDL_GL_SetSwapInterval(1);

	return true;
}

void SDLHandler::destroy()
{
	SDL_GL_DeleteContext(maincontext);
	SDL_DestroyWindow(mainwindow);
	SDL_Quit();
}

void SDLHandler::mainLoop(Application &app)
{
	// Window is not minimized
	//bool active = true;
	while (1)
	{
		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			app.sdlEvent(event);
		}
		//	if (event.key.keysym.sym == SDLK_ESCAPE) 
		//		break; //konec kdyz ESC

		app.display();
		SDL_GL_SwapWindow(mainwindow);
	}

}