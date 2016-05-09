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

	//request 4.5 OpenGL context and core funcionality
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

	//create window
	mainwindow = SDL_CreateWindow("human_walk", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
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

	ImGui_ImplSdlGL3_Init(mainwindow);

	return true;
}

void SDLHandler::destroy()
{
	ImGui_ImplSdlGL3_Shutdown();
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
		bool quit = false;
		while (SDL_PollEvent(&event))
		{
			ImGui_ImplSdlGL3_ProcessEvent(&event);

			if (event.type == SDL_QUIT || (event.key.keysym.sym == SDLK_ESCAPE && event.type == SDL_KEYDOWN)) {
				quit = true;
				break;
			}
			if (event.type == SDL_WINDOWEVENT) {
				if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
					app.onWindowResize((GLsizei)event.window.data1, (GLsizei)event.window.data2);
					glViewport(0, 0, (GLsizei)event.window.data1, (GLsizei)event.window.data2);
				}
			}
			app.sdlEvent(event);
		}

		if (quit)
			break;

		Uint32 tics = SDL_GetTicks();
		if (lastTics == 0)
			lastTics = tics;
		Uint32 dt = tics - lastTics;
		lastTics = tics;

		ImGui_ImplSdlGL3_NewFrame();

		app.handleGui();
		if (ImGui::Button("Next Model"))
			app.setActiveNextScene();
		ImGui::SameLine(150);
		ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Text("Rotate camera using RMB and zoom using mouse wheel.");
		ImGui::End();

		app.update(dt * 0.001f);
		app.display();

		ImGui::Render();

		SDL_GL_SwapWindow(mainwindow);
	}

} 