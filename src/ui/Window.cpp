#include "ui/Window.h"

// NOTE: Implementation sketch for the SDL2 + OpenGL3 + Dear ImGui backend.
// The actual SDL/GL/ImGui calls are included behind the real headers at build
// time; the structure below documents the intended lifecycle.
//
// #include <SDL.h>
// #include <imgui.h>
// #include <imgui_impl_sdl2.h>
// #include <imgui_impl_opengl3.h>

namespace avb {

Window::Window(std::string title, int width, int height)
    : title_(std::move(title)), width_(width), height_(height) {}

Window::~Window() {
    // TODO: ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown();
    //       ImGui::DestroyContext(imguiContext_);
    //       SDL_GL_DeleteContext(glContext_); SDL_DestroyWindow(sdlWindow_);
}

bool Window::initialize() {
    // TODO:
    //   sdlWindow_  = SDL_CreateWindow(title_.c_str(), centered, w, h, GL|RESIZABLE);
    //   glContext_  = SDL_GL_CreateContext(sdlWindow_);
    //   imguiContext_ = ImGui::CreateContext();
    //   ImGui_ImplSDL2_InitForOpenGL(sdlWindow_, glContext_);
    //   ImGui_ImplOpenGL3_Init("#version 130");
    open_ = true;
    return true;
}

void Window::handleEvent(const SDL_Event& /*event*/) {
    // TODO: ImGui::SetCurrentContext(imguiContext_);
    //       ImGui_ImplSDL2_ProcessEvent(&event);
    //       if (event is SDL_WINDOWEVENT_CLOSE for this window) open_ = false;
}

void Window::renderFrame() {
    if (!open_) {
        return;
    }
    // TODO: SDL_GL_MakeCurrent(sdlWindow_, glContext_);
    //       ImGui::SetCurrentContext(imguiContext_);
    //       ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL2_NewFrame(); ImGui::NewFrame();
    drawUi();
    // TODO: ImGui::Render();
    //       glClear(...); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    //       SDL_GL_SwapWindow(sdlWindow_);
}

} // namespace avb
