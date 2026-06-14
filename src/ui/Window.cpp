#include "ui/Window.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

namespace avb {

namespace {

// GLSL version string matching the GL 3.0 context requested in Application.
constexpr const char* kGlslVersion = "#version 130";

// Extracts the SDL window id an event targets, or 0 for window-agnostic events
// (e.g. SDL_QUIT). Used to route an event to the owning window's ImGui context.
std::uint32_t eventWindowId(const SDL_Event& e) {
    switch (e.type) {
        case SDL_WINDOWEVENT:     return e.window.windowID;
        case SDL_KEYDOWN:
        case SDL_KEYUP:           return e.key.windowID;
        case SDL_TEXTINPUT:       return e.text.windowID;
        case SDL_TEXTEDITING:     return e.edit.windowID;
        case SDL_MOUSEMOTION:     return e.motion.windowID;
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:   return e.button.windowID;
        case SDL_MOUSEWHEEL:      return e.wheel.windowID;
        default:                  return 0;
    }
}

} // namespace

Window::Window(std::string title, int width, int height)
    : title_(std::move(title)), width_(width), height_(height) {}

Window::~Window() {
    if (imguiContext_) {
        SDL_GL_MakeCurrent(sdlWindow_, glContext_);
        ImGui::SetCurrentContext(imguiContext_);
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext(imguiContext_);
        imguiContext_ = nullptr;
    }
    if (glContext_) {
        SDL_GL_DeleteContext(static_cast<SDL_GLContext>(glContext_));
        glContext_ = nullptr;
    }
    if (sdlWindow_) {
        SDL_DestroyWindow(sdlWindow_);
        sdlWindow_ = nullptr;
    }
}

bool Window::initialize() {
    sdlWindow_ = SDL_CreateWindow(
        title_.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width_,
        height_, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    if (!sdlWindow_) {
        SDL_Log("SDL_CreateWindow('%s') failed: %s", title_.c_str(), SDL_GetError());
        return false;
    }
    windowId_ = SDL_GetWindowID(sdlWindow_);

    glContext_ = SDL_GL_CreateContext(sdlWindow_);
    if (!glContext_) {
        SDL_Log("SDL_GL_CreateContext('%s') failed: %s", title_.c_str(), SDL_GetError());
        return false;
    }
    SDL_GL_MakeCurrent(sdlWindow_, glContext_);

    IMGUI_CHECKVERSION();
    imguiContext_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(imguiContext_);

    ImGuiIO& io = ImGui::GetIO();
    // Each window persists its own layout; the string must outlive ImGui.
    iniFilename_ = "imgui_" + title_ + ".ini";
    io.IniFilename = iniFilename_.c_str();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL2_InitForOpenGL(sdlWindow_, glContext_) ||
        !ImGui_ImplOpenGL3_Init(kGlslVersion)) {
        SDL_Log("ImGui backend init failed for '%s'", title_.c_str());
        return false;
    }

    open_ = true;
    return true;
}

void Window::makeContextCurrent() {
    SDL_GL_MakeCurrent(sdlWindow_, glContext_);
    ImGui::SetCurrentContext(imguiContext_);
}

void Window::handleEvent(const SDL_Event& event) {
    const std::uint32_t id = eventWindowId(event);
    if (id != 0 && id != windowId_) {
        return; // belongs to another window
    }

    ImGui::SetCurrentContext(imguiContext_);
    ImGui_ImplSDL2_ProcessEvent(&event);

    if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == windowId_) {
        open_ = false;
        SDL_HideWindow(sdlWindow_);
    }
}

void Window::renderFrame() {
    if (!open_) {
        return;
    }

    SDL_GL_MakeCurrent(sdlWindow_, glContext_);
    ImGui::SetCurrentContext(imguiContext_);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    drawUi();

    // drawUi() may have handed the GL context to another library (the Camera
    // window drives an off-screen OGRE render). Re-assert this window's context
    // before the final clear + ImGui draw, or those commands land in the wrong
    // context and the window is presented blank.
    SDL_GL_MakeCurrent(sdlWindow_, glContext_);
    ImGui::SetCurrentContext(imguiContext_);

    ImGui::Render();

#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif

    // OGRE's off-screen render (driven by the Camera window's drawUi) leaves its
    // render-target framebuffer bound in this GL context. Without resetting it,
    // the window's own clear + ImGui draw land in OGRE's framebuffer instead of
    // the window and it is presented blank. Bind the default framebuffer and
    // clear any leftover scissor so the final draw always targets this window.
    using BindFramebufferFn = void (*)(GLenum, GLuint);
    static auto bindFramebuffer = reinterpret_cast<BindFramebufferFn>(
        SDL_GL_GetProcAddress("glBindFramebuffer"));
    if (bindFramebuffer) {
        bindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    glDisable(GL_SCISSOR_TEST);

    const ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, static_cast<int>(io.DisplaySize.x),
               static_cast<int>(io.DisplaySize.y));
    glClearColor(0.10f, 0.10f, 0.11f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(sdlWindow_);
}

} // namespace avb
