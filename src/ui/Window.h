#pragma once

#include <string>

struct SDL_Window;
union SDL_Event;
struct ImGuiContext;

namespace avb {

/// Base class for a single top-level OS window backed by its own SDL window,
/// OpenGL context and Dear ImGui context.
///
/// Each of the three application windows (Upload / Configure / Camera) derives
/// from this and implements drawUi() to build its widgets every frame. Because
/// every window owns a distinct ImGuiContext, they can be driven independently
/// while remaining open simultaneously.
class Window {
public:
    Window(std::string title, int width, int height);
    virtual ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    /// Creates the SDL window, GL context and ImGui context. Returns false on
    /// failure.
    bool initialize();

    /// Forwards an SDL event to this window's ImGui context if it targets it.
    void handleEvent(const SDL_Event& event);

    /// Renders one frame: new ImGui frame -> drawUi() -> present.
    void renderFrame();

    bool isOpen() const { return open_; }
    void requestClose() { open_ = false; }

    SDL_Window* sdlWindow() const { return sdlWindow_; }

protected:
    /// Subclasses build their ImGui UI here. Called once per frame between the
    /// ImGui new-frame and render calls.
    virtual void drawUi() = 0;

    std::string title_;
    int width_;
    int height_;

private:
    SDL_Window* sdlWindow_{nullptr};
    void* glContext_{nullptr};      // SDL_GLContext (opaque here)
    ImGuiContext* imguiContext_{nullptr};
    bool open_{false};
};

} // namespace avb
