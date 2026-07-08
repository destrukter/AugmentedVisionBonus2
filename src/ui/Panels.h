#pragma once

#include <imgui.h>

namespace avb {

/// Begins the single ImGui panel that fills a whole OS window (each of the
/// three application windows hosts exactly one top-level panel). Callers must
/// still call ImGui::End().
///
/// `allowScroll` should stay false for panels that size content (viewports,
/// letterboxed images) from GetContentRegionAvail(): inside a scrolling window
/// that value grows as the window scrolls down, so avail-sized content feeds
/// back into the scroll range and "runs away" - controls below it get pushed
/// out of view forever. Panels with fixed-size content that may overflow a
/// small window (e.g. the Upload lists) pass true to get normal scrolling.
inline void beginFullWindow(const char* name, bool allowScroll = false) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;
    if (!allowScroll) {
        flags |= ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    }
    ImGui::Begin(name, nullptr, flags);
}

} // namespace avb
