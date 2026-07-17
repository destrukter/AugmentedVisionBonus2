#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <imgui.h>

#include "storage/Assets.h"
#include "storage/DataStore.h"
#include "storage/Types.h"

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

/// Display labels for a list of assignments: the model's name, suffixed with
/// an instance number (" (1)", " (2)", ...) when the same model is assigned
/// to the image more than once. Instances are numbered in assignment-id
/// (creation) order, matching the order of `assignments` as returned by
/// DataStore::assignmentsForImage. Used by the Upload window's assignment
/// list and the Configure window's model dropdown so both show the same
/// labels for the same instances.
inline std::unordered_map<Id, std::string> assignmentDisplayLabels(
    const DataStore& store, const std::vector<Id>& assignments) {
    std::unordered_map<Id, int> totals;
    for (const Id aid : assignments) {
        if (const Assignment* a = store.assignment(aid)) {
            ++totals[a->modelId];
        }
    }
    std::unordered_map<Id, int> seen;
    std::unordered_map<Id, std::string> labels;
    for (const Id aid : assignments) {
        const Assignment* a = store.assignment(aid);
        if (!a) {
            continue;
        }
        const ModelAsset* model = store.model(a->modelId);
        std::string label = model ? model->name : "<missing>";
        if (totals[a->modelId] > 1) {
            label += " (" + std::to_string(++seen[a->modelId]) + ")";
        }
        labels.emplace(aid, std::move(label));
    }
    return labels;
}

} // namespace avb
