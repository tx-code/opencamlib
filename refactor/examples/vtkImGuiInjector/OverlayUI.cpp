// SPDX-FileCopyrightText: Copyright 2023 Jaswant Panchumarti
// SPDX-License-Identifier: BSD-3-Clause

#include "OverlayUI.h"
#include "vtkDearImGuiInjector.h"

#include <imgui.h>
#include <spdlog/spdlog.h>
#include <vtkErrorCode.h>
#include <vtkInteractorObserver.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include "DialogHelpers.h"
#include "RecentFilesManager.h"
#include "SettingsManager.h"
#include "UIComponents.h"


void OverlayUI::setup(vtkObject* caller, unsigned long, void*, void* callData)
{
    spdlog::info("OverlayUI::setup - Starting setup");
    auto* overlay_ = reinterpret_cast<vtkDearImGuiInjector*>(caller);
    if (!callData) {
        spdlog::error("OverlayUI::setup - No callback data, setup failed");
        return;
    }
    if (bool imguiInitStatus = *(static_cast<bool*>(callData))) {
        spdlog::info("OverlayUI::setup - ImGui initialization successful");

        // 设置字体和样式
        auto& io = ImGui::GetIO();
        io.Fonts->AddFontDefault();

        auto& style = ImGui::GetStyle();
        style.ChildRounding = 8;
        style.FrameRounding = 8;
        style.GrabRounding = 8;
        style.PopupRounding = 8;
        style.ScrollbarRounding = 8;
        style.TabRounding = 8;
        style.WindowRounding = 8;
        style.FrameBorderSize = 1.f;

        // 从JSON文件加载最近文件列表和设置
        ocl::RecentFilesManager::LoadRecentFiles();
        ocl::SettingsManager::LoadSettings();
        spdlog::info("OverlayUI::setup - Setup completed");
    }
    else {
        spdlog::error("OverlayUI::setup - ImGui initialization failed");
        vtkErrorWithObjectMacro(overlay_,
                                "Failed to setup overlay UI because ImGUI failed to initialize!");
    }
}

void OverlayUI::draw(vtkObject* caller, unsigned long, void*, void* callData)
{
    auto* overlay_ = reinterpret_cast<vtkDearImGuiInjector*>(caller);

    ImGui::SetNextWindowBgAlpha(0.5);
    ImGui::SetNextWindowPos(ImVec2(5, 25), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(450, 550), ImGuiCond_Once);
    ImGui::Begin("VTK");
    if (ImGui::CollapsingHeader("vtkRenderWindow")) {
        auto rw = overlay_->Interactor->GetRenderWindow();
        ImGui::Text("MTime: %ld", rw->GetMTime());
        ImGui::Text("Name: %s", rw->GetClassName());
        if (ImGui::TreeNode("Capabilities")) {
            ImGui::TextWrapped("OpenGL: %s", rw->ReportCapabilities());
            ImGui::TreePop();
        }
    }
    if (ImGui::CollapsingHeader("vtkRenderWindowInteractor")) {
        auto& iren = overlay_->Interactor;
        ImGui::Text("MTime: %ld", iren->GetMTime());
        ImGui::Text("Name: %s", iren->GetClassName());
        if (ImGui::TreeNode("Style")) {
            auto styleBase = iren->GetInteractorStyle();
            vtkInteractorObserver* iStyle = nullptr;
            if (styleBase->IsA("vtkInteractorStyleSwitchBase")) {
                iStyle = vtkInteractorStyleSwitch::SafeDownCast(styleBase)->GetCurrentStyle();
            }
            else {
                iStyle = styleBase;
            }
            ImGui::Text("MTime: %ld", iStyle->GetMTime());
            ImGui::Text("Name: %s", iStyle->GetClassName());
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Mouse")) {
            int* xy = iren->GetEventPosition();
            ImGui::Text("X: %d", xy[0]);
            ImGui::Text("Y: %d", xy[1]);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Keyboard")) {
            ImGui::Text("KeySym: %s", iren->GetKeySym());
            ImGui::SameLine();
            ocl::DialogHelpers::HelpMarker("VTK does not flush KeySym per frame.");
            ImGui::Text("KeyCode: %c", iren->GetKeyCode());
            ImGui::Text("Mods: %s %s %s",
                        (iren->GetAltKey() ? "ALT" : " "),
                        (iren->GetControlKey() ? "CTRL" : " "),
                        (iren->GetShiftKey() ? "SHIFT" : " "));
            ImGui::TreePop();
        }
    }

    if (ImGui::CollapsingHeader("OCL Example", ImGuiTreeNodeFlags_DefaultOpen)) {
        ocl::UIComponents::DrawCAMExample(overlay_);
    }

    ImGui::End();
}
