// SPDX-FileCopyrightText: Copyright 2023 Jaswant Panchumarti
// SPDX-License-Identifier: BSD-3-Clause

#include "OverlayUI.h"
#include "vtkDearImGuiInjector.h"

#include <boost/math/constants/constants.hpp>
#include <codecvt>
#include <imgui.h>  // to draw custom UI
#include <nfd.h>
#include <spdlog/spdlog.h>
#include <vtkInteractorObserver.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSetGet.h>

#include "oclUtils.h"

namespace
{
//------------------------------------------------------------------------------
std::wstring to_wstring(const char* str)
{
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    return converter.from_bytes(str);
}

//------------------------------------------------------------------------------
void HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void DrawCAMExample(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;
    if (ImGui::BeginMenu("OCL Operations")) {
        if (ImGui::Button("Load STL")) {
            // First, pop up a dialog to ask the user to select the file

            NFD_Init();

            nfdu8char_t* outPath = nullptr;
            nfdu8filteritem_t filters[1] = {{"STL Models", "stl"}};
            nfdopendialogu8args_t args = {0};
            args.filterList = filters;
            args.filterCount = 1;
            auto result = NFD_OpenDialogU8_With(&outPath, &args);

            if (result == NFD_OKAY) {
                modelManager.surface = std::make_unique<ocl::STLSurf>();
                ocl::STLReader reader(to_wstring(outPath), *modelManager.surface);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
                NFD_FreePathU8(outPath);
            }
            else if (result == NFD_CANCEL) {
                modelManager.surface.reset();
                // User canceled the dialog
                spdlog::info("User canceled the dialog");
            }
            else {
                modelManager.surface.reset();
                // Handle other error codes
                spdlog::error("Error: {}", NFD_GetError());
            }

            NFD_Quit();
        }

        if (ImGui::Button("Add Cutter")) {
            ImGui::OpenPopup("###Add Cutter");
        }
        /// The popup menu for adding a cutter
        if (ImGui::BeginPopup("###Add Cutter", ImGuiWindowFlags_AlwaysAutoResize)) {
            static const char* cutter_types[] = {"CylCutter",
                                                 "BallCutter",
                                                 "BullCutter",
                                                 "ConeCutter"};
            static int cutter_type_index = 0;
            ImGui::Combo("Cutter Types",
                         &cutter_type_index,
                         cutter_types,
                         IM_ARRAYSIZE(cutter_types));

            using namespace boost::math::constants;
            static double diameter = 2.0;
            static double length = 10.0;
            static double angle_in_deg =
                radian<double>() * third_pi<double>();  // cone cutter need this
            static double radius = 0.1;                 // bull cutter need this
            if (cutter_type_index == 0) {
                ImGui::InputDouble("Diameter", &diameter, 0.01f, 1.0f, "%.3f");
                ImGui::InputDouble("Length", &length, 0.01f, 1.0f, "%.3f");
            }
            else if (cutter_type_index == 1) {
                ImGui::InputDouble("Diameter", &diameter, 0.01f, 1.0f, "%.3f");
                ImGui::InputDouble("Length", &length, 0.01f, 1.0f, "%.3f");
            }
            else if (cutter_type_index == 2) {
                ImGui::InputDouble("Diameter", &diameter, 0.01f, 1.0f, "%.3f");
                ImGui::InputDouble("Length", &length, 0.01f, 1.0f, "%.3f");
                ImGui::InputDouble("Radius", &radius, 0.01f, 1.0f, "%.3f");
            }
            else if (cutter_type_index == 3) {
                ImGui::InputDouble("Diameter", &diameter, 0.01f, 1.0f, "%.3f");
                ImGui::InputDouble("Length", &length, 0.01f, 1.0f, "%.3f");
                ImGui::InputDouble("Angle", &angle_in_deg, 0.01f, 1.0f, "%.3f");
            }

            if (ImGui::Button("Ok")) {
                // According the cutter_type_index, change the cutter type
                if (cutter_type_index == 0) {
                    modelManager.cutter = std::make_unique<ocl::CylCutter>(diameter, length);
                    spdlog::info("CylCutter created: {}", modelManager.cutter->str());
                }
                else if (cutter_type_index == 1) {
                    modelManager.cutter = std::make_unique<ocl::BallCutter>(diameter, length);
                    spdlog::info("BallCutter created: {}", modelManager.cutter->str());
                }
                else if (cutter_type_index == 2) {
                    modelManager.cutter =
                        std::make_unique<ocl::BullCutter>(diameter, radius, length);
                    spdlog::info("BullCutter created: {}", modelManager.cutter->str());
                }
                else if (cutter_type_index == 3) {
                    modelManager.cutter =
                        std::make_unique<ocl::ConeCutter>(diameter,
                                                          degree<double>() * angle_in_deg,
                                                          length);
                    spdlog::info("ConeCutter created: {}", modelManager.cutter->str());
                }
                UpdateCutterActor(actorManager.cutterActor,
                                  *modelManager.cutter,
                                  ocl::Point(0, 0, 0));
                injector->ForceResetCamera();
            }

            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (ImGui::Button("Add Operation")) {
            ImGui::OpenPopup("###Add Operation");
        }
        /// The popup menu for adding a operation
        if (ImGui::BeginPopup("###Add Operation", ImGuiWindowFlags_AlwaysAutoResize)) {
            static const char* op_types[] = {"WaterLine",
                                             "AdaptiveWaterLine",
                                             "PathDropCutter",
                                             "AdaptivePathDropCutter"};
            static int op_type_index = 0;
            ImGui::Combo("Operation Types", &op_type_index, op_types, IM_ARRAYSIZE(op_types));

            static double sampling = 0.1;
            static double min_sampling = 0.01;  // adaptive waterline need this
            static double lift_step = 0.1;
            static double lift_from = 0.0;
            static double lift_to = 1;

            switch (op_type_index) {
                case 0:
                    ImGui::InputDouble("Sampling", &sampling, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Lift Step", &lift_step, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Lift From", &lift_from, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Lift To", &lift_to, 0.01f, 1.0f, "%.3f");
                    break;
                case 1:
                    ImGui::InputDouble("Sampling", &sampling, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Min Sampling", &min_sampling, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Lift Step", &lift_step, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Lift From", &lift_from, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Lift To", &lift_to, 0.01f, 1.0f, "%.3f");
                    break;
                case 2:
                    ImGui::InputDouble("Sampling", &sampling, 0.01f, 1.0f, "%.3f");
                    break;
                case 3:
                    ImGui::InputDouble("Sampling", &sampling, 0.01f, 1.0f, "%.3f");
                    ImGui::InputDouble("Min Sampling", &min_sampling, 0.01f, 1.0f, "%.3f");
                    break;
                default:
                    break;
            }

            if (ImGui::Button("Run Operation")) {
                if (modelManager.cutter && modelManager.surface) {
                    switch (op_type_index) {
                        case 0:
                            waterline(modelManager,
                                      actorManager,
                                      sampling,
                                      lift_to,
                                      lift_step,
                                      lift_from);
                            break;
                        case 1:
                            adaptiveWaterline(modelManager,
                                              actorManager,
                                              sampling,
                                              min_sampling,
                                              lift_to,
                                              lift_step,
                                              lift_from);
                            break;
                        case 2:
                            pathDropCutter(modelManager, actorManager, sampling);
                            break;
                        case 3:
                            adaptivePathDropCutter(modelManager,
                                                   actorManager,
                                                   sampling,
                                                   min_sampling);
                            break;
                    }
                    injector->ForceResetCamera();
                }
                else {
                    spdlog::error("No Cutter or Surface");
                    ImGui::OpenPopup("No Cutter or Surface");
                    // Always center this window when appearing
                    auto center = ImGui::GetMainViewport()->GetCenter();
                    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
                }
            }

            if (ImGui::BeginPopupModal("No Cutter or Surface",
                                       nullptr,
                                       ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::Text("Please select a cutter and a surface");
                ImGui::SetItemDefaultFocus();
                if (ImGui::Button("OK", ImVec2(120, 0))) {
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }

            ImGui::EndPopup();
        }

        ImGui::EndMenu();
    }


    ImGui::SeparatorText("Data Model");
    bool axesVisible = actorManager.axesActor->GetVisibility();
    if (ImGui::Checkbox("Show Axes", &axesVisible)) {
        actorManager.axesActor->SetVisibility(axesVisible);
    }

    if (modelManager.surface) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    if (ImGui::TreeNode("WorkPieces")) {
        if (modelManager.surface) {
            auto model = actorManager.modelActor;
            ImGui::Text(model->GetObjectName().c_str());

            // Bbox
            auto bbox = model->GetBounds();
            ImGui::Text("Bbox: Min(%.2f, %.2f, %.2f), Max(%.2f, %.2f, %.2f)",
                        bbox[0],
                        bbox[2],
                        bbox[4],
                        bbox[1],
                        bbox[3],
                        bbox[5]);

            bool visible = model->GetVisibility();
            ImGui::Checkbox("Show WorkPiece", &visible);
            model->SetVisibility(visible);

            int representation = model->GetProperty()->GetRepresentation();
            std::array<bool, 3> touched;
            touched[0] = ImGui::RadioButton("Points", &representation, 0);
            ImGui::SameLine();
            touched[1] = ImGui::RadioButton("Wireframe", &representation, 1);
            ImGui::SameLine();
            touched[2] = ImGui::RadioButton("Surface", &representation, 2);
            if (std::find(touched.begin(), touched.end(), true) != touched.end()) {
                model->GetProperty()->SetRepresentation(representation);
            }

            // Transparency
            double trans = 1 - model->GetProperty()->GetOpacity();
            static double _min = 0;
            static double _max = 1;
            if (ImGui::SliderScalar("Transparency", ImGuiDataType_Double, &trans, &_min, &_max)) {
                model->GetProperty()->SetOpacity(1 - trans);
            }
        }
        else {
            ImGui::TextDisabled("No WorkPiece");
        }

        ImGui::TreePop();
    }


    if (modelManager.cutter) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    if (ImGui::TreeNode("Cutters")) {
        if (modelManager.cutter) {
            auto cutter = actorManager.cutterActor;
            ImGui::Text(cutter->GetObjectName().c_str());
            bool visible = cutter->GetVisibility();
            ImGui::Checkbox("Show Cutter", &visible);
            cutter->SetVisibility(visible);

            int representation = cutter->GetProperty()->GetRepresentation();
            std::array<bool, 2> touched;
            touched[0] = ImGui::RadioButton("Wireframe", &representation, 1);
            ImGui::SameLine();
            touched[1] = ImGui::RadioButton("Surface", &representation, 2);
            if (std::find(touched.begin(), touched.end(), true) != touched.end()) {
                cutter->GetProperty()->SetRepresentation(representation);
            }
        }
        else {
            ImGui::TextDisabled("No Cutter");
        }
        ImGui::TreePop();
    }

    if (modelManager.operation) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    if (ImGui::TreeNode("Operations")) {
        if (modelManager.operation) {
            auto operation = actorManager.operationActor;
            ImGui::Text(operation->GetObjectName().c_str());
            bool visible = operation->GetVisibility();
            ImGui::Checkbox("Show Operation", &visible);
            operation->SetVisibility(visible);
        }
        else {
            ImGui::TextDisabled("No Operation");
        }
        ImGui::TreePop();
    }
}
}  // namespace

//------------------------------------------------------------------------------
void OverlayUI::setup(vtkObject* caller, unsigned long, void*, void* callData)
{
    auto* overlay_ = reinterpret_cast<vtkDearImGuiInjector*>(caller);
    if (!callData) {
        return;
    }
    if (bool imguiInitStatus = *(static_cast<bool*>(callData))) {
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
    }
    else {
        vtkErrorWithObjectMacro(overlay_,
                                "Failed to setup overlay UI because ImGUI failed to initialize!");
    }
}

//------------------------------------------------------------------------------
void OverlayUI::draw(vtkObject* caller, unsigned long, void*, void* callData)
{
    auto* overlay_ = reinterpret_cast<vtkDearImGuiInjector*>(caller);
    auto& modelManager = overlay_->ModelManager;
    auto& actorManager = overlay_->ActorManager;

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
            HelpMarker("VTK does not flush KeySym per frame.");
            ImGui::Text("KeyCode: %c", iren->GetKeyCode());
            ImGui::Text("Mods: %s %s %s",
                        (iren->GetAltKey() ? "ALT" : " "),
                        (iren->GetControlKey() ? "CTRL" : " "),
                        (iren->GetShiftKey() ? "SHIFT" : " "));
            ImGui::TreePop();
        }
    }

    if (ImGui::CollapsingHeader("OCL Example", ImGuiTreeNodeFlags_DefaultOpen)) {
        DrawCAMExample(overlay_);
    }


    ImGui::End();
}
