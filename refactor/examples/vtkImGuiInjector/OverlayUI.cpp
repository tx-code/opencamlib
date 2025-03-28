// SPDX-FileCopyrightText: Copyright 2023 Jaswant Panchumarti
// SPDX-License-Identifier: BSD-3-Clause

#include "OverlayUI.h"
#include "vtkCutters.h"
#include "vtkDearImGuiInjector.h"


#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <codecvt>
#include <filesystem>
#include <fstream>
#include <imgui.h>  // to draw custom UI
#include <nfd.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <vtkInteractorObserver.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSetGet.h>

#include "CutterTimerCallback.h"
#include "STLSurfUtils.h"
#include "oclBenchmark.h"
#include "oclUtils.h"


namespace
{
// 最大保存的最近文件数量
constexpr int MAX_RECENT_FILES = 10;
// 存储最近打开的文件路径
std::vector<std::string> g_recentFiles;

// 存储JSON文件的路径
constexpr char RECENT_FILES_JSON[] = "recent_files.json";
constexpr char SETTINGS_JSON[] = "ocl_settings.json";

// 默认设置值
namespace DefaultSettings
{
// Cutter设置
constexpr int cutter_type_index = 0;
constexpr double diameter = 2.0;
constexpr double length = 10.0;
constexpr double radius = 0.1;
// 使用函数获取默认值，因为不能用constexpr存储计算结果
double default_angle_in_deg()
{
    using namespace boost::math::constants;
    return radian<double>() * third_pi<double>();
}

// Operation设置
constexpr int op_type_index = 0;
constexpr double sampling = 0.1;
constexpr double min_sampling = 0.01;
constexpr double lift_step = 0.1;
constexpr double lift_from = 0.0;
constexpr double lift_to = 1.0;
}  // namespace DefaultSettings

// Cutter和Operation的设置
struct OCLSettings
{
    // Cutter设置
    int cutter_type_index = DefaultSettings::cutter_type_index;
    double diameter = DefaultSettings::diameter;
    double length = DefaultSettings::length;
    double angle_in_deg = DefaultSettings::default_angle_in_deg();
    double radius = DefaultSettings::radius;

    // Operation设置
    int op_type_index = DefaultSettings::op_type_index;
    double sampling = DefaultSettings::sampling;
    double min_sampling = DefaultSettings::min_sampling;
    double lift_step = DefaultSettings::lift_step;
    double lift_from = DefaultSettings::lift_from;
    double lift_to = DefaultSettings::lift_to;

    // 随机批量降刀特有设置
    int random_points = 10000;
};

// 全局设置对象
OCLSettings g_settings;

// 全局回调对象
vtkSmartPointer<CutterTimerCallback> g_cutterCallback;

// 从JSON文件加载最近文件列表
void LoadRecentFiles()
{
    g_recentFiles.clear();
    std::ifstream file(RECENT_FILES_JSON);

    if (file.is_open()) {
        try {
            nlohmann::json j;
            file >> j;

            if (j.contains("recent_files") && j["recent_files"].is_array()) {
                int validFileCount = 0;
                for (const auto& path : j["recent_files"]) {
                    if (path.is_string()) {
                        std::string filePath = path.get<std::string>();
                        // 检查文件是否存在
                        if (std::filesystem::exists(filePath)) {
                            g_recentFiles.push_back(filePath);
                            validFileCount++;
                        }
                    }
                }
                spdlog::info("Loaded {} recent files", validFileCount);
            }
        }
        catch (const std::exception& e) {
            spdlog::error("Error parsing recent files JSON: {}", e.what());
        }
        file.close();
    }
}

// 保存最近文件列表到JSON文件
void SaveRecentFiles()
{
    nlohmann::json j;
    j["recent_files"] = g_recentFiles;

    std::ofstream file(RECENT_FILES_JSON);
    if (file.is_open()) {
        file << j.dump(4);  // 使用4个空格缩进
        file.close();
        spdlog::info("Saved {} recent files", g_recentFiles.size());
    }
    else {
        spdlog::error("Failed to save recent files list");
    }
}

// 添加文件到最近文件列表
void AddToRecentFiles(const std::string& filePath)
{
    // 如果已经在列表中，先移除
    auto it = std::find(g_recentFiles.begin(), g_recentFiles.end(), filePath);
    if (it != g_recentFiles.end()) {
        g_recentFiles.erase(it);
    }

    // 添加到列表开头
    g_recentFiles.insert(g_recentFiles.begin(), filePath);

    // 限制列表大小
    if (g_recentFiles.size() > MAX_RECENT_FILES) {
        g_recentFiles.resize(MAX_RECENT_FILES);
    }

    // 保存到JSON文件
    SaveRecentFiles();
}

// 从JSON文件加载设置
void LoadSettings()
{
    std::ifstream file(SETTINGS_JSON);

    if (file.is_open()) {
        try {
            nlohmann::json j;
            file >> j;

            // 加载Cutter设置
            if (j.contains("cutter")) {
                auto& cutter = j["cutter"];

                if (cutter.contains("type_index"))
                    g_settings.cutter_type_index = cutter["type_index"].get<int>();

                if (cutter.contains("diameter"))
                    g_settings.diameter = cutter["diameter"].get<double>();

                if (cutter.contains("length"))
                    g_settings.length = cutter["length"].get<double>();

                if (cutter.contains("angle_in_deg"))
                    g_settings.angle_in_deg = cutter["angle_in_deg"].get<double>();

                if (cutter.contains("radius"))
                    g_settings.radius = cutter["radius"].get<double>();
            }

            // 加载Operation设置
            if (j.contains("operation")) {
                auto& op = j["operation"];

                if (op.contains("type_index"))
                    g_settings.op_type_index = op["type_index"].get<int>();

                if (op.contains("sampling"))
                    g_settings.sampling = op["sampling"].get<double>();

                if (op.contains("min_sampling"))
                    g_settings.min_sampling = op["min_sampling"].get<double>();

                if (op.contains("lift_step"))
                    g_settings.lift_step = op["lift_step"].get<double>();

                if (op.contains("lift_from"))
                    g_settings.lift_from = op["lift_from"].get<double>();

                if (op.contains("lift_to"))
                    g_settings.lift_to = op["lift_to"].get<double>();

                if (op.contains("random_points"))
                    g_settings.random_points = op["random_points"].get<int>();
            }

            spdlog::info("Settings loaded successfully");
        }
        catch (const std::exception& e) {
            spdlog::error("Error parsing settings JSON: {}", e.what());
        }
        file.close();
    }
    else {
        spdlog::info("No settings file found, using defaults");
    }
}

// 保存设置到JSON文件
void SaveSettings()
{
    nlohmann::json j;

    // 保存Cutter设置
    j["cutter"] = {{"type_index", g_settings.cutter_type_index},
                   {"diameter", g_settings.diameter},
                   {"length", g_settings.length},
                   {"angle_in_deg", g_settings.angle_in_deg},
                   {"radius", g_settings.radius}};

    // 保存Operation设置
    j["operation"] = {{"type_index", g_settings.op_type_index},
                      {"sampling", g_settings.sampling},
                      {"min_sampling", g_settings.min_sampling},
                      {"lift_step", g_settings.lift_step},
                      {"lift_from", g_settings.lift_from},
                      {"lift_to", g_settings.lift_to},
                      {"random_points", g_settings.random_points}};

    std::ofstream file(SETTINGS_JSON);
    if (file.is_open()) {
        file << j.dump(4);  // 使用4个空格缩进
        file.close();
        spdlog::info("Settings saved successfully");
    }
    else {
        spdlog::error("Failed to save settings");
    }
}

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

// 前向声明所有子函数
void DrawLoadStlUI(vtkDearImGuiInjector* injector);
void DrawCutterUI(vtkDearImGuiInjector* injector);
void DrawOperationUI(vtkDearImGuiInjector* injector);
void DrawDataModelUI(vtkDearImGuiInjector* injector);
void DrawCutterModelUI(vtkDearImGuiInjector* injector);
void DrawCutterAnimationUI(vtkDearImGuiInjector* injector, vtkPoints* points, int& pointIndex);
void DrawOperationModelUI(vtkDearImGuiInjector* injector);

// 主函数声明
void DrawCAMExample(vtkDearImGuiInjector* injector);

// 新增子函数：处理STL文件加载
void DrawLoadStlUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

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

            // 保存文件路径并添加到最近文件列表
            modelManager.stlFilePath = outPath;
            AddToRecentFiles(outPath);

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

    ImGui::SameLine();

    // 最近文件下拉菜单
    if (ImGui::BeginMenu("Recent Files")) {
        if (g_recentFiles.empty()) {
            ImGui::Text("No recent files");
        }
        else {
            for (const auto& filePath : g_recentFiles) {
                if (ImGui::MenuItem(filePath.c_str())) {
                    // 加载选择的文件
                    modelManager.surface = std::make_unique<ocl::STLSurf>();
                    ocl::STLReader reader(to_wstring(filePath.c_str()), *modelManager.surface);
                    UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                    injector->ForceResetCamera();

                    // 更新当前文件路径并重新排序最近文件列表
                    modelManager.stlFilePath = filePath;
                    // AddToRecentFiles(filePath);
                }
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Clear Recent Files")) {
                g_recentFiles.clear();
                SaveRecentFiles();
            }
        }
        ImGui::EndMenu();
    }
}

// 新增子函数：处理刀具添加
void DrawCutterUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (ImGui::Button("Add Cutter")) {
        ImGui::OpenPopup("###Add Cutter");
    }

    /// The popup menu for adding a cutter
    if (ImGui::BeginPopup("###Add Cutter", ImGuiWindowFlags_AlwaysAutoResize)) {
        static const char* cutter_types[] = {"CylCutter", "BallCutter", "BullCutter", "ConeCutter"};

        // 使用全局设置
        bool changed = false;

        changed |= ImGui::Combo("Cutter Types",
                                &g_settings.cutter_type_index,
                                cutter_types,
                                IM_ARRAYSIZE(cutter_types));

        using namespace boost::math::constants;

        if (g_settings.cutter_type_index == 0) {
            changed |= ImGui::InputDouble("Diameter", &g_settings.diameter, 0.01f, 1.0f, "%.3f");
            changed |= ImGui::InputDouble("Length", &g_settings.length, 0.01f, 1.0f, "%.3f");
        }
        else if (g_settings.cutter_type_index == 1) {
            changed |= ImGui::InputDouble("Diameter", &g_settings.diameter, 0.01f, 1.0f, "%.3f");
            changed |= ImGui::InputDouble("Length", &g_settings.length, 0.01f, 1.0f, "%.3f");
        }
        else if (g_settings.cutter_type_index == 2) {
            changed |= ImGui::InputDouble("Diameter", &g_settings.diameter, 0.01f, 1.0f, "%.3f");
            changed |= ImGui::InputDouble("Length", &g_settings.length, 0.01f, 1.0f, "%.3f");
            changed |= ImGui::InputDouble("Radius", &g_settings.radius, 0.01f, 1.0f, "%.3f");
        }
        else if (g_settings.cutter_type_index == 3) {
            changed |= ImGui::InputDouble("Diameter", &g_settings.diameter, 0.01f, 1.0f, "%.3f");
            changed |= ImGui::InputDouble("Length", &g_settings.length, 0.01f, 1.0f, "%.3f");
            changed |= ImGui::InputDouble("Angle", &g_settings.angle_in_deg, 0.01f, 1.0f, "%.3f");
        }

        // 如果有变化，保存设置
        if (changed) {
            SaveSettings();
        }

        if (ImGui::Button("Ok")) {
            // According the cutter_type_index, change the cutter type
            if (g_settings.cutter_type_index == 0) {
                modelManager.cutter =
                    std::make_unique<ocl::CylCutter>(g_settings.diameter, g_settings.length);
                spdlog::info("CylCutter created: {}", modelManager.cutter->str());
            }
            else if (g_settings.cutter_type_index == 1) {
                modelManager.cutter =
                    std::make_unique<ocl::BallCutter>(g_settings.diameter, g_settings.length);
                spdlog::info("BallCutter created: {}", modelManager.cutter->str());
            }
            else if (g_settings.cutter_type_index == 2) {
                modelManager.cutter = std::make_unique<ocl::BullCutter>(g_settings.diameter,
                                                                        g_settings.radius,
                                                                        g_settings.length);
                spdlog::info("BullCutter created: {}", modelManager.cutter->str());
            }
            else if (g_settings.cutter_type_index == 3) {
                modelManager.cutter =
                    std::make_unique<ocl::ConeCutter>(g_settings.diameter,
                                                      degree<double>() * g_settings.angle_in_deg,
                                                      g_settings.length);
                spdlog::info("ConeCutter created: {}", modelManager.cutter->str());
            }
            UpdateCutterActor(actorManager.cutterActor, *modelManager.cutter, ocl::Point(0, 0, 0));
            injector->ForceResetCamera();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// 新增子函数：处理操作添加
void DrawOperationUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (ImGui::Button("Add Operation")) {
        ImGui::OpenPopup("###Add Operation");
    }

    /// The popup menu for adding a operation
    if (ImGui::BeginPopup("###Add Operation", ImGuiWindowFlags_AlwaysAutoResize)) {
        static const char* op_types[] = {"WaterLine",
                                         "AdaptiveWaterLine",
                                         "PathDropCutter",
                                         "AdaptivePathDropCutter",
                                         "RandomBatchDropCutter"};

        // 使用全局设置
        bool changed = false;

        changed |= ImGui::Combo("Operation Types",
                                &g_settings.op_type_index,
                                op_types,
                                IM_ARRAYSIZE(op_types));

        switch (g_settings.op_type_index) {
            case 0:
                changed |=
                    ImGui::InputDouble("Sampling", &g_settings.sampling, 0.01f, 1.0f, "%.3f");
                changed |=
                    ImGui::InputDouble("Lift Step", &g_settings.lift_step, 0.01f, 1.0f, "%.3f");
                changed |=
                    ImGui::InputDouble("Lift From", &g_settings.lift_from, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Lift To", &g_settings.lift_to, 0.01f, 1.0f, "%.3f");
                break;
            case 1:
                changed |=
                    ImGui::InputDouble("Sampling", &g_settings.sampling, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Min Sampling",
                                              &g_settings.min_sampling,
                                              0.01f,
                                              1.0f,
                                              "%.3f");
                changed |=
                    ImGui::InputDouble("Lift Step", &g_settings.lift_step, 0.01f, 1.0f, "%.3f");
                changed |=
                    ImGui::InputDouble("Lift From", &g_settings.lift_from, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Lift To", &g_settings.lift_to, 0.01f, 1.0f, "%.3f");
                break;
            case 2:
                changed |=
                    ImGui::InputDouble("Sampling", &g_settings.sampling, 0.01f, 1.0f, "%.3f");
                break;
            case 3:
                changed |=
                    ImGui::InputDouble("Sampling", &g_settings.sampling, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Min Sampling",
                                              &g_settings.min_sampling,
                                              0.01f,
                                              1.0f,
                                              "%.3f");
                break;
            case 4:
                changed |=
                    ImGui::InputDouble("Sampling", &g_settings.sampling, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputInt("Random Points", &g_settings.random_points, 1000, 10000);
                break;
            default:
                break;
        }

        // 如果有变化，保存设置
        if (changed) {
            SaveSettings();
        }

        if (ImGui::Button("Run Operation")) {
            if (modelManager.cutter && modelManager.surface) {
                switch (g_settings.op_type_index) {
                    case 0:
                        waterline(modelManager,
                                  actorManager,
                                  g_settings.sampling,
                                  g_settings.lift_to,
                                  g_settings.lift_step,
                                  g_settings.lift_from);
                        break;
                    case 1:
                        adaptiveWaterline(modelManager,
                                          actorManager,
                                          g_settings.sampling,
                                          g_settings.min_sampling,
                                          g_settings.lift_to,
                                          g_settings.lift_step,
                                          g_settings.lift_from);
                        break;
                    case 2:
                        pathDropCutter(modelManager, actorManager, g_settings.sampling);
                        break;
                    case 3:
                        adaptivePathDropCutter(modelManager,
                                               actorManager,
                                               g_settings.sampling,
                                               g_settings.min_sampling);
                        break;
                    case 4:
                        randomBatchDropCutter(modelManager,
                                              actorManager,
                                              g_settings.sampling,
                                              g_settings.random_points);
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
}

// 新增子函数：处理数据模型UI显示
void DrawDataModelUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

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

            // 显示当前STL文件路径
            if (!modelManager.stlFilePath.empty()) {
                ImGui::TextWrapped("File: %s", modelManager.stlFilePath.c_str());
            }

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

            // Random Perturbation
            static double max_move_distance = 0.01;
            ImGui::InputDouble("Max Move Distance", &max_move_distance, 0.01f, 1.0f, "%.3f");
            if (ImGui::Button("Random Perturbation")) {
                RandomPerturbation(*modelManager.surface, max_move_distance, true);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
            }
        }
        else {
            ImGui::TextDisabled("No WorkPiece");
        }

        ImGui::TreePop();
    }
}

// 新增子函数：处理刀具UI显示
void DrawCutterModelUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

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
}

// 新增子函数：显示刀具动画控制UI
void DrawCutterAnimationUI(vtkDearImGuiInjector* injector, vtkPoints* points, int& pointIndex)
{
    auto& actorManager = injector->ActorManager;

    // 添加定时器动画区域
    ImGui::SeparatorText("Timer Animation");

    // 动画参数设置
    static int intervalMs = 200;  // 默认每200毫秒移动一步
    bool intervalChanged = ImGui::SliderInt("Interval (ms)", &intervalMs, 50, 1000);

    // 播放按钮
    ImGui::BeginDisabled(g_cutterCallback && g_cutterCallback->TimerId > 0);
    if (ImGui::Button("Play (Timer)")) {
        // 首次创建回调和定时器
        if (!g_cutterCallback) {
            g_cutterCallback = vtkSmartPointer<CutterTimerCallback>::New();
        }

        // 重置回调状态
        g_cutterCallback->Reset();

        // 设置动画参数
        g_cutterCallback->SetActor(actorManager.cutterActor);
        g_cutterCallback->SetPoints(points);
        g_cutterCallback->SetStartIndex(pointIndex);

        // 计算总步数
        int remainingPoints = points->GetNumberOfPoints() - pointIndex;
        g_cutterCallback->SetMaxSteps(remainingPoints);

        // 创建定时器（如果不存在）
        if (g_cutterCallback->TimerId <= 0) {
            int timerId = injector->Interactor->CreateRepeatingTimer(intervalMs);
            g_cutterCallback->SetTimerId(timerId);

            // 只需添加一次观察者
            injector->Interactor->AddObserver(vtkCommand::TimerEvent, g_cutterCallback);
            spdlog::info("Created timer with ID: {}", timerId);
        }
        else if (intervalChanged) {
            // 如果定时器间隔改变，更新定时器
            injector->Interactor->DestroyTimer(g_cutterCallback->TimerId);
            int timerId = injector->Interactor->CreateRepeatingTimer(intervalMs);
            g_cutterCallback->SetTimerId(timerId);
            spdlog::info("Timer updated with new interval: {}ms", intervalMs);
        }

        // 没有显式启动方法了，定时器ID存在就是播放状态
        spdlog::info("Starting timer animation from point {} with interval {}ms",
                     pointIndex,
                     intervalMs);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();

    // Stop button - 只在有效定时器存在时启用
    ImGui::BeginDisabled(!g_cutterCallback || g_cutterCallback->TimerId <= 0);
    if (ImGui::Button("Stop")) {
        if (g_cutterCallback && g_cutterCallback->TimerId > 0) {
            // 更新当前索引
            pointIndex = g_cutterCallback->CurrentIndex;

            // 停止动画并销毁定时器
            g_cutterCallback->Stop(injector->Interactor);
            spdlog::info("Animation stopped manually");
        }
    }
    ImGui::EndDisabled();
}

// 新增子函数：处理操作UI显示
void DrawOperationModelUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (modelManager.operation) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    if (ImGui::TreeNode("Operations")) {
        if (auto& op = modelManager.operation) {
            auto operation = actorManager.operationActor;
            ImGui::Text(operation->GetObjectName().c_str());
            bool visible = operation->GetVisibility();
            ImGui::Checkbox("Show Operation", &visible);
            operation->SetVisibility(visible);

            // TODO: 将Source、Mapper、Actor封装到一起
            auto* mapper = operation->GetMapper();
            assert(mapper);

            auto* polyData = vtkPolyData::SafeDownCast(mapper->GetInput());
            if (polyData) {
                auto* points = polyData->GetPoints();
                auto* lines = polyData->GetLines();

                ImGui::Text("Points: %d, Lines: %d, Polys: %d",
                            points->GetNumberOfPoints(),
                            lines->GetNumberOfCells());

                // Use a int slider to "move" the cutter to the specified point
                static int pointIndex = 0;
                static bool checkCutterLocation = false;
                ImGui::Checkbox("Check Cutter Location", &checkCutterLocation);
                if (checkCutterLocation) {
                    ImGui::Text("Move the cutter to the specified point");
                    if (ImGui::SliderInt("Point Index",
                                         &pointIndex,
                                         0,
                                         points->GetNumberOfPoints() - 1)) {
                        auto* point = points->GetPoint(pointIndex);

                        actorManager.cutterActor->SetPosition(point);
                    }

                    DrawCutterAnimationUI(injector, points, pointIndex);
                }
            }
        }
        else {
            ImGui::TextDisabled("No Operation");
        }
        ImGui::TreePop();
    }
}

// 主函数，调用各个子函数
void DrawCAMExample(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (ImGui::BeginMenu("OCL Operations")) {
        DrawLoadStlUI(injector);
        DrawCutterUI(injector);
        DrawOperationUI(injector);
        if (ImGui::BeginMenu("OCL Benchmark")) {
            static bool verbose = true;
            ImGui::Checkbox("Verbose", &verbose);
            if (ImGui::Button("Run BatchDropCutter")) {
                if (modelManager.cutter && modelManager.surface) {
                    spdlog::info("=====Begin Benchmark=====");
                    spdlog::info("Use Cutter {} and Surface {}",
                                 modelManager.cutter->str(),
                                 modelManager.stlFilePath);
                    run_batchdropcutter(*modelManager.surface, *modelManager.cutter, verbose);
                    spdlog::info("=====End Benchmark=====");
                }
                else {
                    spdlog::error("No cutter or surface");
                }
            }

            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }

    DrawDataModelUI(injector);
    DrawCutterModelUI(injector);
    DrawOperationModelUI(injector);
}
}  // namespace

//------------------------------------------------------------------------------
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
        LoadRecentFiles();
        LoadSettings();
        spdlog::info("OverlayUI::setup - Setup completed");
    }
    else {
        spdlog::error("OverlayUI::setup - ImGui initialization failed");
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
