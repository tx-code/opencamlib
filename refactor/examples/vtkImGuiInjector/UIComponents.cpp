#include "UIComponents.h"

#include "AABBTreeAdaptor.h"
#include "CutterTimerCallback.h"
#include "DialogHelpers.h"
#include "RecentFilesManager.h"
#include "STLSurfUtils.h"
#include "SettingsManager.h"
#include "geo/triangle.hpp"
#include "oclBenchmark.h"
#include "oclUtils.h"
#include "vtkCutters.h"
#include "vtkDearImGuiInjector.h"
#include "vtkUtils.h"


#include <CGAL/Memory_sizer.h>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <vtkLineRepresentation.h>
#include <vtkMapper.h>
#include <vtkPlane.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>


namespace ocl
{

// 全局回调对象
static vtkSmartPointer<CutterTimerCallback> g_cutterCallback;

// 用于Debug DropCutter的静态变量
static std::vector<ocl::CLPoint> g_debugResultPoints;
static int g_debugCurrentPointIndex = 0;
static bool g_showDebugWindow = false;  // 控制调试窗口显示

void UIComponents::DrawLoadStlUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (ImGui::Button("Load Workpiece")) {
        std::string filePath;
        if (DialogHelpers::OpenWorkpieceFileDialog(filePath)) {
            modelManager.surface = std::make_unique<ocl::STLSurf>();
            ReadPolygonMesh(filePath, *modelManager.surface);
            UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
            injector->ForceResetCamera();

            // 保存文件路径并添加到最近文件列表
            modelManager.stlFilePath = filePath;
            RecentFilesManager::AddToRecentFiles(filePath);

            // 重建AABBTree
            modelManager.rebuildAABBTree();

            // 隐藏重叠三角形actor
            actorManager.debugActor->VisibilityOff();
        }
    }

    ImGui::SameLine();

    // 最近文件下拉菜单
    if (ImGui::BeginMenu("Recent Files")) {
        const auto& recentFiles = RecentFilesManager::GetRecentFiles();
        if (recentFiles.empty()) {
            ImGui::Text("No recent files");
        }
        else {
            for (const auto& filePath : recentFiles) {
                if (ImGui::MenuItem(filePath.c_str())) {
                    // 加载选择的文件
                    modelManager.surface = std::make_unique<ocl::STLSurf>();
                    ReadPolygonMesh(filePath, *modelManager.surface);
                    UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                    injector->ForceResetCamera();

                    // 更新当前文件路径
                    modelManager.stlFilePath = filePath;

                    // 重建AABBTree
                    modelManager.rebuildAABBTree();

                    // 隐藏重叠三角形actor
                    actorManager.debugActor->VisibilityOff();
                }
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Clear Recent Files")) {
                RecentFilesManager::ClearRecentFiles();
            }
        }
        ImGui::EndMenu();
    }
}

// 独立显示的窗口
void UIComponents::DrawCutterUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    // 刀具窗口
    auto& settings = SettingsManager::GetSettings();
    if (settings.show_cutter_window) {
        ImGui::SetNextWindowSize(ImVec2(400, 350), ImGuiCond_FirstUseEver);

        // 使用固定位置的窗口，靠近屏幕左侧
        ImVec2 viewportSize = ImGui::GetMainViewport()->Size;
        ImVec2 windowPos(20, 100);
        ImGui::SetNextWindowPos(windowPos, ImGuiCond_FirstUseEver);

        if (ImGui::Begin("Add Cutter",
                         &settings.show_cutter_window,
                         ImGuiWindowFlags_AlwaysAutoResize)) {
            static const char* cutter_types[] = {"CylCutter",
                                                 "BallCutter",
                                                 "BullCutter",
                                                 "ConeCutter"};

            // 使用全局设置
            bool changed = false;

            changed |= ImGui::Combo("Cutter Types",
                                    &settings.cutter_type_index,
                                    cutter_types,
                                    IM_ARRAYSIZE(cutter_types));

            using namespace boost::math::constants;

            if (settings.cutter_type_index == 0) {
                changed |= ImGui::InputDouble("Diameter", &settings.diameter, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Length", &settings.length, 0.01f, 1.0f, "%.3f");
            }
            else if (settings.cutter_type_index == 1) {
                changed |= ImGui::InputDouble("Diameter", &settings.diameter, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Length", &settings.length, 0.01f, 1.0f, "%.3f");
            }
            else if (settings.cutter_type_index == 2) {
                changed |= ImGui::InputDouble("Diameter", &settings.diameter, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Length", &settings.length, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Radius", &settings.radius, 0.01f, 1.0f, "%.3f");
            }
            else if (settings.cutter_type_index == 3) {
                changed |= ImGui::InputDouble("Diameter", &settings.diameter, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Length", &settings.length, 0.01f, 1.0f, "%.3f");
                changed |= ImGui::InputDouble("Angle", &settings.angle_in_deg, 0.01f, 1.0f, "%.3f");
            }


            if (ImGui::Button("Ok")) {
                // 根据cutter_type_index创建相应类型的刀具
                if (settings.cutter_type_index == 0) {
                    modelManager.cutter =
                        std::make_unique<ocl::CylCutter>(settings.diameter, settings.length);
                    spdlog::info("CylCutter created: {}", modelManager.cutter->str());
                }
                else if (settings.cutter_type_index == 1) {
                    modelManager.cutter =
                        std::make_unique<ocl::BallCutter>(settings.diameter, settings.length);
                    spdlog::info("BallCutter created: {}", modelManager.cutter->str());
                }
                else if (settings.cutter_type_index == 2) {
                    modelManager.cutter = std::make_unique<ocl::BullCutter>(settings.diameter,
                                                                            settings.radius,
                                                                            settings.length);
                    spdlog::info("BullCutter created: {}", modelManager.cutter->str());
                }
                else if (settings.cutter_type_index == 3) {
                    modelManager.cutter =
                        std::make_unique<ocl::ConeCutter>(settings.diameter,
                                                          degree<double>() * settings.angle_in_deg,
                                                          settings.length);
                    spdlog::info("ConeCutter created: {}", modelManager.cutter->str());
                }
                UpdateCutterActor(actorManager.cutterActor,
                                  *modelManager.cutter,
                                  ocl::Point(0, 0, 0));
                injector->ForceResetCamera();

                // 隐藏重叠三角形actor
                actorManager.debugActor->VisibilityOff();
            }
        }
        ImGui::End();
    }
}

// 独立显示的窗口
void UIComponents::DrawOperationUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    // 操作窗口
    auto& settings = SettingsManager::GetSettings();
    if (settings.show_operation_window) {
        ImGui::SetNextWindowSize(ImVec2(400, 420), ImGuiCond_FirstUseEver);

        // 使用固定位置的窗口，靠近屏幕左侧
        ImVec2 viewportSize = ImGui::GetMainViewport()->Size;
        ImVec2 windowPos(20, 470);
        ImGui::SetNextWindowPos(windowPos, ImGuiCond_FirstUseEver);

        if (ImGui::Begin("Add Operation",
                         &settings.show_operation_window,
                         ImGuiWindowFlags_AlwaysAutoResize)) {
            static const char* op_types[] = {"WaterLine",
                                             "AdaptiveWaterLine",
                                             "PathDropCutter",
                                             "AdaptivePathDropCutter",
                                             "RandomBatchDropCutter",
                                             "BatchFiberPushCutter"};

            // 使用全局设置
            bool changed = false;

            changed |= ImGui::Combo("Operation Types",
                                    &settings.op_type_index,
                                    op_types,
                                    IM_ARRAYSIZE(op_types));

            switch (settings.op_type_index) {
                case 0:  // waterline
                case 5:  // batch fiber push cutter
                    changed |=
                        ImGui::InputDouble("Sampling", &settings.sampling, 0.01f, 1.0f, "%.3f");
                    changed |=
                        ImGui::InputDouble("Lift Step", &settings.lift_step, 0.01f, 1.0f, "%.3f");
                    changed |=
                        ImGui::InputDouble("Lift From", &settings.lift_from, 0.01f, 1.0f, "%.3f");
                    changed |=
                        ImGui::InputDouble("Lift To", &settings.lift_to, 0.01f, 1.0f, "%.3f");
                    break;
                case 1:
                    changed |=
                        ImGui::InputDouble("Sampling", &settings.sampling, 0.01f, 1.0f, "%.3f");
                    changed |= ImGui::InputDouble("Min Sampling",
                                                  &settings.min_sampling,
                                                  0.01f,
                                                  1.0f,
                                                  "%.3f");
                    changed |=
                        ImGui::InputDouble("Lift Step", &settings.lift_step, 0.01f, 1.0f, "%.3f");
                    changed |=
                        ImGui::InputDouble("Lift From", &settings.lift_from, 0.01f, 1.0f, "%.3f");
                    changed |=
                        ImGui::InputDouble("Lift To", &settings.lift_to, 0.01f, 1.0f, "%.3f");
                    break;
                case 2:
                    changed |=
                        ImGui::InputDouble("Sampling", &settings.sampling, 0.01f, 1.0f, "%.3f");
                    break;
                case 3:
                    changed |=
                        ImGui::InputDouble("Sampling", &settings.sampling, 0.01f, 1.0f, "%.3f");
                    changed |= ImGui::InputDouble("Min Sampling",
                                                  &settings.min_sampling,
                                                  0.01f,
                                                  1.0f,
                                                  "%.3f");
                    break;
                case 4:
                    changed |=
                        ImGui::InputDouble("Sampling", &settings.sampling, 0.01f, 1.0f, "%.3f");
                    changed |=
                        ImGui::InputInt("Random Points", &settings.random_points, 1000, 10000);
                    break;
                default:
                    break;
            }

            ImGui::BeginDisabled(!modelManager.cutter || !modelManager.surface);
            if (ImGui::Button("Run Operation")) {
                if (modelManager.cutter && modelManager.surface) {
                    // 隐藏重叠三角形actor
                    actorManager.debugActor->VisibilityOff();

                    switch (settings.op_type_index) {
                        case 0: {
                            waterline(modelManager,
                                      actorManager,
                                      settings.sampling,
                                      settings.lift_to,
                                      settings.lift_step,
                                      settings.lift_from);
                            break;
                        }
                        case 1:
                            adaptiveWaterline(modelManager,
                                              actorManager,
                                              settings.sampling,
                                              settings.min_sampling,
                                              settings.lift_to,
                                              settings.lift_step,
                                              settings.lift_from);
                            break;
                        case 2:
                            pathDropCutter(modelManager, actorManager, settings.sampling);
                            break;
                        case 3:
                            adaptivePathDropCutter(modelManager,
                                                   actorManager,
                                                   settings.sampling,
                                                   settings.min_sampling);
                            break;
                        case 4:
                            randomBatchDropCutter(modelManager,
                                                  actorManager,
                                                  settings.sampling,
                                                  settings.random_points);
                            break;
                        case 5:
                            batchFiberPushCutter(modelManager,
                                                 actorManager,
                                                 settings.sampling,
                                                 settings.lift_to,
                                                 settings.lift_step,
                                                 settings.lift_from);
                            break;
                    }
                    injector->ForceResetCamera();
                }
            }
            ImGui::EndDisabled();
        }
        ImGui::End();
    }
}

void UIComponents::DrawDataModelUI(vtkDearImGuiInjector* injector)
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

            if (ImGui::BeginMenu("Advanced")) {
                ImGui::SeparatorText("Perturbation");
                // Random Perturbation
                static double max_move_distance = 0.01;
                ImGui::InputDouble("Max Move Distance", &max_move_distance, 0.01f, 1.0f, "%.3f");
                if (ImGui::Button("Random Perturbation")) {
                    RandomPerturbation(*modelManager.surface, max_move_distance, true);
                    UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                    // 重建AABBTree
                    modelManager.rebuildAABBTree();
                }

                ImGui::SeparatorText("Subdivision");
                if (ImGui::Button("Subdivision once")) {
                    SubdivideSurface(*modelManager.surface);
                    UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                    // 重建AABBTree
                    modelManager.rebuildAABBTree();
                }

                ImGui::SeparatorText("Tree");
                static int treeType = 0;
                static bool showTree = false;
                static bool onlyLeafNodes = false;
                bool updateTree = false;
                updateTree |= ImGui::Checkbox("Show Tree", &showTree);
                ImGui::SameLine();
                updateTree |= ImGui::Checkbox("Only leaf nodes", &onlyLeafNodes);
                updateTree |= ImGui::RadioButton("KDTree", &treeType, 0);
                ImGui::SameLine();
                updateTree |= ImGui::RadioButton("AABBTree", &treeType, 1);
                if (showTree && updateTree) {
                    // Although we can cache the tree, we rebuild it here for the sake of simplicity
                    // and test memory usage
                    if (treeType == 0) {
                        auto memory_before = CGAL::Memory_sizer().virtual_size();
                        ocl::KDTree<ocl::Triangle> kdtree;
                        kdtree.setBucketSize(1);
                        kdtree.setXYDimensions();
                        kdtree.build(modelManager.surface->tris);
                        spdlog::info("KDTree allocated {} MB",
                                     (CGAL::Memory_sizer().virtual_size() - memory_before) >> 20);
                        actorManager.treeActor->VisibilityOn();
                        UpdateKDTreeActor(actorManager.treeActor, &kdtree, 0.4, onlyLeafNodes);
                    }
                    else {
                        auto memory_before = CGAL::Memory_sizer().virtual_size();
                        ocl::AABBTreeAdaptor aabbTree;
                        aabbTree.build(modelManager.surface->tris);
                        spdlog::info("AABBTree allocated {} MB",
                                     (CGAL::Memory_sizer().virtual_size() - memory_before) >> 20);
                        actorManager.treeActor->VisibilityOn();
                        UpdateAABBTreeActor(actorManager.treeActor, aabbTree, 0.4);
                    }
                }
                else if (!showTree && updateTree) {
                    actorManager.treeActor->VisibilityOff();
                }

                ImGui::SeparatorText("Sample Points");
                static bool showSamplePoints = false;
                static int number_points = 1e4;
                ImGui::DragInt("Number of Sample Points", &number_points, 10.f, 100, 1e7);
                if (ImGui::Checkbox("Show Sample Points", &showSamplePoints)) {
                    if (showSamplePoints) {
                        Eigen::MatrixXd P;
                        Eigen::MatrixXd N;
                        SampleMeshForPointCloud(*modelManager.surface, number_points, P, N);
                        UpdatePointCloudActor(actorManager.debugActor, P, N);
                        actorManager.debugActor->VisibilityOn();
                    }
                    else {
                        actorManager.debugActor->VisibilityOff();
                    }
                }
                ImGui::EndMenu();
            }
        }
        else {
            ImGui::TextDisabled("No WorkPiece");
        }

        ImGui::TreePop();
    }
}

void UIComponents::DrawCutterModelUI(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (modelManager.cutter) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    if (ImGui::TreeNode("Cutters")) {
        if (modelManager.cutter) {
            auto& cutter = actorManager.cutterActor;
            ImGui::Text(cutter->GetObjectName().c_str());
            bool visible = cutter->GetVisibility();
            ImGui::Checkbox("Show Cutter", &visible);
            cutter->SetVisibility(visible);

            static bool moveCutter = false;
            ImGui::Checkbox("Move", &moveCutter);
            ImGui::SameLine();
            ImGui::BeginDisabled(!moveCutter);
            double pos[3];
            double* actorPos = cutter->GetPosition();
            std::copy(actorPos, actorPos + 3, pos);
            static double minPos = -1e5;
            static double maxPos = 1e5;
            if (ImGui::DragScalarN("Position",
                                   ImGuiDataType_Double,
                                   pos,
                                   3,
                                   0.1f,
                                   &minPos,
                                   &maxPos,
                                   "%.3f")) {
                cutter->SetPosition(pos);
            }
            ImGui::EndDisabled();

            int representation = cutter->GetProperty()->GetRepresentation();
            std::array<bool, 2> touched;
            touched[0] = ImGui::RadioButton("Wireframe", &representation, 1);
            ImGui::SameLine();
            touched[1] = ImGui::RadioButton("Surface", &representation, 2);
            if (std::find(touched.begin(), touched.end(), true) != touched.end()) {
                cutter->GetProperty()->SetRepresentation(representation);
            }

            if (modelManager.surface) {
                if (ImGui::BeginMenu("Advanced")) {
                    bool debugVisible = actorManager.debugActor->GetVisibility();
                    ImGui::MenuItem("Show DebugActor", nullptr, &debugVisible);
                    actorManager.debugActor->SetVisibility(debugVisible);
                    if (ImGui::Button("Test Overlaps")) {
                        // 使用缓存的AABBTree
                        ocl::CLPoint cl(pos[0], pos[1], pos[2]);

                        // 如果AABBTree不存在，先构建它
                        if (!modelManager.aabbTree) {
                            modelManager.rebuildAABBTree();
                        }

                        std::vector<ocl::Triangle> res;
                        if (modelManager.aabbTree) {
                            res = modelManager.aabbTree->search_cutter_overlap(
                                modelManager.cutter.get(),
                                &cl);
                            spdlog::info("Found {} triangles overlapped by the cutter", res.size());
                        }
                        else {
                            spdlog::error("Failed to build AABBTree");
                        }

                        // 可视化被刀具覆盖的三角形
                        if (!res.empty()) {
                            UpdateOverlappedTrianglesActor(actorManager.debugActor, res);
                            actorManager.debugActor->VisibilityOn();
                        }
                        else {
                            actorManager.debugActor->VisibilityOff();
                        }
                    }
                    if (ImGui::Button("Debug Point DropCutter")) {
                        auto res = debugPointDropCutter(modelManager,
                                                        ocl::CLPoint(pos[0], pos[1], pos[2]));
                        UpdateCLPointCloudActor(actorManager.debugActor,
                                                actorManager.legendActor,
                                                res);

                        // 当结果不为空时，设置标志以显示调试窗口
                        if (!res.empty()) {
                            // 保存结果点到全局变量，以便在窗口中使用
                            g_debugResultPoints = res;
                            g_debugCurrentPointIndex = 0;

                            // 显示调试窗口
                            g_showDebugWindow = true;

                            actorManager.debugActor->VisibilityOn();
                            spdlog::info("Found {} CC points for debugging", res.size());
                        }
                        else {
                            actorManager.debugActor->VisibilityOff();
                            spdlog::warn("No CC points found for debugging");
                        }
                    }

                    ImGui::EndMenu();
                }
            }
        }
        else {
            ImGui::TextDisabled("No Cutter");
        }
        ImGui::TreePop();
    }
}

void UIComponents::DrawCutterAnimationUI(vtkDearImGuiInjector* injector,
                                         vtkPoints* points,
                                         int& pointIndex)
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

void UIComponents::DrawOperationModelUI(vtkDearImGuiInjector* injector)
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

            auto* mapper = operation->GetMapper();
            assert(mapper);

            auto* polyData = vtkPolyData::SafeDownCast(mapper->GetInput());
            if (polyData) {
                auto* points = polyData->GetPoints();
                auto* lines = polyData->GetLines();

                ImGui::Text("Points: %d, Lines: %d, Polys: %d",
                            points->GetNumberOfPoints(),
                            lines->GetNumberOfCells());

                // 使用滑块"移动"刀具到指定点
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

        // 使用全局设置
        auto& settings = SettingsManager::GetSettings();
        auto* bounds = actorManager.modelActor->GetBounds();
        ImGui::SeparatorText("Single Operation");
        ImGui::BeginDisabled(!modelManager.cutter || !modelManager.surface);
        // SingleOp Waterline
        if (ImGui::CollapsingHeader("Single Waterline")) {
            bool enablePlaneWidget = actorManager.planeWidget->GetEnabled();
            if (ImGui::Checkbox("Enable Plane Widget", &enablePlaneWidget)) {
                // checked the CheckBox will update the plane widget

                auto* planeRep = actorManager.planeWidget->GetImplicitPlaneRepresentation();
                planeRep->PlaceWidget(bounds);
                planeRep->GetUnderlyingPlane()->SetOrigin((bounds[0] + bounds[1]) / 2.,
                                                          (bounds[2] + bounds[3]) / 2.,
                                                          (bounds[4] + bounds[5]) / 2.);
                if (enablePlaneWidget) {
                    actorManager.planeWidget->On();
                }
                else {
                    actorManager.planeWidget->Off();
                }
            }

            if (enablePlaneWidget) {
                // Now, we are using the plane widget to get the z-height
                auto* planeRep = actorManager.planeWidget->GetImplicitPlaneRepresentation();
                double pos[3];
                planeRep->GetOrigin(pos);
                ImGui::Text("Plane Position: %.3f, %.3f, %.3f", pos[0], pos[1], pos[2]);
                static double sampling_ = 0.1;
                ImGui::InputDouble("Sampling", &sampling_, 0.01f, 1.0f, "%.3f");
                if (ImGui::Button("Run Single Waterline")) {
                    singleWaterline(modelManager, actorManager, sampling_, pos[2], true);
                }
            }
        }

        // Single FiberPushCutter
        if (ImGui::CollapsingHeader("Single FiberPushCutter")) {
            auto* lineRep = actorManager.lineWidget->GetLineRepresentation();
            auto* pointHandle1 = lineRep->GetPoint1Representation();
            auto* pointHandle2 = lineRep->GetPoint2Representation();
            // 4 directions, +X,-X,+Y,-Y
            static int direction = 0;
            static bool first = true;
            bool needUpdated = ImGui::Combo("Direction", &direction, "X+\0-X\0+Y\0-Y\0\0");
            if (needUpdated || first) {
                first = false;
                if (direction == 0 || direction == 1) {
                    double point1[3] = {bounds[0],
                                        (bounds[2] + bounds[3]) / 2.,
                                        (bounds[4] + bounds[5]) / 2.};
                    double point2[3] = {bounds[1],
                                        (bounds[2] + bounds[3]) / 2.,
                                        (bounds[4] + bounds[5]) / 2.};
                    if (direction == 0) {
                        lineRep->SetPoint1WorldPosition(point1);
                        lineRep->SetPoint2WorldPosition(point2);
                    }
                    else {
                        lineRep->SetPoint1WorldPosition(point2);
                        lineRep->SetPoint2WorldPosition(point1);
                    }
                }
                else if (direction == 2 || direction == 3) {
                    double point1[3] = {(bounds[0] + bounds[1]) / 2.,
                                        bounds[2],
                                        (bounds[4] + bounds[5]) / 2.};
                    double point2[3] = {(bounds[0] + bounds[1]) / 2.,
                                        bounds[3],
                                        (bounds[4] + bounds[5]) / 2.};
                    if (direction == 2) {
                        lineRep->SetPoint1WorldPosition(point1);
                        lineRep->SetPoint2WorldPosition(point2);
                    }
                    else {
                        lineRep->SetPoint1WorldPosition(point2);
                        lineRep->SetPoint2WorldPosition(point1);
                    }
                }
            }
            bool enableLineWidget = actorManager.lineWidget->GetEnabled();
            needUpdated = ImGui::Checkbox("Enable Line Widget", &enableLineWidget);
            if (needUpdated) {
                // checked the CheckBox will update the line widget
                if (enableLineWidget) {
                    actorManager.lineWidget->On();
                }
                else {
                    actorManager.lineWidget->Off();
                }
            }

            if (enableLineWidget) {
                double start[3], end[3];
                pointHandle1->GetWorldPosition(start);
                pointHandle2->GetWorldPosition(end);

                // 约束线段方向与选定的坐标轴平行
                if (pointHandle1->GetMTime() < pointHandle2->GetMTime()) {
                    // 刚刚移动了Point2，调整Start点，保持轴向一致
                    if (direction == 0 || direction == 1) {
                        // X轴方向，保持Y和Z不变
                        start[1] = end[1];
                        start[2] = end[2];
                    }
                    else {
                        // Y轴方向，保持X和Z不变
                        start[0] = end[0];
                        start[2] = end[2];
                    }
                    pointHandle1->SetWorldPosition(start);
                }
                else if (pointHandle1->GetMTime() > pointHandle2->GetMTime()) {
                    // 刚刚移动了Point1，调整End点，保持轴向一致
                    if (direction == 0 || direction == 1) {
                        // X轴方向，保持Y和Z不变
                        end[1] = start[1];
                        end[2] = start[2];
                    }
                    else {
                        // Y轴方向，保持X和Z不变
                        end[0] = start[0];
                        end[2] = start[2];
                    }
                    pointHandle2->SetWorldPosition(end);
                }

                ImGui::Text("Line Start: %.3f, %.3f, %.3f", start[0], start[1], start[2]);
                ImGui::Text("Line End: %.3f, %.3f, %.3f", end[0], end[1], end[2]);

                if (ImGui::Button("Run FiberPushCutter")) {
                    fiberPushCutter(modelManager,
                                    actorManager,
                                    Eigen::Vector3d(start[0], start[1], start[2]),
                                    Eigen::Vector3d(end[0], end[1], end[2]),
                                    true);
                }
            }
        }
        ImGui::EndDisabled();

        ImGui::TreePop();
    }
}

void UIComponents::DrawCAMExample(vtkDearImGuiInjector* injector)
{
    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    if (ImGui::BeginMenu("OCL Operations")) {
        auto& settings = SettingsManager::GetSettings();
        ImGui::MenuItem("Create Primitive", nullptr, &settings.show_primitive_window);
        DrawLoadStlUI(injector);
        ImGui::MenuItem("Add Cutter", nullptr, &settings.show_cutter_window);
        ImGui::MenuItem("Add Operation", nullptr, &settings.show_operation_window);
        if (ImGui::BeginMenu("OCL Benchmark")) {
            static bool verbose = true;
            ImGui::Checkbox("Verbose", &verbose);
            if (ImGui::Button("Run BatchDropCutter")) {
                if (modelManager.cutter && modelManager.surface) {
                    run_batchdropcutter(modelManager, verbose);
                }
                else {
                    spdlog::error("No cutter or surface");
                }
            }
            if (ImGui::Button("Run SurfaceSubdivisionBatchDropCutter")) {
                if (modelManager.cutter && modelManager.surface) {
                    run_SurfaceSubdivisionBatchDropCutter(modelManager, verbose);
                }
                else {
                    spdlog::error("No cutter or surface");
                }
            }
            if (ImGui::Button("Run BatchDropCutter (Bucket Size)")) {
                if (modelManager.cutter && modelManager.surface) {
                    run_BatchDropCutter_WithDifferentBucketSize(modelManager, verbose);
                }
                else {
                    spdlog::error("No cutter or surface");
                }
            }
            if (ImGui::Button("Run AABBTree VS KDTree")) {
                if (modelManager.cutter && modelManager.surface) {
                    run_AABBTree_VS_KDTree(modelManager, verbose);
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

    // 在ImGui主窗口外独立显示的窗口
    DrawCutterUI(injector);
    DrawOperationUI(injector);
    DrawDebugDropCutterWindow(injector);
    DrawPrimitiveUI(injector);
}

void UIComponents::DrawDebugDropCutterWindow(vtkDearImGuiInjector* injector)
{
    auto& actorManager = injector->ActorManager;
    auto& modelManager = injector->ModelManager;

    if (!g_showDebugWindow || g_debugResultPoints.empty()) {
        return;
    }

    ImGui::SetNextWindowSize(ImVec2(400, 420), ImGuiCond_FirstUseEver);

    // 使用固定位置的窗口，靠近屏幕右侧
    ImVec2 viewportSize = ImGui::GetMainViewport()->Size;
    ImVec2 windowPos(viewportSize.x - 420, 100);
    ImGui::SetNextWindowPos(windowPos, ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Debug DropCutter Control",
                     &g_showDebugWindow,
                     ImGuiWindowFlags_AlwaysAutoResize)) {
        // 如果有结果点
        const auto& currentPoint = g_debugResultPoints[g_debugCurrentPointIndex];
        auto* cc = currentPoint.cc.load();

        // 点的基本信息
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                           "Point %d/%zu",
                           g_debugCurrentPointIndex + 1,
                           g_debugResultPoints.size());

        ImGui::Separator();

        // 坐标信息
        ImGui::Text("Position: (%.3f, %.3f, %.3f)", currentPoint.x, currentPoint.y, currentPoint.z);

        // 如果有CC点，显示其详细信息
        if (cc) {
            // CC类型信息，添加颜色以区分不同类型
            double color[3];
            GetClColor(cc->type, color);
            ImGui::TextColored(ImVec4(color[0], color[1], color[2], 1.0f),
                               "CC Type: %s",
                               ocl::CCType2String(cc->type).c_str());

            ImGui::Text("CC Point: (%.3f, %.3f, %.3f)", cc->x, cc->y, cc->z);

            // 距离信息
            ImGui::Text("Triangle Distance: %.6f", currentPoint.z - cc->z);
        }

        ImGui::Separator();

        // 上下箭头按钮控制
        ImGui::BeginGroup();
        ImGui::Text("Navigate Points:");
        ImGui::SameLine();

        bool upPressed = ImGui::ArrowButton("##up", ImGuiDir_Up);
        ImGui::SameLine();
        bool downPressed = ImGui::ArrowButton("##down", ImGuiDir_Down);

        if (upPressed && g_debugCurrentPointIndex > 0) {
            g_debugCurrentPointIndex--;
        }

        if (downPressed
            && g_debugCurrentPointIndex < static_cast<int>(g_debugResultPoints.size() - 1)) {
            g_debugCurrentPointIndex++;
        }
        ImGui::EndGroup();

        // 显示滑块控制
        int tempIndex = g_debugCurrentPointIndex;
        if (ImGui::SliderInt("Point Index", &tempIndex, 0, g_debugResultPoints.size() - 1)) {
            g_debugCurrentPointIndex = tempIndex;
        }

        ImGui::Separator();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.4f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.6f, 0.2f, 1.0f));

        // "Go To Point"按钮 - 移动刀具到当前点
        if (ImGui::Button("Go To This Point")) {
            // 移动刀具到当前点
            actorManager.cutterActor->SetPosition(currentPoint.x, currentPoint.y, currentPoint.z);
        }

        ImGui::PopStyleColor(3);

        if (ImGui::Button("Close")) {
            g_showDebugWindow = false;
        }
    }
    ImGui::End();
}

void UIComponents::DrawPrimitiveUI(vtkDearImGuiInjector* injector)
{
    auto& settings = SettingsManager::GetSettings();
    if (!settings.show_primitive_window) {
        return;
    }

    auto& modelManager = injector->ModelManager;
    auto& actorManager = injector->ActorManager;

    ImGui::Begin("Primitive Geometry",
                 &settings.show_primitive_window,
                 ImGuiWindowFlags_AlwaysAutoResize);

    // 使用静态变量保存选择的几何体类型
    static const char* primitive_types[] =
        {"Cube", "Sphere", "Cylinder", "Cone", "Ellipsoid", "Torus", "CustomTriangles"};
    static int primitive_type = 0;

    // 创建下拉菜单
    ImGui::Combo("Primitive Type", &primitive_type, primitive_types, IM_ARRAYSIZE(primitive_types));

    ImGui::Separator();

    // 通用参数
    static int sampling = 50;
    static bool closed = true;

    // 各个几何体的特定参数
    static float cube_length = 10.0f;
    static float cube_width = 10.0f;
    static float cube_height = 10.0f;

    static float sphere_radius = 5.0f;

    static float cylinder_diameter = 4.0f;
    static float cylinder_height = 10.0f;
    static float cylinder_edge_length = 1.0f;

    static float cone_diameter1 = 4.0f;
    static float cone_diameter2 = 0.0f;
    static float cone_height = 10.0f;
    static float cone_edge_length = 1.0f;

    static float ellipsoid_radius1 = 5.0f;
    static float ellipsoid_radius2 = 3.0f;

    static float torus_major_radius = 5.0f;
    static float torus_minor_radius = 2.0f;

    static std::list<Triangle> custom_triangles {
        Triangle {Point {0, 0, 0}, Point {10, 0, 0}, Point {0, 10, 0}}};

    // 根据选择的基本几何体类型显示对应的参数控件
    switch (primitive_type) {
        case 0:  // Cube
            ImGui::DragFloat("Length", &cube_length, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragFloat("Width", &cube_width, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragFloat("Height", &cube_height, 1.0f, 0.01f, 1e6f, "%.3f");

            if (ImGui::Button("Create Cube")) {
                modelManager.createCube(cube_length, cube_width, cube_height);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            break;

        case 1:  // Sphere
            ImGui::DragFloat("Radius", &sphere_radius, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::Separator();
            ImGui::DragInt("Sampling", &sampling, 1, 1, 1000);

            if (ImGui::Button("Create Sphere")) {
                modelManager.createSphere(sphere_radius, sampling);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            break;

        case 2:  // Cylinder
            ImGui::DragFloat("Diameter", &cylinder_diameter, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragFloat("Height", &cylinder_height, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::Separator();
            ImGui::DragFloat("Edge Length", &cylinder_edge_length, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragInt("Sampling", &sampling, 1, 1, 1000);
            ImGui::Separator();
            ImGui::Checkbox("Closed", &closed);

            if (ImGui::Button("Create Cylinder")) {
                modelManager.createCylinder(cylinder_diameter, cylinder_height, sampling, closed);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            break;

        case 3:  // Cone
            ImGui::DragFloat("Bottom Diameter", &cone_diameter1, 1.0f, 0.0f, 1e6f, "%.3f");
            ImGui::DragFloat("Top Diameter", &cone_diameter2, 1.0f, 0.0f, 1e6f, "%.3f");
            ImGui::Separator();
            ImGui::DragFloat("Height", &cone_height, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragFloat("Edge Length", &cone_edge_length, 0.1f, 0.01f, 10.0f, "%.3f");
            ImGui::Separator();
            ImGui::DragInt("Sampling", &sampling, 1, 1, 1000);
            ImGui::Separator();
            ImGui::Checkbox("Closed", &closed);

            if (ImGui::Button("Create Cone")) {
                modelManager.createCone(cone_diameter1,
                                        cone_diameter2,
                                        cone_height,
                                        cone_edge_length,
                                        sampling,
                                        closed);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            break;

        case 4:  // Ellipsoid
            ImGui::DragFloat("Radius X/Y", &ellipsoid_radius1, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragFloat("Radius Z", &ellipsoid_radius2, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::Separator();
            ImGui::DragInt("Sampling", &sampling, 1, 1, 1000);

            if (ImGui::Button("Create Ellipsoid")) {
                modelManager.createEllipsoid(ellipsoid_radius1, ellipsoid_radius2, sampling);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            break;

        case 5:  // Torus
            ImGui::DragFloat("Major Radius", &torus_major_radius, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::DragFloat("Minor Radius", &torus_minor_radius, 1.0f, 0.01f, 1e6f, "%.3f");
            ImGui::Separator();
            ImGui::DragInt("Sampling", &sampling, 1, 1, 1000);

            if (ImGui::Button("Create Torus")) {
                modelManager.createTorus(torus_major_radius, torus_minor_radius, sampling);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            break;

        case 6:  // CustomTriangles
            // Use table to show custom triangles
            static ImGuiTableFlags flags =
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ContextMenuInBody;
            if (custom_triangles.size() > 10) {
                flags |= ImGuiTableFlags_ScrollY;
            }
            if (ImGui::BeginTable("Triangles", 4, flags)) {
                ImGui::TableSetupColumn("P0");
                ImGui::TableSetupColumn("P1");
                ImGui::TableSetupColumn("P2");
                ImGui::TableSetupColumn("Delete");
                ImGui::TableHeadersRow();

                for (auto it = custom_triangles.begin(); it != custom_triangles.end();) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("(%.3f, %.3f, %.3f)", it->p[0].x, it->p[0].y, it->p[0].z);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("(%.3f, %.3f, %.3f)", it->p[1].x, it->p[1].y, it->p[1].z);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("(%.3f, %.3f, %.3f)", it->p[2].x, it->p[2].y, it->p[2].z);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::PushID(it._Ptr);
                    if (ImGui::SmallButton("Del")) {
                        it = custom_triangles.erase(it);
                    }
                    else {
                        ++it;
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }

            if (ImGui::Button("Add")) {
                // popup a dialog to input the points
                ImGui::OpenPopup("Add Custom Triangle");
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(custom_triangles.empty());
            if (ImGui::Button("Empty")) {
                custom_triangles.clear();
            }
            if (ImGui::Button("Create CustomTriangles")) {
                modelManager.createCustomTriangles(custom_triangles);
                UpdateStlSurfActor(actorManager.modelActor, *modelManager.surface);
                injector->ForceResetCamera();
            }
            ImGui::EndDisabled();
            break;
    }

    if (ImGui::BeginPopupModal("Add Custom Triangle", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        static float p0[3] = {0, 0, 0};
        static float p1[3] = {0, 0, 0};
        static float p2[3] = {0, 0, 0};

        ImGui::DragFloat3("P0", p0, 1.0f, -1e6f, 1e6f, "%.3f");
        ImGui::DragFloat3("P1", p1, 1.0f, -1e6f, 1e6f, "%.3f");
        ImGui::DragFloat3("P2", p2, 1.0f, -1e6f, 1e6f, "%.3f");

        if (ImGui::Button("OK")) {
            custom_triangles.emplace_back(ocl::Point(p0[0], p0[1], p0[2]),
                                          ocl::Point(p1[0], p1[1], p1[2]),
                                          ocl::Point(p2[0], p2[1], p2[2]));
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::End();
}
}  // namespace ocl