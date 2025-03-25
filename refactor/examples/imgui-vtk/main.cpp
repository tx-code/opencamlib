// Standard Library
#include <iostream>
#include <spdlog/spdlog.h>

// OpenGL Loader
// This can be replaced with another loader, e.g. glad, but
// remember to also change the corresponding initialize call!
#include <GL/gl3w.h> // GL3w, initialized with gl3wInit() below

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

// ImGui + imgui-vtk
#include <boost/math/constants/constants.hpp>
#include <codecvt>
#include <igl/PI.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <locale>
#include <nfd.h>

#include "VtkViewer.h"

// VTK
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkSmartPointer.h>

// File-Specific Includes
#include "ocl_demo.h"

static void glfw_error_callback(int error, const char *description) {
  spdlog::error("Glfw Error %d: %s\n", error, description);
}

std::wstring to_wstring(const char *str) {
  std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
  return converter.from_bytes(str);
}

int main(int argc, char *argv[]) {
  CAM_DataModel camModel;
  hello_ocl();

  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
    return 1;
  }

  // Use GL 3.2 (All Platforms)
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  // Decide GLSL version
#ifdef __APPLE__
  // GLSL 150
  const char *glsl_version = "#version 150";
#else
  // GLSL 130
  const char *glsl_version = "#version 130";
#endif

  // Create window with graphics context
  GLFWwindow *window =
      glfwCreateWindow(1280, 720, "Cam Viewer Example", nullptr, nullptr);
  if (window == nullptr) {
    spdlog::error("Failed to create window!");
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  // Initialize OpenGL loader
  if (gl3wInit() != 0) {
    spdlog::error("Failed to initialize OpenGL loader!");
    return 1;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;   // Enable Docking
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable; // Enable Multi-Viewport /
  // Platform Windows'

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // When viewports are enabled we tweak WindowRounding/WindowBg so platform
  // windows can look identical to regular ones.
  ImGuiStyle &style = ImGui::GetStyle();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    style.WindowRounding = 0.0f;
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
  }

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Initialize VtkViewer objects
  VtkViewer camViewer;

  // Our state
  bool show_demo_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
    // tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
    // your main application.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
    // data to your main application. Generally you may always pass all inputs
    // to dear imgui, and hide them from your application based on those two
    // flags.
    glfwPollEvents();
    // if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
    //   ImGui_ImplGlfw_Sleep(10);
    //   continue;
    // }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // 1. Show the big demo window (Most of the sample code is in
    // ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear
    // ImGui!).
    if (show_demo_window) {
      ImGui::ShowDemoWindow(&show_demo_window);
    }

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair
    // to created a named window.
    {
      ImGui::Checkbox(
          "Demo Window",
          &show_demo_window); // Edit bools storing our window open/close state
      ImGui::ColorEdit3(
          "clear color",
          reinterpret_cast<float *>(
              &clear_color)); // Edit 3 floats representing a color

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

      if (ImGui::CollapsingHeader("One shot example",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("CylCutter + Waterline")) {
          cylCutter_waterline_demo(camViewer);
        }
        if (ImGui::Button("CylCutter + Adaptive Waterline")) {
          cylCutter_adaptiveWaterline_demo(camViewer);
        }
        if (ImGui::Button("ConeCutter + PathDropCutter")) {
          coneCutter_pathDropCutter_demo(camViewer);
        }
      }

      if (ImGui::CollapsingHeader("CAM example",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SeparatorText("Tools");
        if (ImGui::Button("Load STL")) {
          // First, pop up a dialog to ask the user to select the file

          NFD_Init();

          nfdu8char_t *outPath = nullptr;
          nfdu8filteritem_t filters[1] = {{"STL Models", "stl"}};
          nfdopendialogu8args_t args = {0};
          args.filterList = filters;
          args.filterCount = 1;
          auto result = NFD_OpenDialogU8_With(&outPath, &args);

          if (result == NFD_OKAY) {
            camModel.surface = std::make_unique<ocl::STLSurf>();
            ocl::STLReader reader(to_wstring(outPath), *camModel.surface);
            DrawStlSurf(camViewer, *camModel.surface);
            NFD_FreePathU8(outPath);
          } else if (result == NFD_CANCEL) {
            // User canceled the dialog
            spdlog::info("User canceled the dialog");
          } else {
            // Handle other error codes
            spdlog::error("Error: {}", NFD_GetError());
          }

          NFD_Quit();
        }

        static const char *cutter_types[] = {"CylCutter", "BallCutter",
                                             "BullCutter", "ConeCutter"};
        static int cutter_type_index = 0;
        ImGui::Combo("Cutter Types", &cutter_type_index, cutter_types,
                     IM_ARRAYSIZE(cutter_types));

        using namespace boost::math::constants;
        static double diameter = 2.0;
        static double length = 10.0;
        static double angle_in_deg =
            radian<double>() * third_pi<double>(); // cone cutter need this
        static double radius = 0.1;                // bull cutter need this
        if (cutter_type_index == 0) {
          ImGui::InputDouble("Diameter", &diameter);
          ImGui::InputDouble("Length", &length);
        } else if (cutter_type_index == 1) {
          ImGui::InputDouble("Diameter", &diameter);
          ImGui::InputDouble("Length", &length);
        } else if (cutter_type_index == 2) {
          ImGui::InputDouble("Diameter", &diameter);
          ImGui::InputDouble("Length", &length);
          ImGui::InputDouble("Radius", &radius);
        } else if (cutter_type_index == 3) {
          ImGui::InputDouble("Diameter", &diameter);
          ImGui::InputDouble("Length", &length);
          ImGui::InputDouble("Angle", &angle_in_deg);
        }

        if (ImGui::Button("Change Cutter")) {
          // According the cutter_type_index, change the cutter type
          if (cutter_type_index == 0) {
            camModel.cutter =
                std::make_unique<ocl::CylCutter>(diameter, length);
            spdlog::info("CylCutter created: {}", camModel.cutter->str());
          } else if (cutter_type_index == 1) {
            camModel.cutter =
                std::make_unique<ocl::BallCutter>(diameter, length);
            spdlog::info("BallCutter created: {}", camModel.cutter->str());
          } else if (cutter_type_index == 2) {
            camModel.cutter =
                std::make_unique<ocl::BullCutter>(diameter, radius, length);
            spdlog::info("BullCutter created: {}", camModel.cutter->str());
          } else if (cutter_type_index == 3) {
            camModel.cutter = std::make_unique<ocl::ConeCutter>(
                diameter, degree<double>() * angle_in_deg, length);
            spdlog::info("ConeCutter created: {}", camModel.cutter->str());
          }
          DrawCutter(camViewer, *camModel.cutter, ocl::Point(0, 0, 0));
        }

        static const char *op_types[] = {"WaterLine", "AdaptiveWaterLine",
                                         "PathDropCutter",
                                         "AdaptivePathDropCutter"};
        static int op_type_index = 0;
        ImGui::Combo("Operation Types", &op_type_index, op_types,
                     IM_ARRAYSIZE(op_types));

        static double sampling = 0.1;
        static double min_sampling = 0.01; // adaptive waterline need this
        static double lift_step = 0.1;
        static double lift_from = 0.0;
        static double lift_to = 1;

        switch (op_type_index) {
        case 0:
          ImGui::InputDouble("Sampling", &sampling);
          ImGui::InputDouble("Lift Step", &lift_step);
          ImGui::InputDouble("Lift From", &lift_from);
          ImGui::InputDouble("Lift To", &lift_to);
          break;
        case 1:
          ImGui::InputDouble("Sampling", &sampling);
          ImGui::InputDouble("Min Sampling", &min_sampling);
          ImGui::InputDouble("Lift Step", &lift_step);
          ImGui::InputDouble("Lift From", &lift_from);
          ImGui::InputDouble("Lift To", &lift_to);
          break;
        case 2:
          ImGui::InputDouble("Sampling", &sampling);
          break;
        case 3:
          ImGui::InputDouble("Sampling", &sampling);
          ImGui::InputDouble("Min Sampling", &min_sampling);
          break;
        default:
          break;
        }

        if (ImGui::Button("Run Operation")) {
          if (camModel.cutter && camModel.surface) {

            switch (op_type_index) {
            case 0:
              waterline(camModel, lift_to, sampling, &camViewer, lift_step,
                        lift_from);
              break;
            case 1:
              adaptiveWaterline(camModel, lift_to, sampling, min_sampling,
                                &camViewer, lift_step, lift_from);
              break;
            case 2:
              pathDropCutter(camModel, sampling, &camViewer);
              break;
            case 3:
              adaptivePathDropCutter(camModel, sampling, min_sampling,
                                     &camViewer);
              break;
            }
          } else {
            ImGui::OpenPopup("No Cutter or Surface");

            // Always center this window when appearing
            auto center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing,
                                    ImVec2(0.5f, 0.5f));
          }
        }

        if (ImGui::BeginPopupModal("No Cutter or Surface", nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize)) {
          ImGui::Text("Please select a cutter and a surface");
          ImGui::SetItemDefaultFocus();
          if (ImGui::Button("OK", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
          }
          ImGui::EndPopup();
        }
      }

      ImGui::SeparatorText("Data Model");
      bool axesVisible = camViewer.axesActor->GetVisibility();
      if (ImGui::Checkbox("Show Axes", &axesVisible)) {
        camViewer.axesActor->SetVisibility(axesVisible);
      }

      if (ImGui::TreeNodeEx("WorkPieces", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (auto model = camViewer.modelActor) {
          ImGui::Text(model->GetObjectName().c_str());

          // Bbox
          auto bbox = model->GetBounds();
          ImGui::Text("Bbox: Min(%.2f, %.2f, %.2f), Max(%.2f, %.2f, %.2f)",
                      bbox[0], bbox[2], bbox[4], bbox[1], bbox[3], bbox[5]);

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
          if (std::find(touched.begin(), touched.end(), true) !=
              touched.end()) {
            model->GetProperty()->SetRepresentation(representation);
          }

          // Transparency
          double trans = 1 - model->GetProperty()->GetOpacity();
          static double _min = 0;
          static double _max = 1;
          if (ImGui::SliderScalar("Transparency", ImGuiDataType_Double, &trans,
                                  &_min, &_max)) {
            model->GetProperty()->SetOpacity(1 - trans);
          }
        }

        ImGui::TreePop();
      }

      if (ImGui::TreeNodeEx("Cutters", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (auto cutter = camViewer.cutterActor) {
          ImGui::Text(cutter->GetObjectName().c_str());
          bool visible = cutter->GetVisibility();
          ImGui::Checkbox("Show Cutter", &visible);
          cutter->SetVisibility(visible);

          int representation = cutter->GetProperty()->GetRepresentation();
          std::array<bool, 2> touched;
          touched[0] = ImGui::RadioButton("Wireframe", &representation, 1);
          ImGui::SameLine();
          touched[1] = ImGui::RadioButton("Surface", &representation, 2);
          if (std::find(touched.begin(), touched.end(), true) !=
              touched.end()) {
            cutter->GetProperty()->SetRepresentation(representation);
          }
        }
        ImGui::TreePop();
      }

      if (ImGui::TreeNode("Operations")) {
        ImGui::TreePop();
      }
    }

    // 4. Show a simple VtkViewer Instance (Always Open)
    ImGui::SetNextWindowSize(ImVec2(360, 240), ImGuiCond_FirstUseEver);
    ImGui::Begin("Cam Viewer", nullptr, VtkViewer::NoScrollFlags());
    camViewer.render(); // default render size = ImGui::GetContentRegionAvail()
    ImGui::End();

    ImGui::Render();

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                 clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Update and Render additional Platform Windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow *backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
