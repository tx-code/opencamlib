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
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "VtkViewer.h"

// VTK
#include <vtkActor.h>
#include <vtkSmartPointer.h>

// File-Specific Includes
#include "ocl_demo.h"

static void glfw_error_callback(int error, const char* description) {
    spdlog::error("Glfw Error %d: %s\n", error, description);
}

int main(int argc, char* argv[]) {
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
    const char* glsl_version = "#version 130";
#endif

    // Create window with graphics context
    GLFWwindow* window =
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
    ImGuiIO& io = ImGui::GetIO();
    (void) io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable; // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable; // Enable Multi-Viewport /
    // Platform Windows'

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform
    // windows can look identical to regular ones.
    ImGuiStyle& style = ImGui::GetStyle();
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
                reinterpret_cast<float *>(&clear_color)); // Edit 3 floats representing a color

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
            1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            if (ImGui::CollapsingHeader("One shot example",ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("CylCutter + Waterline")) {
                    cylCutter_waterline_demo(camViewer);
                }
                if (ImGui::Button("ConeCutter + PathDropCutter")) {
                    coneCutter_pathDropCutter_demo(camViewer);
                }
            }

            if (ImGui::TreeNodeEx("WorkPieces", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (auto model = camViewer.modelActor) {
                    ImGui::Text(model->GetObjectName().c_str());
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

                ImGui::TreePop();
            }

            if (ImGui::TreeNode("Cutters")) {
                // TODO Cutters related controls
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
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
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
