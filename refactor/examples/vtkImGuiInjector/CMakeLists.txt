# A mini example to test the ocl library
message(STATUS "Configuring vtkImGuiInjector")

find_package(glad REQUIRED)
find_package(nfd REQUIRED)
find_package(nlohmann_json REQUIRED)

# the imgui should be installed with the glfw3 and opengl3 bindings
# vcpkg install imgui[glfw-binding, opengl3-binding]
find_package(imgui CONFIG REQUIRED)

# 获取 imgui::imgui 目标的包含目录
get_target_property(IMGUI_INCLUDE_DIRS imgui::imgui INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "IMGUI_INCLUDE_DIRS: ${IMGUI_INCLUDE_DIRS}")

find_path(IMGUI_BACKENDS_GLFW_PATH
    NAMES
    imgui_impl_glfw.h
    PATHS
    ${IMGUI_INCLUDE_DIRS}
    DOC "Path to ImGui GLFW backends implementation files"
)

find_path(IMGUI_BACKENDS_OPENGL3_PATH
    NAMES
    imgui_impl_opengl3.h
    PATHS
    ${IMGUI_INCLUDE_DIRS}
    DOC "Path to ImGui OpenGL3 backends implementation files"
)

if(NOT IMGUI_BACKENDS_OPENGL3_PATH)
    message(FATAL_ERROR "ImGui OpenGL3 backends implementation files not found!")
    return()
endif()

if(NOT IMGUI_BACKENDS_GLFW_PATH)
    message(FATAL_ERROR "ImGui GLFW backends implementation files not found!")
    return()
endif()

# VTK
find_package(VTK REQUIRED)

if(NOT VTK_FOUND)
    message(FATAL_ERROR "VTK not found!")
    return()
endif()

add_executable(vtkImGuiInjector
    main.cpp
    OverlayUI.cpp
    vtkDearImGuiInjector.cpp
    vtkUtils.cpp
    vtkCutters.cpp
    oclUtils.cpp
    STLSurfUtils.cpp
)

if(VTK_VERSION VERSION_LESS "9.0.0")
    include(${VTK_USE_FILE})
else()
    # vtk_module_autoinit is needed
    vtk_module_autoinit(
        TARGETS vtkImGuiInjector
        MODULES ${VTK_LIBRARIES}
    )
endif()

message(STATUS "VTK Version: ${VTK_VERSION}")

target_link_libraries(vtkImGuiInjector PRIVATE imgui::imgui ${VTK_LIBRARIES} glad::glad ocl nfd::nfd nlohmann_json::nlohmann_json)