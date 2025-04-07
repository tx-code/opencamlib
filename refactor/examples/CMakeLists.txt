option(OCL_BUILD_VTKINJECTOR "build the vtk injector" ON)

if(OCL_BUILD_VTKINJECTOR)
    add_subdirectory(vtkImGuiInjector)
endif()

option(OCL_BUILD_IMGUIVTK "build the imgui vtk" OFF)

if(OCL_BUILD_IMGUIVTK)
    add_subdirectory(imgui-vtk)
endif()

option(OCL_BUILD_NOGUI "build the nogui" OFF)

if(OCL_BUILD_NOGUI)
    add_subdirectory(nogui)
endif()
