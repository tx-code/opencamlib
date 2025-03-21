option(OCL_BUILD_VTKINJECTOR "build the vtk injector" OFF)

if(OCL_BUILD_VTKINJECTOR)
    add_subdirectory(vtkImGuiInjector)
endif()

option(OCL_BUILD_IMGUIVTK "build the imgui vtk" ON)

if(OCL_BUILD_IMGUIVTK)
    add_subdirectory(imgui-vtk)
endif()
