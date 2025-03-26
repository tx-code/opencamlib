// SPDX-FileCopyrightText: Copyright 2023 Jaswant Panchumarti
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkObject.h>
#include <vtkWeakPointer.h>

#include "oclUtils.h"

#if __has_include(<vtkXRenderWindowInteractor.h>)
#define USES_X11 1
#endif
#if __has_include(<vtkWin32RenderWindowInteractor.h>) && WIN32
#define USES_WIN32 1
#undef USES_X11
#endif
#if __has_include(<vtkWebAssemblyRenderWindowInteractor.h>) && EMSCRIPTEN
#define USES_SDL2 1
#undef USES_X11
#undef USES_WIN32
#endif
#if !defined(USES_X11) && !defined(USES_WIN32) && !defined(USES_SDL2)
#warning "Keycodes may not be recognized!"
#endif

class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkCallbackCommand;
class vtkInteractorStyle;

class vtkDearImGuiInjector: public vtkObject
{
public:
    static vtkDearImGuiInjector* New();
    vtkTypeMacro(vtkDearImGuiInjector, vtkObject);

    // rendering and interaction callbacks are installed here.
    void Inject(vtkRenderWindowInteractor* interactor);

    void InstallEventCallback(vtkRenderWindowInteractor* interactor);
    void UninstallEventCallback();

    // Manually reset the camera
    void ForceResetCamera();

    // Observe this event and draw application specific ImGui widgets
    static const unsigned long ImGuiDrawEvent = vtkCommand::UserEvent + 1;
    static const unsigned long ImGuiSetupEvent = vtkCommand::UserEvent + 2;
    static const unsigned long ImGuiTearDownEvent = vtkCommand::UserEvent + 3;
    vtkWeakPointer<vtkRenderWindowInteractor> Interactor;

    // Just for example, we have two managers here.
    // ActorManager is used to store the actors for the model, cutter, legend, and operation
    vtkActorManager ActorManager;
    // ModelManager is used to store the model, cutter, and operation
    CAMModelManager ModelManager;

protected:
    vtkDearImGuiInjector();
    ~vtkDearImGuiInjector() override;

    // pair imgui with vtk
    bool SetUp(vtkRenderWindow* renWin);
    void TearDown(vtkObject* caller, unsigned long eid, void* callData);

    // hooks into vtkRenderWindow
    void BeginDearImGuiOverlay(vtkObject* caller, unsigned long eid, void* callData);
    void RenderDearImGuiOverlay(vtkObject* caller, unsigned long eid, void* callData);

    // Mouse will be set here.
    void UpdateMousePosAndButtons(vtkRenderWindowInteractor* interactor);
    void UpdateMouseCursor(vtkRenderWindow* renWin);

    // Run the event loop.
    void PumpEvents(vtkObject* caller, unsigned long eid, void* callData);

    // route an event through Dear ImGUI.
    // VTK[X,Win32,Cocoa]Interactor ---> DearImGui ---> VTK[...]InteractorStyle
    static void
    InterceptEvent(vtkObject* caller, unsigned long eid, void* clientData, void* callData);

    vtkNew<vtkCallbackCommand> EventInterceptor;
    vtkWeakPointer<vtkInteractorStyle> CurrentIStyle;

    double Time = 0;
    bool MouseJustPressed[3] = {false, false, false};
    bool FinishedSetup = false;
    bool Focused = true;
    bool GrabMouse = false;     // true: pass mouse to vtk, false: imgui accepts mouse
                                // and doesn't give it to VTK (when ui is focused)
    bool GrabKeyboard = false;  // true: pass keys to vtk, false: imgui accepts
                                // keys and doesn't give it to VTK (when ui is focused)

    bool ShowDemo = true;
    bool ShowAppMetrics = false;
    bool ShowAppStyleEditor = false;
    bool ShowAppAbout = false;

private:
    vtkDearImGuiInjector(const vtkDearImGuiInjector&) = delete;
    void operator=(const vtkDearImGuiInjector&) = delete;
};
