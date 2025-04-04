﻿#pragma once

#include <exception>
#include <iostream>
#include <string>

#include <imgui.h>

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

class vtkCameraOrientationWidget;
class vtkAxesActor;

// RGB Color in range [0.0, 1.0]
#define DEFAULT_BACKGROUND 0.39, 0.39, 0.39
// Alpha value in range [0.0, 1.0] where 1 = opaque
#define DEFAULT_ALPHA 1

class VtkViewerError : public std::runtime_error {
public:
    explicit VtkViewerError(const std::string& message) throw()
        : std::runtime_error(message) {
    }

    ~VtkViewerError() override = default;

public:
    char const* what() const throw() override { return exception::what(); }
};

class VtkViewer {
private:
    static void isCurrentCallbackFn(vtkObject* caller, long unsigned int eventId,
                                    void* clientData, void* callData);

    void processEvents();

private:
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkGenericRenderWindowInteractor> interactor;
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkCameraOrientationWidget> camManipulator;

private:
    unsigned int viewportWidth, viewportHeight;
    unsigned int tex;
    bool firstRender;

public:
    VtkViewer();

    ~VtkViewer();

    enum ActorType {
        AT_Default,
        AT_Model,
        AT_Cutter,
        AT_Operation
    };

    vtkSmartPointer<vtkAxesActor> axesActor;

    vtkSmartPointer<vtkActor> modelActor;
    vtkSmartPointer<vtkActor> cutterActor;
    vtkSmartPointer<vtkActor> operationActor;

private:
    IMGUI_IMPL_API void init();

public:
    IMGUI_IMPL_API void render();

    IMGUI_IMPL_API void render(const ImVec2 size);

    IMGUI_IMPL_API void addActor(const vtkSmartPointer<vtkProp>& prop, ActorType type = AT_Default);

    IMGUI_IMPL_API void removeActor(const vtkSmartPointer<vtkProp>& actor);

    void setViewportSize(const ImVec2 newSize);

public:
    static inline unsigned int NoScrollFlags() {
        return ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    }

public:
    inline void setRenderWindow(
        const vtkSmartPointer<vtkGenericOpenGLRenderWindow>& renderWindow) {
        this->renderWindow = renderWindow;
    }

    inline void setInteractor(
        const vtkSmartPointer<vtkGenericRenderWindowInteractor>& interactor) {
        this->interactor = interactor;
    }

    inline void
    setInteractorStyle(const vtkSmartPointer<vtkInteractorStyleTrackballCamera>
        & interactorStyle) {
        this->interactorStyle = interactorStyle;
    }

    inline void setRenderer(const vtkSmartPointer<vtkRenderer>& renderer) {
        this->renderer = renderer;
    }

public:
    inline vtkSmartPointer<vtkGenericOpenGLRenderWindow>& getRenderWindow() {
        return renderWindow;
    }

    inline vtkSmartPointer<vtkGenericRenderWindowInteractor>& getInteractor() {
        return interactor;
    }

    inline vtkSmartPointer<vtkInteractorStyleTrackballCamera>&
    getInteractorStyle() {
        return interactorStyle;
    }

    inline vtkSmartPointer<vtkRenderer>& getRenderer() { return renderer; }

public:
    inline unsigned int getViewportWidth() const { return viewportWidth; }

    inline unsigned int getViewportHeight() const { return viewportHeight; }

    inline unsigned int getTexture() const { return tex; }
};
