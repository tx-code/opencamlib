﻿#pragma once

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkImplicitPlaneRepresentation.h>
#include <vtkImplicitPlaneWidget2.h>
#include <vtkLegendBoxActor.h>
#include <vtkLineWidget2.h>
#include <vtkSmartPointer.h>


struct vtkActorManager
{
    vtkSmartPointer<vtkActor> modelActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkActor> cutterActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkActor> operationActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkLegendBoxActor> legendActor {vtkSmartPointer<vtkLegendBoxActor>::New()};

    // Helps
    vtkSmartPointer<vtkActor> treeActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkAxesActor> axesActor {vtkSmartPointer<vtkAxesActor>::New()};
    // For Debug and Test
    vtkSmartPointer<vtkActor> debugActor {vtkSmartPointer<vtkActor>::New()};

    // Widgets
    // For singleWaterline
    vtkSmartPointer<vtkImplicitPlaneWidget2> planeWidget {
        vtkSmartPointer<vtkImplicitPlaneWidget2>::New()};
    // For single fiberPushCutter
    vtkSmartPointer<vtkLineWidget2> lineWidget {vtkSmartPointer<vtkLineWidget2>::New()};
};
