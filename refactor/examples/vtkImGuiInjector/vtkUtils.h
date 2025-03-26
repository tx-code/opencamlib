#pragma once

#include <vtkActor.h>
#include <vtkArrowSource.h>
#include <vtkAxes.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkColor.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkDoubleArray.h>
#include <vtkFollower.h>
#include <vtkIntArray.h>
#include <vtkLegendBoxActor.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricSuperToroid.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangle.h>
#include <vtkTubeFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVectorText.h>
#include <vtkVertex.h>
#include <vtkVertexGlyphFilter.h>


#include <cmath>
#include <memory>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

#include "cutters/ballcutter.hpp"
#include "cutters/conecutter.hpp"
#include "cutters/cylcutter.hpp"
#include "geo/bbox.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"

struct vtkActorManager
{
    vtkSmartPointer<vtkActor> modelActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkActor> cutterActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkActor> operationActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkLegendBoxActor> legendActor {vtkSmartPointer<vtkLegendBoxActor>::New()};

    vtkSmartPointer<vtkAxesActor> axesActor {vtkSmartPointer<vtkAxesActor>::New()};
};

// Common colors
constexpr double white[3] = {1.0, 1.0, 1.0};
constexpr double black[3] = {0.0, 0.0, 0.0};
constexpr double grey[3] = {127.0 / 255.0, 127.0 / 255.0, 127.0 / 255.0};

constexpr double red[3] = {1.0, 0.0, 0.0};
constexpr double pink[3] = {255.0 / 255.0, 192.0 / 255.0, 203.0 / 255.0};
constexpr double orange[3] = {255.0 / 255.0, 165.0 / 255.0, 0.0 / 255.0};
constexpr double yellow[3] = {1.0, 1.0, 0.0};

constexpr double green[3] = {0.0, 1.0, 0.0};
constexpr double lgreen[3] = {150.0 / 255.0, 255.0 / 255.0, 150.0 / 255.0};
constexpr double grass[3] = {182.0 / 255.0, 248.0 / 255.0, 71.0 / 255.0};

constexpr double blue[3] = {0.0, 0.0, 1.0};
constexpr double lblue[3] = {125.0 / 255.0, 191.0 / 255.0, 255.0 / 255.0};
constexpr double cyan[3] = {0.0, 1.0, 1.0};
constexpr double mag[3] = {153.0 / 255.0, 42.0 / 255.0, 165.0 / 255.0};

// Common actor property setting methods
inline void SetActorColor(vtkActor* actor, const double color[3])
{
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
}

inline void SetActorOpacity(vtkActor* actor, double op = 0.5)
{
    actor->GetProperty()->SetOpacity(op);
}

inline void SetActorWireframe(vtkActor* actor)
{
    actor->GetProperty()->SetRepresentationToWireframe();
}

inline void SetActorSurface(vtkActor* actor)
{
    actor->GetProperty()->SetRepresentationToSurface();
}

inline void SetActorPoints(vtkActor* actor)
{
    actor->GetProperty()->SetRepresentationToPoints();
}

inline void SetActorFlat(vtkActor* actor)
{
    actor->GetProperty()->SetInterpolationToFlat();
}

inline void SetActorGouraud(vtkActor* actor)
{
    actor->GetProperty()->SetInterpolationToGouraud();
}

inline void SetActorPhong(vtkActor* actor)
{
    actor->GetProperty()->SetInterpolationToPhong();
}

// Helper functions to create various VTK actors

// Create a cone actor
inline vtkSmartPointer<vtkActor> CreateCone(double center[3] = nullptr,
                                            double radius = 1.0,
                                            double angle = 45.0,
                                            double height = 0.4,
                                            const double color[3] = red,
                                            int resolution = 60)
{
    double defaultCenter[3] = {-2.0, 0.0, 0.0};
    if (!center)
        center = defaultCenter;

    vtkNew<vtkConeSource> source;
    source->SetResolution(resolution);
    source->SetRadius(radius);
    source->SetHeight(height);

    vtkNew<vtkTransform> transform;
    transform->Translate(center[0], center[1], center[2] - source->GetHeight() / 2);
    transform->RotateY(-90);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(source->GetOutputPort());
    transformFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(transformFilter->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a sphere actor
inline vtkSmartPointer<vtkActor> CreateSphere(double radius = 1.0,
                                              int resolution = 20,
                                              double center[3] = nullptr,
                                              const double color[3] = red)
{
    double defaultCenter[3] = {0.0, 2.0, 0.0};
    if (!center)
        center = defaultCenter;

    vtkNew<vtkSphereSource> source;
    source->SetRadius(radius);
    source->SetCenter(center);
    source->SetThetaResolution(resolution);
    source->SetPhiResolution(resolution);
    source->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(source->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a cube actor
inline vtkSmartPointer<vtkActor>
CreateCube(double center[3] = nullptr, double length = 1.0, const double color[3] = green)
{
    double defaultCenter[3] = {2.0, 2.0, 0.0};
    if (!center)
        center = defaultCenter;

    vtkNew<vtkCubeSource> source;
    source->SetCenter(center);
    source->SetXLength(length);
    source->SetYLength(length);
    source->SetZLength(length);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(source->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a cylinder actor
inline vtkSmartPointer<vtkActor> CreateCylinder(double center[3] = nullptr,
                                                double radius = 0.5,
                                                double height = 2.0,
                                                const double color[3] = cyan,
                                                double rotXYZ[3] = nullptr,
                                                int resolution = 50)
{
    double defaultCenter[3] = {0.0, -2.0, 0.0};
    if (!center)
        center = defaultCenter;

    double defaultRot[3] = {0.0, 0.0, 0.0};
    if (!rotXYZ)
        rotXYZ = defaultRot;

    vtkNew<vtkCylinderSource> source;
    source->SetCenter(0, 0, 0);
    source->SetHeight(height);
    source->SetRadius(radius);
    source->SetResolution(resolution);

    vtkNew<vtkTransform> transform;
    transform->Translate(center[0], center[1], center[2] + height / 2);
    transform->RotateX(rotXYZ[0]);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(source->GetOutputPort());
    transformFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(transformFilter->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a line actor
inline vtkSmartPointer<vtkActor>
CreateLine(double p1[3] = nullptr, double p2[3] = nullptr, const double color[3] = cyan)
{
    double defaultP1[3] = {0.0, 0.0, 0.0};
    double defaultP2[3] = {1.0, 1.0, 1.0};
    if (!p1)
        p1 = defaultP1;
    if (!p2)
        p2 = defaultP2;

    vtkNew<vtkLineSource> source;
    source->SetPoint1(p1);
    source->SetPoint2(p2);
    source->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(source->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

inline vtkSmartPointer<vtkActor>
CreateLine(const ocl::Point& p1, const ocl::Point& p2, const double color[3] = cyan)
{
    double p1_data[3] = {p1.x, p1.y, p1.z};
    double p2_data[3] = {p2.x, p2.y, p2.z};
    return CreateLine(p1_data, p2_data, color);
}

// Create a tube actor (line with thickness)
inline vtkSmartPointer<vtkActor> CreateTube(double p1[3] = nullptr,
                                            double p2[3] = nullptr,
                                            double radius = 0.1,
                                            const double color[3] = cyan)
{
    double defaultP1[3] = {0.0, 0.0, 0.0};
    double defaultP2[3] = {1.0, 1.0, 1.0};
    if (!p1)
        p1 = defaultP1;
    if (!p2)
        p2 = defaultP2;

    vtkNew<vtkPoints> points;
    points->InsertNextPoint(p1);
    points->InsertNextPoint(p2);

    vtkNew<vtkLine> line;
    line->GetPointIds()->SetId(0, 0);
    line->GetPointIds()->SetId(1, 1);

    vtkNew<vtkCellArray> lines;
    lines->InsertNextCell(line->GetNumberOfPoints(), line->GetPointIds()->GetPointer(0));

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    vtkNew<vtkTubeFilter> tubeFilter;
    tubeFilter->SetInputData(polyData);
    tubeFilter->SetRadius(radius);
    tubeFilter->SetNumberOfSides(50);
    tubeFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(tubeFilter->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a circle actor
inline vtkSmartPointer<vtkActor> CreateCircle(double center[3] = nullptr,
                                              double radius = 1.0,
                                              const double color[3] = cyan,
                                              int resolution = 50)
{
    double defaultCenter[3] = {0.0, 0.0, 0.0};
    if (!center)
        center = defaultCenter;

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> lines;

    int id = 0;
    for (int n = 0; n < resolution; n++) {
        vtkNew<vtkLine> line;
        double angle1 = (double(n) / double(resolution)) * 2 * vtkMath::Pi();
        double angle2 = (double(n + 1) / double(resolution)) * 2 * vtkMath::Pi();

        double p1[3] = {center[0] + radius * std::cos(angle1),
                        center[1] + radius * std::sin(angle1),
                        center[2]};

        double p2[3] = {center[0] + radius * std::cos(angle2),
                        center[1] + radius * std::sin(angle2),
                        center[2]};

        points->InsertNextPoint(p1);
        points->InsertNextPoint(p2);

        line->GetPointIds()->SetId(0, id++);
        line->GetPointIds()->SetId(1, id++);

        lines->InsertNextCell(line->GetNumberOfPoints(), line->GetPointIds()->GetPointer(0));
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a point actor
inline vtkSmartPointer<vtkActor> CreatePoint(double center[3] = nullptr,
                                             const double color[3] = red)
{
    double defaultCenter[3] = {0.0, 0.0, 0.0};
    if (!center)
        center = defaultCenter;

    vtkNew<vtkPointSource> source;
    source->SetCenter(center);
    source->SetRadius(0);
    source->SetNumberOfPoints(1);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(source->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create an arrow actor
inline vtkSmartPointer<vtkActor>
CreateArrow(double center[3] = nullptr, const double color[3] = blue, double rotXYZ[3] = nullptr)
{
    double defaultCenter[3] = {0.0, 0.0, 0.0};
    if (!center)
        center = defaultCenter;

    double defaultRot[3] = {0.0, 0.0, 0.0};
    if (!rotXYZ)
        rotXYZ = defaultRot;

    vtkNew<vtkArrowSource> source;

    vtkNew<vtkTransform> transform;
    transform->Translate(center[0], center[1], center[2]);
    transform->RotateX(rotXYZ[0]);
    transform->RotateY(rotXYZ[1]);
    transform->RotateZ(rotXYZ[2]);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(source->GetOutputPort());
    transformFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(transformFilter->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Create a 2D text actor for HUD-like text
inline vtkSmartPointer<vtkTextActor> CreateText(const std::string& text = "text",
                                                int size = 18,
                                                const double color[3] = white,
                                                int pos[2] = nullptr)
{
    int defaultPos[2] = {100, 100};
    if (!pos)
        pos = defaultPos;

    vtkNew<vtkTextActor> actor;
    actor->SetInput(text.c_str());

    auto properties = actor->GetTextProperty();
    properties->SetFontFamilyToArial();
    properties->SetFontSize(size);
    properties->SetColor(color[0], color[1], color[2]);

    actor->SetDisplayPosition(pos[0], pos[1]);

    return actor;
}

// Create a toroid actor
inline vtkSmartPointer<vtkActor> CreateToroid(double r1 = 1.0,
                                              double r2 = 0.25,
                                              double center[3] = nullptr,
                                              double rotXYZ[3] = nullptr,
                                              const double color[3] = red)
{
    double defaultCenter[3] = {0.0, 0.0, 0.0};
    if (!center)
        center = defaultCenter;

    double defaultRot[3] = {0.0, 0.0, 0.0};
    if (!rotXYZ)
        rotXYZ = defaultRot;

    vtkNew<vtkParametricSuperToroid> parfun;
    parfun->SetRingRadius(r1);
    parfun->SetCrossSectionRadius(r2);
    parfun->SetN1(1);
    parfun->SetN2(1);

    vtkNew<vtkParametricFunctionSource> source;
    source->SetParametricFunction(parfun);

    vtkNew<vtkTransform> transform;
    transform->Translate(center[0], center[1], center[2]);
    transform->RotateX(rotXYZ[0]);
    transform->RotateY(rotXYZ[1]);
    transform->RotateZ(rotXYZ[2]);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(source->GetOutputPort());
    transformFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(transformFilter->GetOutput());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    SetActorColor(actor, color);

    return actor;
}

// Color utility functions based on CC type (cutter-contact type)
inline void GetClColor(ocl::CCType ccType, double color[3])
{
    switch (ccType) {
        case ocl::CCType::VERTEX:
            std::copy_n(red, 3, color);
            break;
        case ocl::CCType::VERTEX_CYL:
            std::copy_n(pink, 3, color);
            break;
        case ocl::CCType::FACET:
            std::copy_n(green, 3, color);
            break;
        case ocl::CCType::FACET_TIP:  // conecutter tip-contact
            std::copy_n(lgreen, 3, color);
            break;
        case ocl::CCType::FACET_CYL:  // conecutter cylinder-contact
            std::copy_n(grass, 3, color);
            break;
        case ocl::CCType::EDGE:
            std::copy_n(blue, 3, color);
            break;
        case ocl::CCType::EDGE_HORIZ:
            std::copy_n(orange, 3, color);
            break;
        case ocl::CCType::EDGE_SHAFT:
            std::copy_n(mag, 3, color);
            break;
        case ocl::CCType::EDGE_BALL:
            std::copy_n(lblue, 3, color);
            break;
        case ocl::CCType::EDGE_CYL:
            std::copy_n(lblue, 3, color);
            break;
        case ocl::CCType::EDGE_POS:
            std::copy_n(lblue, 3, color);
            break;
        case ocl::CCType::EDGE_NEG:
            std::copy_n(mag, 3, color);
            break;
        case ocl::CCType::EDGE_HORIZ_CYL:
            std::copy_n(pink, 3, color);
            break;
        case ocl::CCType::EDGE_HORIZ_TOR:
            std::copy_n(orange, 3, color);
            break;
        case ocl::CCType::EDGE_CONE:
            std::copy_n(pink, 3, color);
            break;
        case ocl::CCType::EDGE_CONE_BASE:
            std::copy_n(cyan, 3, color);
            break;
        case ocl::CCType::NONE:
            std::copy_n(white, 3, color);
            break;
        default:
        case ocl::CCType::CCTYPE_ERROR:
            std::copy_n(white, 3, color);
            break;
    }
}

// Get CC color based on CC type for visualizing CC points
inline void GetCcColor(ocl::CCType ccType, double color[3])
{
    switch (ccType) {
        case ocl::CCType::FACET:
            std::copy_n(lblue, 3, color);
            break;
        case ocl::CCType::FACET_TIP:
            std::copy_n(mag, 3, color);
            break;
        case ocl::CCType::FACET_CYL:
            std::copy_n(yellow, 3, color);
            break;
        case ocl::CCType::VERTEX:
            std::copy_n(green, 3, color);
            break;
        case ocl::CCType::EDGE:
            std::copy_n(pink, 3, color);
            break;
        case ocl::CCType::EDGE_HORIZ_CYL:
            std::copy_n(red, 3, color);
            break;
        case ocl::CCType::EDGE_HORIZ_TOR:
            std::copy_n(orange, 3, color);
            break;
        case ocl::CCType::EDGE_POS:
            std::copy_n(lblue, 3, color);
            break;
        case ocl::CCType::EDGE_NEG:
            std::copy_n(mag, 3, color);
            break;
        case ocl::CCType::NONE:
            std::copy_n(white, 3, color);
            break;
        default:
        case ocl::CCType::CCTYPE_ERROR:
            double error_color[3] = {0.0, 0.5, 1.0};
            std::copy_n(error_color, 3, color);
            break;
    }
}

// Draw a cutter at a given position
void UpdateCutterActor(vtkSmartPointer<vtkActor>& actor,
                       const ocl::MillingCutter& cutter,
                       const ocl::Point& p);

// Create an STL Actor from an OCL STLSurf
void UpdateStlSurfActor(vtkSmartPointer<vtkActor>& actor,
                        const ocl::STLSurf& stl,
                        const double color[3] = white);

// Create a vtkLookupTable for CCType coloring
vtkSmartPointer<vtkLookupTable> CreateCCTypeLookupTable(bool forCLPoints = true);

// Draw a point cloud with CCType-based coloring using lookup table
void UpdateCLPointCloudActor(vtkSmartPointer<vtkActor>& pointsActor,
                             vtkSmartPointer<vtkLegendBoxActor>& legendActor,
                             const std::vector<ocl::CLPoint>& clpoints, bool forCLPoints = true);

// 新增能处理多层loops的函数
void UpdateLoopsActor(vtkSmartPointer<vtkActor>& actor,
                      const std::vector<std::vector<std::vector<ocl::Point>>>& all_loops);
