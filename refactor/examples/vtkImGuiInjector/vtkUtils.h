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
#include <vtkNamedColors.h>
#include <vtkNew.h>
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
#include <vtkUnstructuredGrid.h>
#include <vtkHexahedron.h>
#include <vtkDataSetMapper.h>
#include <vtkOutlineFilter.h>
#include <vtkExtractEdges.h>
#include <vtkVoxel.h>

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
#include "common/kdtree.hpp"

struct vtkActorManager
{
    vtkSmartPointer<vtkActor> modelActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkActor> cutterActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkActor> operationActor {vtkSmartPointer<vtkActor>::New()};
    vtkSmartPointer<vtkLegendBoxActor> legendActor {vtkSmartPointer<vtkLegendBoxActor>::New()};

    vtkSmartPointer<vtkActor> kdtreeActor {vtkSmartPointer<vtkActor>::New()};
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

// Color utility functions based on CC type (cutter-contact type)
inline void GetClColor(ocl::CCType ccType, double color[3])
{
    switch (ccType) {
        case ocl::CCType::NONE:
            // 白色
            color[0] = 1.0;
            color[1] = 1.0;
            color[2] = 1.0;
            break;
        case ocl::CCType::VERTEX:
            // 亮绿色
            color[0] = 0.0;
            color[1] = 1.0;
            color[2] = 0.0;
            break;
        case ocl::CCType::VERTEX_CYL:
            // 深蓝色
            color[0] = 0.0;
            color[1] = 0.0;
            color[2] = 0.7;
            break;
        case ocl::CCType::EDGE:
            // 深粉色
            color[0] = 1.0;
            color[1] = 0.08;
            color[2] = 0.58;
            break;
        case ocl::CCType::EDGE_HORIZ:
            // 深绿松石色
            color[0] = 0.0;
            color[1] = 0.81;
            color[2] = 0.82;
            break;
        case ocl::CCType::EDGE_SHAFT:
            // 橙红色
            color[0] = 1.0;
            color[1] = 0.27;
            color[2] = 0.0;
            break;
        case ocl::CCType::EDGE_HORIZ_CYL:
            // 红色
            color[0] = 1.0;
            color[1] = 0.0;
            color[2] = 0.0;
            break;
        case ocl::CCType::EDGE_HORIZ_TOR:
            // 橙色
            color[0] = 1.0;
            color[1] = 0.65;
            color[2] = 0.0;
            break;
        case ocl::CCType::EDGE_BALL:
            // 深天蓝
            color[0] = 0.0;
            color[1] = 0.75;
            color[2] = 1.0;
            break;
        case ocl::CCType::EDGE_POS:
            // 春绿色
            color[0] = 0.0;
            color[1] = 1.0;
            color[2] = 0.5;
            break;
        case ocl::CCType::EDGE_NEG:
            // 紫色
            color[0] = 0.5;
            color[1] = 0.0;
            color[2] = 0.5;
            break;
        case ocl::CCType::EDGE_CYL:
            // 石板蓝
            color[0] = 0.42;
            color[1] = 0.35;
            color[2] = 0.8;
            break;
        case ocl::CCType::EDGE_CONE:
            // 中兰花紫
            color[0] = 0.73;
            color[1] = 0.33;
            color[2] = 0.83;
            break;
        case ocl::CCType::EDGE_CONE_BASE:
            // 青色
            color[0] = 0.0;
            color[1] = 1.0;
            color[2] = 1.0;
            break;
        case ocl::CCType::FACET:
            // 银灰色
            color[0] = 0.75;
            color[1] = 0.75;
            color[2] = 0.75;
            break;
        case ocl::CCType::FACET_TIP:  // conecutter tip-contact
            // 洋红色
            color[0] = 1.0;
            color[1] = 0.0;
            color[2] = 1.0;
            break;
        case ocl::CCType::FACET_CYL:  // conecutter cylinder-contact
            // 金黄色
            color[0] = 1.0;
            color[1] = 0.84;
            color[2] = 0.0;
            break;
        default:
        case ocl::CCType::CCTYPE_ERROR:
            // 暗灰色
            color[0] = 0.33;
            color[1] = 0.33;
            color[2] = 0.33;
            break;
    }
}

// Get CC color based on CC type for visualizing CC points
inline void GetCcColor(ocl::CCType ccType, double color[3])
{
    switch (ccType) {
        case ocl::CCType::NONE:
            // 白色
            color[0] = 1.0;
            color[1] = 1.0;
            color[2] = 1.0;
            break;
        case ocl::CCType::VERTEX:
            // 酸橙绿
            color[0] = 0.2;
            color[1] = 0.8;
            color[2] = 0.2;
            break;
        case ocl::CCType::VERTEX_CYL:
            // 深青色
            color[0] = 0.0;
            color[1] = 0.4;
            color[2] = 0.6;
            break;
        case ocl::CCType::EDGE:
            // 热粉红
            color[0] = 1.0;
            color[1] = 0.41;
            color[2] = 0.71;
            break;
        case ocl::CCType::EDGE_HORIZ:
            // 海蓝色
            color[0] = 0.13;
            color[1] = 0.7;
            color[2] = 0.67;
            break;
        case ocl::CCType::EDGE_SHAFT:
            // 棕色
            color[0] = 0.65;
            color[1] = 0.16;
            color[2] = 0.16;
            break;
        case ocl::CCType::EDGE_HORIZ_CYL:
            // 深红色
            color[0] = 0.86;
            color[1] = 0.08;
            color[2] = 0.24;
            break;
        case ocl::CCType::EDGE_HORIZ_TOR:
            // 珊瑚色
            color[0] = 1.0;
            color[1] = 0.5;
            color[2] = 0.31;
            break;
        case ocl::CCType::EDGE_BALL:
            // 宝蓝色
            color[0] = 0.0;
            color[1] = 0.5;
            color[2] = 0.8;
            break;
        case ocl::CCType::EDGE_POS:
            // 矢车菊蓝
            color[0] = 0.39;
            color[1] = 0.58;
            color[2] = 0.93;
            break;
        case ocl::CCType::EDGE_NEG:
            // 深兰花紫
            color[0] = 0.6;
            color[1] = 0.2;
            color[2] = 0.8;
            break;
        case ocl::CCType::EDGE_CYL:
            // 锰紫色
            color[0] = 0.33;
            color[1] = 0.0;
            color[2] = 0.55;
            break;
        case ocl::CCType::EDGE_CONE:
            // 深绿色
            color[0] = 0.0;
            color[1] = 0.5;
            color[2] = 0.0;
            break;
        case ocl::CCType::EDGE_CONE_BASE:
            // 浅蓝绿色
            color[0] = 0.0;
            color[1] = 0.8;
            color[2] = 0.8;
            break;
        case ocl::CCType::FACET:
            // 皇家蓝
            color[0] = 0.25;
            color[1] = 0.41;
            color[2] = 0.88;
            break;
        case ocl::CCType::FACET_TIP:
            // 深洋红
            color[0] = 0.55;
            color[1] = 0.0;
            color[2] = 0.55;
            break;
        case ocl::CCType::FACET_CYL:
            // 黄色
            color[0] = 1.0;
            color[1] = 1.0;
            color[2] = 0.0;
            break;
        default:
        case ocl::CCType::CCTYPE_ERROR:
            // 黑色
            color[0] = 0.0;
            color[1] = 0.0;
            color[2] = 0.0;
            break;
    }
}

// Create an STL Actor from an OCL STLSurf
void UpdateStlSurfActor(vtkSmartPointer<vtkActor>& actor,
                        const ocl::STLSurf& stl,
                        const double color[3] = white);

// Create a vtkLookupTable for CCType coloring
vtkSmartPointer<vtkLookupTable> CreateCCTypeLookupTable(bool forCLPoints = true);

// Draw a point cloud with CCType-based coloring using lookup table
void UpdateCLPointCloudActor(vtkSmartPointer<vtkActor>& pointsActor,
                             vtkSmartPointer<vtkLegendBoxActor>& legendActor,
                             const std::vector<ocl::CLPoint>& clpoints,
                             bool forCLPoints = true);

// 新增能处理多层loops的函数
void UpdateLoopsActor(vtkSmartPointer<vtkActor>& actor,
                      const std::vector<std::vector<std::vector<ocl::Point>>>& all_loops);

// 创建一个KDTree的可视化
template <class BBObj>
void UpdateKDTreeActor(vtkSmartPointer<vtkActor>& actor,
                       const ocl::KDTree<BBObj>* kdtree, 
                       double opacity = 0.3,
                       bool onlyLeafNodes = false) {
    if (!kdtree || !kdtree->getRoot()) {
        spdlog::error("KDTree is null or has no root node");
        return;
    }

    vtkNew<vtkUnstructuredGrid> grid;
    vtkNew<vtkPoints> points;
    
    if (onlyLeafNodes) {
        // 递归函数来找到叶子节点并创建可视化
        std::function<void(ocl::KDNode<BBObj>*)> findLeafNodes = 
            [&](ocl::KDNode<BBObj>* node) {
                if (!node) return;
                
                // 如果是叶子节点
                if (node->isLeaf && node->tris && !node->tris->empty()) {
                    // 计算叶子节点的包围盒
                    ocl::Bbox bbox;
                    bool first = true;
                    
                    for (const auto& obj : *(node->tris)) {
                        if (first) {
                            bbox = obj.bb;
                            first = false;
                        } else {
                            bbox.addTriangle(obj);
                        }
                    }
                    
                    // 创建一个vtkVoxel来表示此叶子节点的包围盒
                    vtkNew<vtkVoxel> voxel;
                    
                    // 定义八个顶点
                    // bbox[0] = minx, bbox[1] = maxx, bbox[2] = miny,
                    // bbox[3] = maxy, bbox[4] = minz, bbox[5] = maxz
                    const vtkIdType pointIds[8] = {
                        points->InsertNextPoint(bbox[0], bbox[2], bbox[4]),  // minx, miny, minz
                        points->InsertNextPoint(bbox[1], bbox[2], bbox[4]),  // maxx, miny, minz
                        points->InsertNextPoint(bbox[0], bbox[3], bbox[4]),  // minx, maxy, minz
                        points->InsertNextPoint(bbox[1], bbox[3], bbox[4]),  // maxx, maxy, minz
                        points->InsertNextPoint(bbox[0], bbox[2], bbox[5]),  // minx, miny, maxz
                        points->InsertNextPoint(bbox[1], bbox[2], bbox[5]),  // maxx, miny, maxz
                        points->InsertNextPoint(bbox[0], bbox[3], bbox[5]),  // minx, maxy, maxz
                        points->InsertNextPoint(bbox[1], bbox[3], bbox[5])   // maxx, maxy, maxz
                    };
                    
                    // 设置voxel的顶点
                    for (int i = 0; i < 8; ++i) {
                        voxel->GetPointIds()->SetId(i, pointIds[i]);
                    }
                    
                    // 将voxel插入到网格中
                    grid->InsertNextCell(voxel->GetCellType(), voxel->GetPointIds());
                } else {
                    // 如果不是叶子节点，继续递归
                    if (node->hi) {
                        findLeafNodes(node->hi);
                    }
                    if (node->lo) {
                        findLeafNodes(node->lo);
                    }
                }
            };
        
        // 从根节点开始查找所有叶子节点
        findLeafNodes(kdtree->getRoot());
    } else {
        // 递归函数来构建KDTree的可视化网格
        std::function<void(ocl::KDNode<BBObj>*, int)> buildGridFromNode = 
            [&](ocl::KDNode<BBObj>* node, int depth) {
                if (!node) return;
                
                // 获取节点的包围盒
                ocl::Bbox bbox;
                
                // 如果是叶子节点，从三角形构建包围盒
                if (node->isLeaf && node->tris) {
                    bool first = true;
                    for (const auto& obj : *(node->tris)) {
                        if (first) {
                            bbox = obj.bb;
                            first = false;
                        } else {
                            bbox.addTriangle(obj);
                        }
                    }
                } 
                // 否则基于子节点构建包围盒
                else {
                    if (node->hi) {
                        buildGridFromNode(node->hi, depth + 1);
                    }
                    if (node->lo) {
                        buildGridFromNode(node->lo, depth + 1);
                    }
                    
                    // 对于非叶子节点，我们根据切分维度创建包围盒
                    if (node->hi || node->lo) {
                        // 这部分需要根据KDTree的实际实现调整
                        // 以下是示例，可能需要根据实际情况修改
                        double xmin = -1000, xmax = 1000;
                        double ymin = -1000, ymax = 1000;
                        double zmin = -1000, zmax = 1000;
                        
                        // 根据切分维度调整bbox
                        if (node->dim == 0) xmax = node->cutval;      // X min
                        else if (node->dim == 1) xmin = node->cutval; // X max
                        else if (node->dim == 2) ymax = node->cutval; // Y min
                        else if (node->dim == 3) ymin = node->cutval; // Y max
                        else if (node->dim == 4) zmax = node->cutval; // Z min
                        else if (node->dim == 5) zmin = node->cutval; // Z max
                        
                        bbox = ocl::Bbox(xmin, xmax, ymin, ymax, zmin, zmax);
                    }
                }
                
                // 创建一个vtkVoxel来表示此节点的包围盒
                vtkNew<vtkVoxel> voxel;
                
                // 定义八个顶点
                // bbox[0] = minx, bbox[1] = maxx, bbox[2] = miny,
                // bbox[3] = maxy, bbox[4] = minz, bbox[5] = maxz
                const vtkIdType pointIds[8] = {
                    points->InsertNextPoint(bbox[0], bbox[2], bbox[4]),  // minx, miny, minz
                    points->InsertNextPoint(bbox[1], bbox[2], bbox[4]),  // maxx, miny, minz
                    points->InsertNextPoint(bbox[0], bbox[3], bbox[4]),  // minx, maxy, minz
                    points->InsertNextPoint(bbox[1], bbox[3], bbox[4]),  // maxx, maxy, minz
                    points->InsertNextPoint(bbox[0], bbox[2], bbox[5]),  // minx, miny, maxz
                    points->InsertNextPoint(bbox[1], bbox[2], bbox[5]),  // maxx, miny, maxz
                    points->InsertNextPoint(bbox[0], bbox[3], bbox[5]),  // minx, maxy, maxz
                    points->InsertNextPoint(bbox[1], bbox[3], bbox[5])   // maxx, maxy, maxz
                };
                
                // 设置voxel的顶点
                for (int i = 0; i < 8; ++i) {
                    voxel->GetPointIds()->SetId(i, pointIds[i]);
                }
                
                // 将voxel插入到网格中
                grid->InsertNextCell(voxel->GetCellType(), voxel->GetPointIds());
            };
        
        // 从根节点开始构建网格
        buildGridFromNode(kdtree->getRoot(), 0);
    }
    
    // 设置网格的点
    grid->SetPoints(points);
    
    // 创建一个mapper
    vtkNew<vtkDataSetMapper> mapper;
    mapper->SetInputData(grid);
    
    // 更新actor
    actor->SetMapper(mapper);
    
    // 设置颜色和透明度
    SetActorColor(actor, blue);
    SetActorOpacity(actor, opacity);
    
    // 设置为线框显示模式
    SetActorWireframe(actor);
}
