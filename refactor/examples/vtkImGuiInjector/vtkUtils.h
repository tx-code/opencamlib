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
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkExtractEdges.h>
#include <vtkFollower.h>
#include <vtkHexahedron.h>
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
#include <vtkOutlineFilter.h>
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
#include <vtkUnstructuredGrid.h>
#include <vtkVectorText.h>
#include <vtkVertex.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkVoxel.h>


#include <cmath>
#include <memory>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

#include "AABBTreeAdaptor.h"

#include "common/kdtree.hpp"
#include "cutters/ballcutter.hpp"
#include "cutters/conecutter.hpp"
#include "cutters/cylcutter.hpp"
#include "geo/bbox.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"

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
void UpdateKDTreeActor(vtkSmartPointer<vtkActor>& actor,
                       const ocl::KDTree<ocl::Triangle>* kdtree,
                       double opacity = 0.3,
                       bool onlyLeafNodes = false);

// 创建一个AABBTree的可视化
void UpdateAABBTreeActor(vtkSmartPointer<vtkActor>& actor,
                         const ocl::AABBTreeAdaptor& aabbTree,
                         double opacity = 0.3,
                         int showLevel = -1);

// 可视化被刀具覆盖的三角形
void UpdateOverlappedTrianglesActor(vtkSmartPointer<vtkActor>& actor,
                                    const std::vector<ocl::Triangle>& triangles,
                                    const double color[3] = red,
                                    double opacity = 0.7);

// 可视化点云
void UpdatePointCloudActor(vtkSmartPointer<vtkActor>& actor,
                           const Eigen::MatrixXd& points,
                           const Eigen::MatrixXd& normals,
                           const double color[3] = red,
                           double opacity = 0.7);

// 可视化纤维
void UpdateFiberActor(vtkSmartPointer<vtkActor>& actor,
                      const std::vector<ocl::Fiber>& fibers,
                      const double lineColor[3] = grey,
                      double opacity = 0.7);
