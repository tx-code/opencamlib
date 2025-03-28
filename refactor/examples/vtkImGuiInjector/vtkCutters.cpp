#include "vtkCutters.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <spdlog/spdlog.h>
#include <vtkAppendPolyData.h>
#include <vtkConeSource.h>
#include <vtkCylinderSource.h>
#include <vtkDiskSource.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricTorus.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>


// 定义颜色常量
constexpr double red[3] = {1.0, 0.0, 0.0};
constexpr double green[3] = {0.0, 1.0, 0.0};
constexpr double blue[3] = {0.0, 0.0, 1.0};
constexpr double yellow[3] = {1.0, 1.0, 0.0};

// 辅助函数
inline void SetActorColor(vtkActor* actor, const double color[3])
{
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
}

inline void SetActorWireframe(vtkActor* actor)
{
    actor->GetProperty()->SetRepresentationToWireframe();
}

// 绘制圆柱铣刀(Cylindrical Cutter)
// 圆柱铣刀的CL位置在圆柱Cap的中心
void UpdateCylCutter(vtkSmartPointer<vtkActor>& actor,
                     const ocl::CylCutter& cutter,
                     const ocl::Point& p)
{
    assert(actor);
    const double r = cutter.getRadius();
    const double flutesLength = cutter.getLength();

    // 创建圆柱体
    vtkNew<vtkCylinderSource> cylinderSource;
    cylinderSource->SetCenter(p.x, p.y - flutesLength / 2.0, p.z);
    cylinderSource->SetHeight(flutesLength);
    cylinderSource->SetRadius(r);
    cylinderSource->SetResolution(30);
    cylinderSource->CappingOn();

    // Use Transform to rotate the cylinder
    vtkNew<vtkTransform> transform;
    transform->RotateX(-90);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetInputConnection(cylinderSource->GetOutputPort());
    transformFilter->SetTransform(transform);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(transformFilter->GetOutputPort());
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(red[0], red[1], red[2]);
    actor->SetObjectName(cutter.str());
    SetActorWireframe(actor);
}

// 绘制球头铣刀(Ball Cutter)
// 球头铣刀的CL位置在半球顶部
void UpdateBallCutter(vtkSmartPointer<vtkActor>& actor,
                      const ocl::BallCutter& cutter,
                      const ocl::Point& p)
{
    assert(actor);
    double r = cutter.getRadius();
    double flutesLength = cutter.getLength();
    double shaftLength = flutesLength - r;

    // 创建圆柱部分（刀杆）
    vtkNew<vtkCylinderSource> cylinderSource;
    cylinderSource->SetCenter(p.x, p.y - shaftLength / 2.0 - r, p.z);
    cylinderSource->SetHeight(shaftLength);
    cylinderSource->SetRadius(r);
    cylinderSource->SetResolution(30);
    cylinderSource->CappingOn();

    // 创建球体部分
    vtkNew<vtkSphereSource> sphereSource;
    sphereSource->SetCenter(p.x, p.y - r, p.z);
    sphereSource->SetRadius(r);
    sphereSource->SetPhiResolution(30);
    sphereSource->SetThetaResolution(30);
    // 半球
    sphereSource->SetStartTheta(0);
    sphereSource->SetEndTheta(180);

    // 合并圆柱和球体
    vtkNew<vtkAppendPolyData> appendFilter;
    appendFilter->AddInputConnection(cylinderSource->GetOutputPort());
    appendFilter->AddInputConnection(sphereSource->GetOutputPort());
    appendFilter->Update();

    // Use Transform to rotate the cylinder
    vtkNew<vtkTransform> transform;
    transform->RotateX(-90);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetInputConnection(appendFilter->GetOutputPort());
    transformFilter->SetTransform(transform);

    // 创建映射器
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(yellow[0], yellow[1], yellow[2]);
    actor->SetObjectName(cutter.str());
    SetActorWireframe(actor);
}

// 绘制牛头铣刀(Bull Cutter)
// 如果我们需要支持更多类型的刀具，可以统一使用vtkRotationExtrusionFilter来绘制 (2D草图 -> 3D实体)
void UpdateBullCutter(vtkSmartPointer<vtkActor>& actor,
                      const ocl::BullCutter& cutter,
                      const ocl::Point& p)
{
    assert(actor);
    /// 圆柱表示刀身，圆环表示刀头
    // https://en.wikipedia.org/wiki/Torus
    double r = cutter.getRadius();
    double minorRadius = cutter.getRadius2();
    double majorRadius = r - minorRadius;
    double flutesLength = cutter.getLength();

    // 创建圆柱部分
    vtkNew<vtkCylinderSource> cylinderSource;
    cylinderSource->SetHeight(flutesLength - minorRadius);
    cylinderSource->SetRadius(r);
    cylinderSource->SetResolution(30);
    cylinderSource->CappingOn();

    // 移动圆柱到正确位置
    vtkNew<vtkTransform> cylinderTransform;
    cylinderTransform->Translate(p.x, p.y, p.z + minorRadius + (flutesLength - minorRadius) / 2.0);
    cylinderTransform->RotateX(90);  // 纵轴旋转为Z轴

    vtkNew<vtkTransformPolyDataFilter> cylinderTransformFilter;
    cylinderTransformFilter->SetInputConnection(cylinderSource->GetOutputPort());
    cylinderTransformFilter->SetTransform(cylinderTransform);

    // 创建底部圆环（使用参数曲面来表示环面）
    vtkNew<vtkParametricTorus> torus;
    torus->SetRingRadius(majorRadius);
    torus->SetCrossSectionRadius(minorRadius);
    torus->SetMinimumV(boost::math::constants::pi<double>()); // just half torus

    vtkNew<vtkParametricFunctionSource> toroidFunctionSource;
    toroidFunctionSource->SetParametricFunction(torus);
    toroidFunctionSource->SetUResolution(30);
    toroidFunctionSource->SetVResolution(30);
    toroidFunctionSource->SetWResolution(30);
    toroidFunctionSource->Update();

    // 移动圆环到正确位置
    vtkNew<vtkTransform> toroidTransform;
    toroidTransform->Translate(p.x, p.y, p.z + minorRadius);

    vtkNew<vtkTransformPolyDataFilter> toroidTransformFilter;
    toroidTransformFilter->SetInputConnection(toroidFunctionSource->GetOutputPort());
    toroidTransformFilter->SetTransform(toroidTransform);

    // Use a disk to represent the bottom circle
    vtkNew<vtkDiskSource> diskSource;
    diskSource->SetOuterRadius(majorRadius);
    diskSource->SetInnerRadius(0);
    diskSource->SetCircumferentialResolution(30);


    // move disk to correct position
    vtkNew<vtkTransform> diskTransform;
    diskTransform->Translate(p.x, p.y, p.z);

    vtkNew<vtkTransformPolyDataFilter> diskTransformFilter;
    diskTransformFilter->SetInputConnection(diskSource->GetOutputPort());
    diskTransformFilter->SetTransform(diskTransform);

    // 合并圆柱和圆环
    vtkNew<vtkAppendPolyData> appendFilter;
    appendFilter->AddInputConnection(cylinderTransformFilter->GetOutputPort());
    appendFilter->AddInputConnection(toroidTransformFilter->GetOutputPort());
    appendFilter->AddInputConnection(diskTransformFilter->GetOutputPort());
    appendFilter->Update();

    // 创建映射器
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(appendFilter->GetOutput());

    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(green[0], green[1], green[2]);
    actor->SetObjectName(cutter.str());
    SetActorWireframe(actor);
}

// 绘制锥形铣刀(Cone Cutter)
// 对于锥形铣刀，圆锥顶点即为CL位置
void UpdateConeCutter(vtkSmartPointer<vtkActor>& actor,
                      const ocl::ConeCutter& cutter,
                      const ocl::Point& p)
{
    assert(actor);
    double r = cutter.getRadius();
    double angle = cutter.getAngle();
    double coneHeight = r / std::tan(angle);
    double fullLength = cutter.getLength();
    double shaftLength = fullLength - coneHeight;

    // 创建圆柱部分（刀杆）
    vtkNew<vtkCylinderSource> cylinderSource;
    cylinderSource->SetCenter(p.x, p.y - shaftLength / 2.0 - coneHeight, p.z);
    cylinderSource->SetHeight(shaftLength);
    cylinderSource->SetRadius(r);
    cylinderSource->SetResolution(30);
    cylinderSource->CappingOn();

    // 创建锥体部分
    vtkNew<vtkConeSource> coneSource;
    coneSource->SetCenter(p.x, p.y - coneHeight / 2.0, p.z);
    coneSource->SetHeight(coneHeight);
    coneSource->SetRadius(r);
    coneSource->SetResolution(30);
    // 先让圆锥轴线跟上面的圆柱轴线重合（Y轴），之后统一绕X轴旋转-90度
    coneSource->SetDirection(0, 1, 0);
    coneSource->CappingOn();

    // 合并圆柱和锥体
    vtkNew<vtkAppendPolyData> appendFilter;
    appendFilter->AddInputConnection(cylinderSource->GetOutputPort());
    appendFilter->AddInputConnection(coneSource->GetOutputPort());
    appendFilter->Update();

    // Use Transform to rotate the cylinder
    vtkNew<vtkTransform> transform;
    transform->RotateX(-90);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetInputConnection(appendFilter->GetOutputPort());
    transformFilter->SetTransform(transform);

    // 创建映射器
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(blue[0], blue[1], blue[2]);
    actor->SetObjectName(cutter.str());
    SetActorWireframe(actor);
}

// 通用绘制铣刀函数，根据铣刀类型动态选择绘制方法
void UpdateCutterActor(vtkSmartPointer<vtkActor>& actor,
                       const ocl::MillingCutter& cutter,
                       const ocl::Point& p)
{
    // 使用dynamic_cast和typeid判断铣刀类型
    if (const auto* cylCutter = dynamic_cast<const ocl::CylCutter*>(&cutter)) {
        // 绘制圆柱铣刀
        UpdateCylCutter(actor, *cylCutter, p);
    }
    else if (const auto* ballCutter = dynamic_cast<const ocl::BallCutter*>(&cutter)) {
        // 绘制球头铣刀
        UpdateBallCutter(actor, *ballCutter, p);
    }
    else if (const auto* bullCutter = dynamic_cast<const ocl::BullCutter*>(&cutter)) {
        // 绘制牛头铣刀
        UpdateBullCutter(actor, *bullCutter, p);
    }
    else if (const auto* coneCutter = dynamic_cast<const ocl::ConeCutter*>(&cutter)) {
        // 绘制锥形铣刀
        UpdateConeCutter(actor, *coneCutter, p);
    }
    else {
        // 未知铣刀类型，使用默认绘制方法
        spdlog::warn("Unknown cutter type: {}, will create a default cylinder cutter",
                     cutter.str());
        UpdateCylCutter(actor, ocl::CylCutter(), p);
    }
}
