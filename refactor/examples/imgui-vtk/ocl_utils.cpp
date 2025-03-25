#include "ocl_utils.h"
#include <cmath>
#include <vtkAppendPolyData.h>

namespace {
// 绘制圆柱铣刀(Cylindrical Cutter)
// 圆柱铣刀的CL位置在圆柱Cap的中心
inline vtkSmartPointer<vtkActor> DrawCylCutter(VtkViewer &viewer,
                                               const ocl::CylCutter &cutter,
                                               const ocl::Point &p) {
  double r = cutter.getRadius();
  double flutesLength = cutter.getLength();

  // 创建圆柱体
  vtkNew<vtkCylinderSource> cylinderSource;
  cylinderSource->SetCenter(p.x, p.y - flutesLength / 2.0, p.z);
  cylinderSource->SetHeight(flutesLength);
  cylinderSource->SetRadius(r);
  cylinderSource->SetResolution(30);
  cylinderSource->CappingOn();

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(cylinderSource->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(red[0], red[1], red[2]);
  actor->RotateX(-90);
  SetActorWireframe(actor);

  viewer.addActor(actor, VtkViewer::AT_Cutter);
  return actor;
}

// 绘制球头铣刀(Ball Cutter)
// 球头铣刀的CL位置在球头中心 (TODO: 需要验证)
inline vtkSmartPointer<vtkActor> DrawBallCutter(VtkViewer &viewer,
                                                const ocl::BallCutter &cutter,
                                                const ocl::Point &p) {
  double r = cutter.getRadius();
  double flutesLength = cutter.getLength();

  // 创建圆柱部分（刀杆）
  vtkNew<vtkCylinderSource> cylinderSource;
  cylinderSource->SetCenter(p.x, p.y - (flutesLength - r) / 2.0, p.z);
  cylinderSource->SetHeight(flutesLength - r);
  cylinderSource->SetRadius(r);
  cylinderSource->SetResolution(30);
  cylinderSource->CappingOn();

  // 创建球体部分
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->SetCenter(p.x, p.y, p.z);
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

  // 创建映射器和演员
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(appendFilter->GetOutput());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->RotateX(-90);
  actor->GetProperty()->SetColor(yellow[0], yellow[1], yellow[2]);
  SetActorWireframe(actor);

  viewer.addActor(actor, VtkViewer::AT_Cutter);
  return actor;
}

// 绘制牛头铣刀(Bull Cutter)
// FIXME: 刀头部分不知如何绘制
inline vtkSmartPointer<vtkActor> DrawBullCutter(VtkViewer &viewer,
                                                const ocl::BullCutter &cutter,
                                                const ocl::Point &p) {
  double r1 = cutter.getRadius() - cutter.getRadius2(); // 圆柱部分半径
  double r2 = cutter.getRadius2();                      // 圆角半径
  double flutesLength = cutter.getLength();

  // 创建圆柱部分
  vtkNew<vtkCylinderSource> cylinderSource;
  cylinderSource->SetHeight(flutesLength - r2);
  cylinderSource->SetRadius(r1);
  cylinderSource->SetResolution(30);
  cylinderSource->CappingOn();

  // 移动圆柱到正确位置
  vtkNew<vtkTransform> cylinderTransform;
  cylinderTransform->Translate(p.x, p.y, p.z + r2 + (flutesLength - r2) / 2.0);
  cylinderTransform->RotateX(90); // 纵轴旋转为Z轴

  vtkNew<vtkTransformPolyDataFilter> cylinderTransformFilter;
  cylinderTransformFilter->SetInputConnection(cylinderSource->GetOutputPort());
  cylinderTransformFilter->SetTransform(cylinderTransform);

  // 创建底部圆环（使用参数曲面来表示环面）
  vtkNew<vtkParametricSuperToroid> toroidSource;
  toroidSource->SetN1(1.0);
  toroidSource->SetN2(1.0);
  toroidSource->SetRingRadius(r1);
  toroidSource->SetCrossSectionRadius(r2);
  toroidSource->SetXRadius(r1);
  toroidSource->SetYRadius(r1);
  toroidSource->SetZRadius(r2);

  vtkNew<vtkParametricFunctionSource> toroidFunctionSource;
  toroidFunctionSource->SetParametricFunction(toroidSource);
  toroidFunctionSource->SetUResolution(30);
  toroidFunctionSource->SetVResolution(30);
  toroidFunctionSource->SetWResolution(30);
  toroidFunctionSource->Update();

  // 移动圆环到正确位置
  vtkNew<vtkTransform> toroidTransform;
  toroidTransform->Translate(p.x, p.y, p.z + r2);

  vtkNew<vtkTransformPolyDataFilter> toroidTransformFilter;
  toroidTransformFilter->SetInputConnection(
      toroidFunctionSource->GetOutputPort());
  toroidTransformFilter->SetTransform(toroidTransform);

  // 合并圆柱和圆环
  vtkNew<vtkAppendPolyData> appendFilter;
  appendFilter->AddInputConnection(cylinderTransformFilter->GetOutputPort());
  appendFilter->AddInputConnection(toroidTransformFilter->GetOutputPort());
  appendFilter->Update();

  // 创建映射器和演员
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(appendFilter->GetOutput());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(green[0], green[1], green[2]);
  SetActorWireframe(actor);

  viewer.addActor(actor, VtkViewer::AT_Cutter);
  return actor;
}

// 绘制锥形铣刀(Cone Cutter)
// 对于锥形铣刀，圆锥顶点即为CL位置 （TODO: 需要验证）
inline vtkSmartPointer<vtkActor> DrawConeCutter(VtkViewer &viewer,
                                                const ocl::ConeCutter &cutter,
                                                const ocl::Point &p) {
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

  // 创建映射器和演员
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(appendFilter->GetOutput());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(blue[0], blue[1], blue[2]);
  actor->RotateX(-90);
  SetActorWireframe(actor);

  viewer.addActor(actor, VtkViewer::AT_Cutter);
  return actor;
}

} // namespace

void DrawStlSurf(VtkViewer &viewer, const ocl::STLSurf &stl,
                 const double color[3]) {
  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> triangles;

  int pointId = 0;
  for (const auto &t : stl.tris) {
    vtkNew<vtkTriangle> triangle;

    // Add points for this triangle
    for (int j = 0; j < 3; j++) {
      const ocl::Point &p = t.p[j];
      points->InsertNextPoint(p.x, p.y, p.z);
      triangle->GetPointIds()->SetId(j, pointId++);
    }

    triangles->InsertNextCell(triangle);
  }

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  SetActorColor(actor, color);
  actor->SetObjectName(fmt::format("STL Surface(N={})", stl.size()));

  viewer.addActor(actor, VtkViewer::AT_Model);
}

vtkSmartPointer<vtkLookupTable> CreateCCTypeLookupTable(bool forCLPoints) {
  vtkNew<vtkLookupTable> lut;
  lut->SetNumberOfTableValues(static_cast<int>(ocl::CCType::CCTYPE_ERROR) + 1);
  lut->SetTableRange(0, static_cast<int>(ocl::CCType::CCTYPE_ERROR));

  // 为每种CCType设置颜色
  for (int i = 0; i <= static_cast<int>(ocl::CCType::CCTYPE_ERROR); i++) {
    auto ccType = static_cast<ocl::CCType>(i);
    double color[3];

    if (forCLPoints) {
      GetClColor(ccType, color);
    } else {
      GetCcColor(ccType, color);
    }

    lut->SetTableValue(i, color[0], color[1], color[2], 1.0);
  }

  lut->Build();
  return lut;
}

void DrawCLPointCloudWithLUT(VtkViewer &viewer,
                             const std::vector<ocl::CLPoint> &clpoints,
                             bool forCLPoints) {
  // 创建点集
  vtkNew<vtkPoints> points;

  // 遍历所有点，添加到点集中
  for (const auto &p : clpoints) {
    points->InsertNextPoint(p.x, p.y, p.z);
  }

  // 创建多边形数据对象
  vtkNew<vtkPolyData> pointsPolydata;
  pointsPolydata->SetPoints(points);

  // 使用顶点滤波器将点转换为可渲染的顶点
  vtkNew<vtkVertexGlyphFilter> vertexFilter;
  vertexFilter->SetInputData(pointsPolydata);
  vertexFilter->Update();

  // 复制结果数据
  vtkNew<vtkPolyData> polydata;
  polydata->ShallowCopy(vertexFilter->GetOutput());

  // 创建用于存储CCType整型值的数组
  vtkNew<vtkIntArray> typeValues;
  typeValues->SetNumberOfComponents(1);
  typeValues->SetName("CCType");

  // 添加每个点的类型值
  for (const auto &p : clpoints) {
    auto *cc = p.cc.load();
    typeValues->InsertNextValue(static_cast<int>(cc->type));
  }

  // 将类型值添加到多边形数据的点数据中
  polydata->GetPointData()->SetScalars(typeValues);

  // 创建颜色查找表
  auto lut = CreateCCTypeLookupTable(forCLPoints);

  // 设置映射器
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polydata);
  mapper->SetLookupTable(lut);
  mapper->SetScalarRange(0, static_cast<int>(ocl::CCType::CCTYPE_ERROR));
  mapper->SetScalarModeToUsePointData();
  mapper->ScalarVisibilityOn();

  // 创建演员对象
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(5); // 增大点的大小以便更好地可视化

  viewer.addActor(actor, VtkViewer::AT_Operation);
}

void DrawAllLoops(
    VtkViewer &viewer,
    const std::vector<std::vector<std::vector<ocl::Point>>> &all_loops) {
  // 创建一个vtkPoints对象来存储所有点
  vtkNew<vtkPoints> points;
  // 创建一个vtkCellArray对象来存储所有线段
  vtkNew<vtkCellArray> lines;
  // 创建颜色数组
  vtkNew<vtkUnsignedCharArray> colors;
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");

  int pointCount = 0;
  int totalLoops = 0;

  // 准备一个颜色表，为每层提供不同颜色
  vtkNew<vtkLookupTable> lut;
  lut->SetHueRange(0.0, 0.667); // 红色到蓝色的色调范围
  lut->SetSaturationRange(0.8, 0.8);
  lut->SetValueRange(0.8, 0.8);
  lut->SetNumberOfTableValues(all_loops.size() > 0 ? all_loops.size() : 1);
  lut->Build();

  // 遍历所有层级
  for (size_t layer_idx = 0; layer_idx < all_loops.size(); layer_idx++) {
    const auto &layer_loops = all_loops[layer_idx];

    // 获取该层的颜色
    double color[3];
    lut->GetColor(static_cast<double>(layer_idx) /
                      (all_loops.size() > 1 ? all_loops.size() - 1 : 1),
                  color);
    unsigned char colorUC[3];
    for (int i = 0; i < 3; i++) {
      colorUC[i] = static_cast<unsigned char>(color[i] * 255.0);
    }

    // 遍历该层级的所有循环
    for (const auto &loop : layer_loops) {
      int loopSize = loop.size();
      if (loopSize < 2) {
        continue; // 至少需要两个点才能形成线段
      }

      // 记录该循环的起始点索引
      int startPointId = pointCount;

      // 添加该循环的所有点
      for (const auto &p : loop) {
        points->InsertNextPoint(p.x, p.y, p.z);
        pointCount++;
      }

      // 创建该循环的线段并设置颜色
      for (int i = 0; i < loopSize; i++) {
        vtkNew<vtkLine> line;
        line->GetPointIds()->SetId(0, startPointId + i);
        line->GetPointIds()->SetId(1, startPointId + (i + 1) % loopSize);
        lines->InsertNextCell(line);

        // 为每条线段添加颜色
        colors->InsertNextTypedTuple(colorUC);
      }

      totalLoops++;
    }
  }

  if (pointCount == 0)
    return; // 如果没有点，则直接返回

  // 创建vtkPolyData对象并设置点和线
  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetLines(lines);
  polyData->GetCellData()->SetScalars(colors);

  // 创建mapper并设置输入数据
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);

  // 创建actor并设置mapper
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  // 添加actor到viewer
  viewer.addActor(actor, VtkViewer::AT_Operation);

  spdlog::info(
      "Rendered {} loops across {} layers with total {} points and {} lines",
      totalLoops, all_loops.size(), pointCount, lines->GetNumberOfCells());
}

// 通用绘制铣刀函数，根据铣刀类型动态选择绘制方法
void DrawCutter(VtkViewer &viewer, const ocl::MillingCutter &cutter,
                const ocl::Point &p) {
  // 使用dynamic_cast和typeid判断铣刀类型
  if (const ocl::CylCutter *cylCutter =
          dynamic_cast<const ocl::CylCutter *>(&cutter)) {
    // 绘制圆柱铣刀
    DrawCylCutter(viewer, *cylCutter, p);
  } else if (const ocl::BallCutter *ballCutter =
                 dynamic_cast<const ocl::BallCutter *>(&cutter)) {
    // 绘制球头铣刀
    DrawBallCutter(viewer, *ballCutter, p);
  } else if (const ocl::BullCutter *bullCutter =
                 dynamic_cast<const ocl::BullCutter *>(&cutter)) {
    // 绘制牛头铣刀
    DrawBullCutter(viewer, *bullCutter, p);
  } else if (const ocl::ConeCutter *coneCutter =
                 dynamic_cast<const ocl::ConeCutter *>(&cutter)) {
    // 绘制锥形铣刀
    DrawConeCutter(viewer, *coneCutter, p);
  } else {
    // 未知铣刀类型，使用默认绘制方法
    spdlog::warn("Unknown cutter type: {}", cutter.str());

    // 绘制简单的圆柱体表示
    double center[3] = {p.x, p.y, p.z};
    auto defaultCyl =
        CreateCylinder(center, cutter.getRadius(), cutter.getLength(), grey);
    SetActorWireframe(defaultCyl);
    viewer.addActor(defaultCyl);
  }
}
