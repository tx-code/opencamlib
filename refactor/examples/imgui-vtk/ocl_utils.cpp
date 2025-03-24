#include "ocl_utils.h"

void DrawStlSurf(VtkViewer& viewer, const ocl::STLSurf& stl, const double color[3]) {
    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;

    int pointId = 0;
    for (const auto& t: stl.tris) {
        vtkNew<vtkTriangle> triangle;

        // Add points for this triangle
        for (int j = 0; j < 3; j++) {
            const ocl::Point& p = t.p[j];
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

void DrawCLPointCloudWithLUT(VtkViewer& viewer, const std::vector<ocl::CLPoint>& clpoints,
                             bool forCLPoints) {
    // 创建点集
    vtkNew<vtkPoints> points;

    // 遍历所有点，添加到点集中
    for (const auto& p: clpoints) {
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
    for (const auto& p: clpoints) {
        auto* cc = p.cc.load();
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

void DrawAllLoops(VtkViewer& viewer, const std::vector<std::vector<std::vector<ocl::Point>>>& all_loops) {
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
        const auto& layer_loops = all_loops[layer_idx];

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
        for (const auto& loop: layer_loops) {
            int loopSize = loop.size();
            if (loopSize < 2) {
                continue; // 至少需要两个点才能形成线段
            }

            // 记录该循环的起始点索引
            int startPointId = pointCount;

            // 添加该循环的所有点
            for (const auto& p: loop) {
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
