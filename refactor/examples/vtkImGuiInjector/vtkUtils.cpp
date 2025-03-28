/// Note: We assume the input actor is not empty and just update the source and mapper

#include "vtkUtils.h"

#include <cmath>
#include <vtkAppendPolyData.h>

// 空的命名空间，已经移除所有cutter相关的实现
namespace
{
}  // namespace

void UpdateStlSurfActor(vtkSmartPointer<vtkActor>& actor,
                        const ocl::STLSurf& stl,
                        const double color[3])
{
    assert(actor);
    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;

    int pointId = 0;
    for (const auto& t : stl.tris) {
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

    actor->SetMapper(mapper);
    SetActorColor(actor, color);
    actor->SetObjectName(fmt::format("STL Surface(N={})", stl.size()));
}

vtkSmartPointer<vtkLookupTable> CreateCCTypeLookupTable(bool forCLPoints)
{
    vtkNew<vtkLookupTable> lut;
    lut->SetNumberOfTableValues(static_cast<int>(ocl::CCType::CCTYPE_ERROR) + 1);
    lut->SetTableRange(0, static_cast<int>(ocl::CCType::CCTYPE_ERROR));

    // 为每种CCType设置颜色
    for (int i = 0; i <= static_cast<int>(ocl::CCType::CCTYPE_ERROR); i++) {
        auto ccType = static_cast<ocl::CCType>(i);
        double color[3] = {0, 0, 0};  // 初始化为黑色

        if (forCLPoints) {
            GetClColor(ccType, color);
        }
        else {
            GetCcColor(ccType, color);
        }

        // 显式设置RGBA值，Alpha为1.0
        lut->SetTableValue(i, color[0], color[1], color[2], 1.0);
    }

    lut->Build();
    return lut;
}

void UpdateCLPointCloudActor(vtkSmartPointer<vtkActor>& pointsActor,
                             vtkSmartPointer<vtkLegendBoxActor>& legendActor,
                             const std::vector<ocl::CLPoint>& clpoints,
                             bool forCLPoints)
{
    assert(pointsActor);
    // 创建点集
    vtkNew<vtkPoints> points;

    // 遍历所有点，添加到点集中
    for (const auto& p : clpoints) {
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
    for (const auto& p : clpoints) {
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
    mapper->SetScalarRange(0, ocl::CCType::CCTYPE_ERROR);
    mapper->SetScalarModeToUsePointData();
    mapper->ScalarVisibilityOn();

    pointsActor->SetMapper(mapper);
    pointsActor->GetProperty()->SetPointSize(5);  // 增大点的大小以便更好地可视化

    if (legendActor && !legendActor->GetNumberOfEntries()) {
        spdlog::info("Set the entries for the legend box");
        // Set the number of entries in the legend
        // CCType::CCTYPE_ERROR是最后一个枚举值，加1得到实际的枚举元素数量
        int numEntries = static_cast<int>(ocl::CCType::CCTYPE_ERROR) + 1;
        spdlog::info("Setting legend entries: {}", numEntries);
        legendActor->SetNumberOfEntries(numEntries);

        // Create a cube symbol for the legend entries
        vtkNew<vtkCubeSource> cubeSource;
        cubeSource->Update();

        // Add entries to the legend
        for (size_t i = 0; i <= ocl::CCType::CCTYPE_ERROR; i++) {
            // 使用三分量数组先获取RGB颜色
            double rgb[3] = {0, 0, 0};

            // Get the color for this CCType
            if (forCLPoints) {
                GetCcColor(static_cast<ocl::CCType>(i), rgb);
            }
            else {
                GetClColor(static_cast<ocl::CCType>(i), rgb);
            }

            // 设置图例项
            std::string typeName = ocl::CCType2String(static_cast<ocl::CCType>(i));
            legendActor->SetEntry(static_cast<int>(i),
                                  cubeSource->GetOutput(),
                                  typeName.c_str(),
                                  rgb);
        }

        // Configure legend appearance
        legendActor->UseBackgroundOn();
        double bgColor[4] = {0.1, 0.1, 0.1, 0.7};  // RGBA
        legendActor->SetBackgroundColor(bgColor);

        // Position the legend in the bottom right corner
        legendActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
        legendActor->GetPositionCoordinate()->SetValue(0.7, 0.05);
        legendActor->GetPosition2Coordinate()->SetCoordinateSystemToNormalizedViewport();
        legendActor->GetPosition2Coordinate()->SetValue(0.95, 0.45);

        // // Set appearance for text
        // vtkTextProperty* textProp = legend->GetEntryTextProperty();
        // textProp->SetFontFamilyToArial();
        // textProp->SetFontSize(10);
        // textProp->SetBold(0);
        // textProp->SetItalic(0);
        // textProp->SetShadow(0);
        // textProp->SetColor(1.0, 1.0, 1.0);
    }
}

void UpdateLoopsActor(vtkSmartPointer<vtkActor>& actor,
                      const std::vector<std::vector<std::vector<ocl::Point>>>& all_loops)
{
    assert(actor);
    // 创建一个vtkPoints对象来存储所有点
    vtkNew<vtkPoints> points;
    // 创建一个vtkCellArray对象来存储所有线段
    vtkNew<vtkCellArray> lines;
    // 创建颜色数组
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    // 创建顶点的颜色数组
    vtkNew<vtkUnsignedCharArray> pointColors;
    pointColors->SetNumberOfComponents(3);
    pointColors->SetName("PointColors");

    int pointCount = 0;
    int totalLoops = 0;

    // 准备一个颜色表，为每层提供不同颜色
    vtkNew<vtkLookupTable> lut;
    lut->SetHueRange(0.0, 0.667);  // 红色到蓝色的色调范围
    lut->SetSaturationRange(0.8, 0.8);
    lut->SetValueRange(0.8, 0.8);
    lut->SetNumberOfTableValues(!all_loops.empty() ? all_loops.size() : 1);
    lut->Build();

    // 遍历所有层级
    for (size_t layer_idx = 0; layer_idx < all_loops.size(); layer_idx++) {
        const auto& layer_loops = all_loops[layer_idx];

        // 获取该层的颜色
        double color[3];
        lut->GetColor(static_cast<double>(layer_idx)
                          / (all_loops.size() > 1 ? all_loops.size() - 1 : 1),
                      color);
        unsigned char colorUC[3];
        for (int i = 0; i < 3; i++) {
            colorUC[i] = static_cast<unsigned char>(color[i] * 255.0);
        }

        // 遍历该层级的所有循环
        for (const auto& loop : layer_loops) {
            const int loopSize = loop.size();
            if (loopSize < 2) {
                continue;  // 至少需要两个点才能形成线段
            }

            // 记录该循环的起始点索引
            int startPointId = pointCount;

            // 添加该循环的所有点
            for (const auto& p : loop) {
                points->InsertNextPoint(p.x, p.y, p.z);
                // 为每个点添加颜色
                pointColors->InsertNextTypedTuple(colorUC);
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
        return;  // 如果没有点，则直接返回

    // 创建单个polyData同时包含点和线
    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);

    // 添加顶点为Verts
    vtkNew<vtkCellArray> vertices;
    for (int i = 0; i < pointCount; i++) {
        vtkNew<vtkVertex> vertex;
        vertex->GetPointIds()->SetId(0, i);
        vertices->InsertNextCell(vertex);
    }
    polyData->SetVerts(vertices);

    // 添加线段为Lines
    polyData->SetLines(lines);

    // 设置点的颜色 - 将颜色数据设置为点数据
    polyData->GetPointData()->SetScalars(pointColors);

    // 创建单元颜色数组，包含顶点和线段的颜色
    vtkNew<vtkUnsignedCharArray> cellColors;
    cellColors->SetNumberOfComponents(3);
    cellColors->SetName("CellColors");

    // 先添加所有顶点的颜色
    for (int i = 0; i < pointCount; i++) {
        cellColors->InsertNextTypedTuple(pointColors->GetPointer(3 * i));
    }

    // 再添加所有线段的颜色
    for (int i = 0; i < lines->GetNumberOfCells(); i++) {
        cellColors->InsertNextTypedTuple(colors->GetPointer(3 * i));
    }

    // 设置单元颜色
    polyData->GetCellData()->SetScalars(cellColors);

    // 创建mapper并设置输入数据
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    // 使用单元数据的颜色进行渲染
    mapper->SetScalarModeToUseCellData();
    mapper->ScalarVisibilityOn();

    // 设置mapper
    actor->SetMapper(mapper);
    // 设置点的大小
    actor->GetProperty()->SetPointSize(5);

    spdlog::info("Rendered {} loops across {} layers with total {} points and {} lines",
                 totalLoops,
                 all_loops.size(),
                 pointCount,
                 lines->GetNumberOfCells());
}
