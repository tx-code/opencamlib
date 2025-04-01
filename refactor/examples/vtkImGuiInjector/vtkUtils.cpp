/// Note: We assume the input actor is not empty and just update the source and mapper

#include "vtkUtils.h"

#include <cmath>
#include <vtkAppendPolyData.h>
#include <vtkSignedDistance.h>

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

void UpdateKDTreeActor(vtkSmartPointer<vtkActor>& actor,
                       const ocl::KDTree<ocl::Triangle>* kdtree,
                       double opacity,
                       bool onlyLeafNodes)
{
    using node_type = ocl::KDNode<ocl::Triangle>;
    if (!kdtree || !kdtree->getRoot()) {
        spdlog::error("KDTree is null or has no root node");
        return;
    }

    vtkNew<vtkUnstructuredGrid> grid;
    vtkNew<vtkPoints> points;

    if (onlyLeafNodes) {
        // 递归函数来找到叶子节点并创建可视化
        std::function<void(node_type*)> findLeafNodes = [&](node_type* node) {
            if (!node)
                return;

            // 如果是叶子节点
            if (node->isLeaf && node->tris && !node->tris->empty()) {
                // 计算叶子节点的包围盒
                ocl::Bbox bbox;
                bool first = true;

                for (const auto& obj : *(node->tris)) {
                    if (first) {
                        bbox = obj.bb;
                        first = false;
                    }
                    else {
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
            }
            else {
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
    }
    else {
        // 递归函数来构建KDTree的可视化网格
        std::function<void(node_type*, int)> buildGridFromNode = [&](node_type* node, int depth) {
            if (!node)
                return;

            // 获取节点的包围盒
            ocl::Bbox bbox;

            // 如果是叶子节点，从三角形构建包围盒
            if (node->isLeaf && node->tris) {
                bool first = true;
                for (const auto& obj : *(node->tris)) {
                    if (first) {
                        bbox = obj.bb;
                        first = false;
                    }
                    else {
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
                    if (node->dim == 0)
                        xmax = node->cutval;  // X min
                    else if (node->dim == 1)
                        xmin = node->cutval;  // X max
                    else if (node->dim == 2)
                        ymax = node->cutval;  // Y min
                    else if (node->dim == 3)
                        ymin = node->cutval;  // Y max
                    else if (node->dim == 4)
                        zmax = node->cutval;  // Z min
                    else if (node->dim == 5)
                        zmin = node->cutval;  // Z max

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

void UpdateAABBTreeActor(vtkSmartPointer<vtkActor>& actor,
                         const ocl::AABBTreeAdaptor& aabbTree,
                         double opacity,
                         int showLevel)
{
    std::vector<std::vector<CGAL::Bbox_3>> boxes;

    // 从AABB树获取根节点
    const auto& tree = aabbTree.getTree();
    if (!tree.size()) {
        spdlog::error("AABBTree is empty");
        return;
    }

    // 递归遍历AABB树，收集所有高度层级的包围盒
    // 递归函数来收集包围盒
    std::function<void(const std::size_t, const ocl::AABBTreeAdaptor::Node&, int)> traversal =
        [&boxes, &traversal](const std::size_t nb_primitives,
                             const ocl::AABBTreeAdaptor::Node& node,
                             int lvl) {
            // 确保boxes有足够空间存储当前层级的boxes
            if (static_cast<int>(boxes.size()) <= lvl)
                boxes.push_back(std::vector<CGAL::Bbox_3>());

            // 添加当前节点的包围盒
            boxes[lvl].push_back(node.bbox());

            // 递归遍历
            switch (nb_primitives) {
                case 2:
                    break;
                case 3:
                    traversal(2, node.right_child(), lvl + 1);
                    break;
                default:
                    traversal(nb_primitives / 2, node.left_child(), lvl + 1);
                    traversal(nb_primitives - nb_primitives / 2, node.right_child(), lvl + 1);
            }
        };

    // 从根节点开始遍历
    if (tree.size() > 0) {
        traversal(tree.size(), *tree.root_node(), 0);
    }

    // 创建网格对象来表示AABB树
    vtkNew<vtkUnstructuredGrid> grid;
    vtkNew<vtkPoints> points;

    // 根据showLevel参数决定显示哪一层级的包围盒
    int level_to_show = showLevel;

    if (level_to_show == -1) {
        // 显示所有层级的包围盒
        spdlog::info("AABBTree has {} levels, showing all levels", boxes.size());

        // 遍历所有层级和每层的所有包围盒
        for (size_t lvl = 0; lvl < boxes.size(); ++lvl) {
            for (const auto& bbox : boxes[lvl]) {
                // 创建一个vtkVoxel来表示此包围盒
                vtkNew<vtkVoxel> voxel;

                // 从CGAL的Bbox_3获取边界点坐标
                double xmin = bbox.xmin(), xmax = bbox.xmax();
                double ymin = bbox.ymin(), ymax = bbox.ymax();
                double zmin = bbox.zmin(), zmax = bbox.zmax();

                // 定义八个顶点（与KDTree的实现类似）
                const vtkIdType pointIds[8] = {
                    points->InsertNextPoint(xmin, ymin, zmin),  // 0: minx, miny, minz
                    points->InsertNextPoint(xmax, ymin, zmin),  // 1: maxx, miny, minz
                    points->InsertNextPoint(xmin, ymax, zmin),  // 2: minx, maxy, minz
                    points->InsertNextPoint(xmax, ymax, zmin),  // 3: maxx, maxy, minz
                    points->InsertNextPoint(xmin, ymin, zmax),  // 4: minx, miny, maxz
                    points->InsertNextPoint(xmax, ymin, zmax),  // 5: maxx, miny, maxz
                    points->InsertNextPoint(xmin, ymax, zmax),  // 6: minx, maxy, maxz
                    points->InsertNextPoint(xmax, ymax, zmax)   // 7: maxx, maxy, maxz
                };

                // 设置voxel的顶点
                for (int i = 0; i < 8; ++i) {
                    voxel->GetPointIds()->SetId(i, pointIds[i]);
                }

                // 将voxel插入到网格中
                grid->InsertNextCell(voxel->GetCellType(), voxel->GetPointIds());
            }
        }

        spdlog::info("Created AABB visualization with all boxes from all levels");
    }
    else {
        // 确保level_to_show在有效范围内
        if (level_to_show < 0 || level_to_show >= static_cast<int>(boxes.size())) {
            level_to_show = boxes.size() - 1;
        }

        spdlog::info("AABBTree has {} levels, showing level {}", boxes.size(), level_to_show);

        // 如果没有节点数据，则返回
        if (boxes.empty() || boxes[level_to_show].empty()) {
            spdlog::warn("No AABBTree boxes to display at level {}", level_to_show);
            return;
        }

        // 遍历选定层级的所有包围盒，创建vtkVoxel对象
        for (const auto& bbox : boxes[level_to_show]) {
            // 创建一个vtkVoxel来表示此包围盒
            vtkNew<vtkVoxel> voxel;

            // 从CGAL的Bbox_3获取边界点坐标
            double xmin = bbox.xmin(), xmax = bbox.xmax();
            double ymin = bbox.ymin(), ymax = bbox.ymax();
            double zmin = bbox.zmin(), zmax = bbox.zmax();

            // 定义八个顶点（与KDTree的实现类似）
            const vtkIdType pointIds[8] = {
                points->InsertNextPoint(xmin, ymin, zmin),  // 0: minx, miny, minz
                points->InsertNextPoint(xmax, ymin, zmin),  // 1: maxx, miny, minz
                points->InsertNextPoint(xmin, ymax, zmin),  // 2: minx, maxy, minz
                points->InsertNextPoint(xmax, ymax, zmin),  // 3: maxx, maxy, minz
                points->InsertNextPoint(xmin, ymin, zmax),  // 4: minx, miny, maxz
                points->InsertNextPoint(xmax, ymin, zmax),  // 5: maxx, miny, maxz
                points->InsertNextPoint(xmin, ymax, zmax),  // 6: minx, maxy, maxz
                points->InsertNextPoint(xmax, ymax, zmax)   // 7: maxx, maxy, maxz
            };

            // 设置voxel的顶点
            for (int i = 0; i < 8; ++i) {
                voxel->GetPointIds()->SetId(i, pointIds[i]);
            }

            // 将voxel插入到网格中
            grid->InsertNextCell(voxel->GetCellType(), voxel->GetPointIds());
        }

        spdlog::info("Created AABB visualization with {} boxes at level {}",
                     boxes[level_to_show].size(),
                     level_to_show);
    }

    // 设置网格的点
    grid->SetPoints(points);

    // 创建mapper
    vtkNew<vtkDataSetMapper> mapper;
    mapper->SetInputData(grid);

    // 更新actor
    actor->SetMapper(mapper);

    // 设置颜色和透明度
    SetActorColor(actor, green);
    SetActorOpacity(actor, opacity);

    // 设置为线框显示模式
    SetActorWireframe(actor);
}

void UpdateOverlappedTrianglesActor(vtkSmartPointer<vtkActor>& actor,
                                    const std::vector<ocl::Triangle>& triangles,
                                    const double color[3],
                                    double opacity)
{
    assert(actor);
    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> cells;

    int pointId = 0;
    for (const auto triangle : triangles) {

        vtkNew<vtkTriangle> vtkTri;

        // Add points for this triangle
        for (int j = 0; j < 3; j++) {
            const ocl::Point& p = triangle.p[j];
            points->InsertNextPoint(p.x, p.y, p.z);
            vtkTri->GetPointIds()->SetId(j, pointId++);
        }

        cells->InsertNextCell(vtkTri);
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetPolys(cells);

    // 计算表面法线以实现更好的渲染效果
    vtkNew<vtkPolyDataNormals> normalGenerator;
    normalGenerator->SetInputData(polyData);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(normalGenerator->GetOutputPort());

    // 调整mapper设置确保可见性
    mapper->SetResolveCoincidentTopologyToPolygonOffset();
    mapper->SetResolveCoincidentTopologyPolygonOffsetParameters(-1.0, -1.0);

    actor->SetMapper(mapper);
    SetActorColor(actor, color);
    SetActorOpacity(actor, opacity);

    // 设置高亮效果
    actor->GetProperty()->SetEdgeVisibility(true);
    actor->GetProperty()->SetEdgeColor(1.0, 1.0, 1.0);  // 白色边缘
    actor->GetProperty()->SetLineWidth(2.0);

    // 提高渲染顺序（确保在其他对象之后渲染）
    actor->SetPosition(0, 0, 0.01);  // 略微向观察者方向移动

    actor->SetObjectName(fmt::format("Overlapped Triangles(N={})", triangles.size()));
}
