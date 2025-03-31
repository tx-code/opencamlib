#pragma once

class vtkDearImGuiInjector;
class vtkPoints;

namespace ocl {

/**
 * @brief UI组件类，包含各种UI绘制函数
 */
class UIComponents {
public:
    /**
     * @brief 绘制加载STL文件的UI
     * @param injector VTK ImGui注入器
     */
    static void DrawLoadStlUI(vtkDearImGuiInjector* injector);
    
    /**
     * @brief 绘制刀具添加UI
     * @param injector VTK ImGui注入器
     */
    static void DrawCutterUI(vtkDearImGuiInjector* injector);
    
    /**
     * @brief 绘制操作添加UI
     * @param injector VTK ImGui注入器
     */
    static void DrawOperationUI(vtkDearImGuiInjector* injector);
    
    /**
     * @brief 绘制数据模型UI
     * @param injector VTK ImGui注入器
     */
    static void DrawDataModelUI(vtkDearImGuiInjector* injector);
    
    /**
     * @brief 绘制刀具模型UI
     * @param injector VTK ImGui注入器
     */
    static void DrawCutterModelUI(vtkDearImGuiInjector* injector);
    
    /**
     * @brief 绘制刀具动画UI
     * @param injector VTK ImGui注入器
     * @param points 路径点集
     * @param pointIndex 当前点索引
     */
    static void DrawCutterAnimationUI(vtkDearImGuiInjector* injector, vtkPoints* points, int& pointIndex);
    
    /**
     * @brief 绘制操作模型UI
     * @param injector VTK ImGui注入器
     */
    static void DrawOperationModelUI(vtkDearImGuiInjector* injector);
    
    /**
     * @brief 绘制CAM示例UI
     * @param injector VTK ImGui注入器
     */
    static void DrawCAMExample(vtkDearImGuiInjector* injector);
};

} // namespace ocl 