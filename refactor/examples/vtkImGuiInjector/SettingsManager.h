#pragma once

#include <boost/math/constants/constants.hpp>

namespace ocl {

/**
 * @brief Cutter和Operation的设置
 */
struct OCLSettings
{
    // Cutter设置
    int cutter_type_index = 0;  // 默认CylCutter
    double diameter = 2.0;
    double length = 10.0;
    double angle_in_deg = GetDefaultAngleInDeg();
    double radius = 0.1;

    // Operation设置
    int op_type_index = 0;  // 默认Waterline
    bool single_z_op = false; // for debuging waterline/adaptive waterline
    double sampling = 0.1;
    double min_sampling = 0.01;
    double lift_step = 0.1;
    double lift_from = 0.0;
    double lift_to = 1.0;

    // 随机批量降刀特有设置
    int random_points = 10000;

    // 获取默认角度值（因为不能用constexpr存储计算结果）
    static double GetDefaultAngleInDeg()
    {
        using namespace boost::math::constants;
        return radian<double>() * third_pi<double>();
    }
};

/**
 * @brief 设置管理器类，处理应用程序设置的加载和保存
 */
class SettingsManager {
public:
    /**
     * @brief 加载设置
     */
    static void LoadSettings();
    
    /**
     * @brief 保存设置
     */
    static void SaveSettings();
    
    /**
     * @brief 获取当前设置引用
     * @return 设置引用
     */
    static OCLSettings& GetSettings();

private:
    // 存储设置的JSON文件路径
    static constexpr char SETTINGS_JSON[] = "ocl_settings.json";
    
    // 全局设置对象
    static OCLSettings s_settings;
};

} // namespace ocl 