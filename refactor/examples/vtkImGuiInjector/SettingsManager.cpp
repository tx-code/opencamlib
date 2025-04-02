#include "SettingsManager.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace ocl
{

// 初始化静态成员
OCLSettings SettingsManager::s_settings;

void SettingsManager::LoadSettings()
{
    std::ifstream file(SETTINGS_JSON);

    if (file.is_open()) {
        try {
            nlohmann::json j;
            file >> j;

            // 加载Cutter设置
            if (j.contains("cutter")) {
                auto& cutter = j["cutter"];

                if (cutter.contains("type_index"))
                    s_settings.cutter_type_index = cutter["type_index"].get<int>();

                if (cutter.contains("diameter"))
                    s_settings.diameter = cutter["diameter"].get<double>();

                if (cutter.contains("length"))
                    s_settings.length = cutter["length"].get<double>();

                if (cutter.contains("angle_in_deg"))
                    s_settings.angle_in_deg = cutter["angle_in_deg"].get<double>();

                if (cutter.contains("radius"))
                    s_settings.radius = cutter["radius"].get<double>();
            }

            // 加载Operation设置
            if (j.contains("operation")) {
                auto& op = j["operation"];

                if (op.contains("type_index"))
                    s_settings.op_type_index = op["type_index"].get<int>();

                if (op.contains("single_z_op"))
                    s_settings.single_z_op = op["single_z_op"].get<bool>();

                if (op.contains("sampling"))
                    s_settings.sampling = op["sampling"].get<double>();

                if (op.contains("min_sampling"))
                    s_settings.min_sampling = op["min_sampling"].get<double>();

                if (op.contains("lift_step"))
                    s_settings.lift_step = op["lift_step"].get<double>();

                if (op.contains("lift_from"))
                    s_settings.lift_from = op["lift_from"].get<double>();

                if (op.contains("lift_to"))
                    s_settings.lift_to = op["lift_to"].get<double>();

                if (op.contains("random_points"))
                    s_settings.random_points = op["random_points"].get<int>();
            }

            spdlog::info("Settings loaded successfully");
        }
        catch (const std::exception& e) {
            spdlog::error("Error parsing settings JSON: {}", e.what());
        }
        file.close();
    }
    else {
        spdlog::info("No settings file found, using defaults");
    }
}

void SettingsManager::SaveSettings()
{
    nlohmann::json j;

    // 保存Cutter设置
    j["cutter"] = {{"type_index", s_settings.cutter_type_index},
                   {"diameter", s_settings.diameter},
                   {"length", s_settings.length},
                   {"angle_in_deg", s_settings.angle_in_deg},
                   {"radius", s_settings.radius}};

    // 保存Operation设置
    j["operation"] = {{"type_index", s_settings.op_type_index},
                      {"single_z_op", s_settings.single_z_op},
                      {"sampling", s_settings.sampling},
                      {"min_sampling", s_settings.min_sampling},
                      {"lift_step", s_settings.lift_step},
                      {"lift_from", s_settings.lift_from},
                      {"lift_to", s_settings.lift_to},
                      {"random_points", s_settings.random_points}};

    std::ofstream file(SETTINGS_JSON);
    if (file.is_open()) {
        file << j.dump(4);  // 使用4个空格缩进
        file.close();
        spdlog::info("Settings saved successfully");
    }
    else {
        spdlog::error("Failed to save settings");
    }
}

OCLSettings& SettingsManager::GetSettings()
{
    return s_settings;
}

}  // namespace ocl