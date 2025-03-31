#include "RecentFilesManager.h"

#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace ocl {

// 初始化静态成员
std::vector<std::string> RecentFilesManager::s_recentFiles;

void RecentFilesManager::LoadRecentFiles()
{
    s_recentFiles.clear();
    std::ifstream file(RECENT_FILES_JSON);

    if (file.is_open()) {
        try {
            nlohmann::json j;
            file >> j;

            if (j.contains("recent_files") && j["recent_files"].is_array()) {
                int validFileCount = 0;
                for (const auto& path : j["recent_files"]) {
                    if (path.is_string()) {
                        std::string filePath = path.get<std::string>();
                        // 检查文件是否存在
                        if (std::filesystem::exists(filePath)) {
                            s_recentFiles.push_back(filePath);
                            validFileCount++;
                        }
                    }
                }
                spdlog::info("Loaded {} recent files", validFileCount);
            }
        }
        catch (const std::exception& e) {
            spdlog::error("Error parsing recent files JSON: {}", e.what());
        }
        file.close();
    }
}

void RecentFilesManager::SaveRecentFiles()
{
    nlohmann::json j;
    j["recent_files"] = s_recentFiles;

    std::ofstream file(RECENT_FILES_JSON);
    if (file.is_open()) {
        file << j.dump(4);  // 使用4个空格缩进
        file.close();
        spdlog::info("Saved {} recent files", s_recentFiles.size());
    }
    else {
        spdlog::error("Failed to save recent files list");
    }
}

void RecentFilesManager::AddToRecentFiles(const std::string& filePath)
{
    // 如果已经在列表中，先移除
    auto it = std::find(s_recentFiles.begin(), s_recentFiles.end(), filePath);
    if (it != s_recentFiles.end()) {
        s_recentFiles.erase(it);
    }

    // 添加到列表开头
    s_recentFiles.insert(s_recentFiles.begin(), filePath);

    // 限制列表大小
    if (s_recentFiles.size() > MAX_RECENT_FILES) {
        s_recentFiles.resize(MAX_RECENT_FILES);
    }

    // 保存到JSON文件
    SaveRecentFiles();
}

const std::vector<std::string>& RecentFilesManager::GetRecentFiles()
{
    return s_recentFiles;
}

void RecentFilesManager::ClearRecentFiles()
{
    s_recentFiles.clear();
    SaveRecentFiles();
}

} // namespace ocl 