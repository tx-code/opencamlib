#pragma once

#include <string>
#include <vector>

namespace ocl {

/**
 * @brief 管理最近打开文件列表的类
 */
class RecentFilesManager {
public:
    /**
     * @brief 加载最近文件列表
     */
    static void LoadRecentFiles();
    
    /**
     * @brief 保存最近文件列表到JSON文件
     */
    static void SaveRecentFiles();
    
    /**
     * @brief 添加文件到最近文件列表
     * @param filePath 文件路径
     */
    static void AddToRecentFiles(const std::string& filePath);
    
    /**
     * @brief 获取最近文件列表
     * @return 最近文件列表
     */
    static const std::vector<std::string>& GetRecentFiles();
    
    /**
     * @brief 清空最近文件列表
     */
    static void ClearRecentFiles();

private:
    // 最大保存的最近文件数量
    static constexpr int MAX_RECENT_FILES = 10;
    
    // 存储JSON文件的路径
    static constexpr char RECENT_FILES_JSON[] = "recent_files.json";
    
    // 存储最近打开的文件路径
    static std::vector<std::string> s_recentFiles;
};

} // namespace ocl 