// SPDX-FileCopyrightText: Copyright 2023 Jaswant Panchumarti
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <string>

namespace ocl {

/**
 * @brief 对话框辅助函数类
 */
class DialogHelpers {
public:
    /**
     * @brief 将ASCII字符串转换为宽字符串
     * @param str ASCII字符串
     * @return 宽字符串
     */
    static std::wstring ToWString(const char* str);
    
    /**
     * @brief 显示带有问号图标的帮助标记，鼠标悬停时显示提示
     * @param desc 提示内容
     */
    static void HelpMarker(const char* desc);
    
    /**
     * @brief 打开文件对话框选择STL文件
     * @param outFilePath 选择的文件路径输出
     * @return 是否成功选择文件
     */
    static bool OpenSTLFileDialog(std::string& outFilePath);
};

} // namespace ocl 