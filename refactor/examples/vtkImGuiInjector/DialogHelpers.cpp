// SPDX-FileCopyrightText: Copyright 2023 Jaswant Panchumarti
// SPDX-License-Identifier: BSD-3-Clause

#include "DialogHelpers.h"

#include <codecvt>
#include <imgui.h>
#include <nfd.h>
#include <spdlog/spdlog.h>

namespace ocl {

std::wstring DialogHelpers::ToWString(const char* str)
{
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    return converter.from_bytes(str);
}

void DialogHelpers::HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool DialogHelpers::OpenSTLFileDialog(std::string& outFilePath)
{
    NFD_Init();

    nfdu8char_t* outPath = nullptr;
    nfdu8filteritem_t filters[1] = {{"STL Models", "stl"}};
    nfdopendialogu8args_t args = {0};
    args.filterList = filters;
    args.filterCount = 1;
    auto result = NFD_OpenDialogU8_With(&outPath, &args);

    bool success = false;
    if (result == NFD_OKAY) {
        outFilePath = outPath;
        NFD_FreePathU8(outPath);
        success = true;
    }
    else if (result == NFD_CANCEL) {
        // 用户取消对话框
        spdlog::info("User canceled the dialog");
    }
    else {
        // 处理其他错误代码
        spdlog::error("Error: {}", NFD_GetError());
    }

    NFD_Quit();
    return success;
}

} // namespace ocl 