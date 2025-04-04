﻿#pragma once

#include <imgui.h>
#include <string>
#include <unordered_map>

// Translated from VTK KeySym to ImGuiKey
inline std::unordered_map<std::string, ImGuiKey> KeySymToImGuiKey = {
    {"Cancel", ImGuiKey_Escape},
    {"BackSpace", ImGuiKey_Backspace},
    {"Tab", ImGuiKey_Tab},
    {"Return", ImGuiKey_Enter},
    {"Shift_L", ImGuiKey_LeftShift},
    {"Control_L", ImGuiKey_LeftCtrl},
    {"Alt_L", ImGuiKey_LeftAlt},
    {"Pause", ImGuiKey_Pause},
    {"Caps_Lock", ImGuiKey_CapsLock},
    {"Escape", ImGuiKey_Escape},
    {"space", ImGuiKey_Space},
    {"Prior", ImGuiKey_PageUp},
    {"Next", ImGuiKey_PageDown},
    {"End", ImGuiKey_End},
    {"Home", ImGuiKey_Home},
    {"Left", ImGuiKey_LeftArrow},
    {"Up", ImGuiKey_UpArrow},
    {"Right", ImGuiKey_RightArrow},
    {"Down", ImGuiKey_DownArrow},
    {"Snapshot", ImGuiKey_PrintScreen},
    {"Insert", ImGuiKey_Insert},
    {"Delete", ImGuiKey_Delete},
    {"0", ImGuiKey_0},
    {"1", ImGuiKey_1},
    {"2", ImGuiKey_2},
    {"3", ImGuiKey_3},
    {"4", ImGuiKey_4},
    {"5", ImGuiKey_5},
    {"6", ImGuiKey_6},
    {"7", ImGuiKey_7},
    {"8", ImGuiKey_8},
    {"9", ImGuiKey_9},
    {"a", ImGuiKey_A},
    {"b", ImGuiKey_B},
    {"c", ImGuiKey_C},
    {"d", ImGuiKey_D},
    {"e", ImGuiKey_E},
    {"f", ImGuiKey_F},
    {"g", ImGuiKey_G},
    {"h", ImGuiKey_H},
    {"i", ImGuiKey_I},
    {"j", ImGuiKey_J},
    {"k", ImGuiKey_K},
    {"l", ImGuiKey_L},
    {"m", ImGuiKey_M},
    {"n", ImGuiKey_N},
    {"o", ImGuiKey_O},
    {"p", ImGuiKey_P},
    {"q", ImGuiKey_Q},
    {"r", ImGuiKey_R},
    {"s", ImGuiKey_S},
    {"t", ImGuiKey_T},
    {"u", ImGuiKey_U},
    {"v", ImGuiKey_V},
    {"w", ImGuiKey_W},
    {"x", ImGuiKey_X},
    {"y", ImGuiKey_Y},
    {"z", ImGuiKey_Z},
    {"KP_0", ImGuiKey_Keypad0},
    {"KP_1", ImGuiKey_Keypad1},
    {"KP_2", ImGuiKey_Keypad2},
    {"KP_3", ImGuiKey_Keypad3},
    {"KP_4", ImGuiKey_Keypad4},
    {"KP_5", ImGuiKey_Keypad5},
    {"KP_6", ImGuiKey_Keypad6},
    {"KP_7", ImGuiKey_Keypad7},
    {"KP_8", ImGuiKey_Keypad8},
    {"KP_9", ImGuiKey_Keypad9},
    {"asterisk", ImGuiKey_KeypadMultiply},
    {"plus", ImGuiKey_KeypadAdd},
    {"bar", ImGuiKey_KeypadEnter},
    {"minus", ImGuiKey_KeypadSubtract},
    {"period", ImGuiKey_KeypadDecimal},
    {"slash", ImGuiKey_KeypadDivide},
    {"F1", ImGuiKey_F1},
    {"F2", ImGuiKey_F2},
    {"F3", ImGuiKey_F3},
    {"F4", ImGuiKey_F4},
    {"F5", ImGuiKey_F5},
    {"F6", ImGuiKey_F6},
    {"F7", ImGuiKey_F7},
    {"F8", ImGuiKey_F8},
    {"F9", ImGuiKey_F9},
    {"F10", ImGuiKey_F10},
    {"F11", ImGuiKey_F11},
    {"F12", ImGuiKey_F12},
    {"F13", ImGuiKey_F13},
    {"F14", ImGuiKey_F14},
    {"F15", ImGuiKey_F15},
    {"F16", ImGuiKey_F16},
    {"F17", ImGuiKey_F17},
    {"F18", ImGuiKey_F18},
    {"F19", ImGuiKey_F19},
    {"F20", ImGuiKey_F20},
    {"F21", ImGuiKey_F21},
    {"F22", ImGuiKey_F22},
    {"F23", ImGuiKey_F23},
    {"F24", ImGuiKey_F24},
    {"Num_Lock", ImGuiKey_NumLock},
    {"Scroll_Lock", ImGuiKey_ScrollLock},
};
