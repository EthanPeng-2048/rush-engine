module;
#include <algorithm>
#include <cstddef>
#include <string>

module rush.engine.ui;

import rush.engine.core;
import rush.engine.render;
import rush.engine.input;

namespace apex {
namespace {

constexpr Color kFocus{0.55f, 0.85f, 1.f, 1.f};
constexpr Color kRow{0.60f, 0.60f, 0.62f, 1.f};
constexpr Color kDisabled{0.32f, 0.32f, 0.34f, 1.f};
constexpr Color kFieldBg{0.14f, 0.15f, 0.18f, 1.f};
constexpr Color kFieldFocus{0.25f, 0.45f, 0.6f, 1.f};
constexpr Color kPanel{0.f, 0.f, 0.f, 0.55f};

bool IsFocusable(UiKind kind) {
    return kind == UiKind::Button || kind == UiKind::Value || kind == UiKind::Digits ||
           kind == UiKind::TextField;
}

const UiWidget* Focused(const UiScreen& screen) {
    if (screen.cursor < 0 || screen.cursor >= static_cast<int>(screen.widgets.size())) return nullptr;
    return &screen.widgets[static_cast<std::size_t>(screen.cursor)];
}

// Step the focus over focusable rows in `dir` (+1/-1), stopping at the ends
// (menus do not wrap: a wrap on the first Up press is how people lose a row).
void MoveFocus(UiScreen& screen, int dir) {
    const int count = static_cast<int>(screen.widgets.size());
    for (int i = screen.cursor + dir; i >= 0 && i < count; i += dir) {
        if (IsFocusable(screen.widgets[static_cast<std::size_t>(i)].kind) &&
            screen.widgets[static_cast<std::size_t>(i)].enabled) {
            screen.cursor = i;
            return;
        }
    }
}

void AppendText(UiWidget& widget, const char* text, int len) {
    const std::size_t maxLen =
        widget.digits > 0 ? static_cast<std::size_t>(widget.digits) : static_cast<std::size_t>(16);
    for (int i = 0; i < len; ++i) {
        const char c = text[i];
        // Printable ASCII only: the engine has no font for anything else and a
        // silently dropped glyph is worse than a rejected keystroke.
        if (c < 32 || c > 126) continue;
        if (widget.value.size() >= maxLen) break;
        widget.value.push_back(c);
    }
}

}  // namespace

void UiController::ResetFocus(UiScreen& screen) {
    screen.cursor = 0;
    for (int i = 0; i < static_cast<int>(screen.widgets.size()); ++i) {
        if (IsFocusable(screen.widgets[static_cast<std::size_t>(i)].kind) &&
            screen.widgets[static_cast<std::size_t>(i)].enabled) {
            screen.cursor = i;
            break;
        }
    }
    for (UiWidget& widget : screen.widgets) widget.focused = false;
    if (const int c = screen.cursor; c >= 0 && c < static_cast<int>(screen.widgets.size())) {
        screen.widgets[static_cast<std::size_t>(c)].focused = true;
    }
}

UiAction UiController::Update(UiScreen& screen, const InputState& in) {
    UiAction action;
    if (screen.widgets.empty()) return action;

    const UiWidget* focused = Focused(screen);
    if (!focused || !IsFocusable(focused->kind)) ResetFocus(screen);

    for (UiWidget& widget : screen.widgets) widget.focused = false;
    UiWidget& current = screen.widgets[static_cast<std::size_t>(screen.cursor)];
    current.focused = true;

    if (in.menuBack) {
        action.kind = UiActionKind::Back;
        return action;
    }

    switch (current.kind) {
        case UiKind::TextField: {
            bool edited = false;
            if (in.textLen > 0) {
                AppendText(current, in.text, in.textLen);
                edited = true;
            }
            if (in.textBackspace && !current.value.empty()) {
                current.value.pop_back();
                edited = true;
            }
            if (edited) {
                action.kind = UiActionKind::TextChanged;
                action.id = current.id;
                return action;   // typing wins the frame; Enter can wait one frame
            }
            if (in.menuSelect) {
                action.kind = UiActionKind::Activate;
                action.id = current.id;
            }
            // Navigation keys are part of the text, not focus movement.
            return action;
        }
        case UiKind::Digits: {
            const int digitCount = std::max(1, current.digits);
            if (in.menuLeft) current.subCursor = (current.subCursor + digitCount - 1) % digitCount;
            if (in.menuRight) current.subCursor = (current.subCursor + 1) % digitCount;
            if (in.menuUp || in.menuDown) {
                if (current.value.size() < static_cast<std::size_t>(digitCount)) {
                    current.value.resize(static_cast<std::size_t>(digitCount), '0');
                }
                char& digit = current.value[static_cast<std::size_t>(current.subCursor)];
                const int step = in.menuUp ? 1 : 9;   // wrap 0..9
                digit = static_cast<char>('0' + ((digit - '0' + step) % 10));
                action.kind = in.menuUp ? UiActionKind::AdjustRight : UiActionKind::AdjustLeft;
                action.id = current.id;
                action.delta = digit - '0';
                return action;
            }
            if (in.menuSelect) {
                action.kind = UiActionKind::Activate;
                action.id = current.id;
            }
            return action;
        }
        case UiKind::Value: {
            if (in.menuLeft || in.menuRight) {
                action.kind = in.menuLeft ? UiActionKind::AdjustLeft : UiActionKind::AdjustRight;
                action.id = current.id;
                action.delta = in.menuLeft ? -1 : 1;
                return action;
            }
            break;
        }
        case UiKind::Button: {
            if (in.menuSelect) {
                action.kind = UiActionKind::Activate;
                action.id = current.id;
                return action;
            }
            break;
        }
        default:
            break;
    }

    if (in.menuUp) MoveFocus(screen, -1);
    if (in.menuDown) MoveFocus(screen, +1);
    if (in.menuUp || in.menuDown) {
        for (UiWidget& widget : screen.widgets) widget.focused = false;
        screen.widgets[static_cast<std::size_t>(screen.cursor)].focused = true;
    }
    return action;
}

void UiController::Draw(const UiScreen& screen, IRenderer& renderer) const {
    const float w = static_cast<float>(renderer.ScreenWidth());
    const float h = static_cast<float>(renderer.ScreenHeight());
    if (screen.dimBackground) renderer.DrawRect2D(0.f, 0.f, w, h, kPanel);
    if (!screen.title.empty()) {
        renderer.DrawText2D(screen.title, screen.x, screen.y, screen.titleSize, Color::White());
    }

    float y = screen.rowStartY;
    for (const UiWidget& widget : screen.widgets) {
        const Color base = widget.customColor
                               ? widget.color
                               : (widget.enabled ? kRow : kDisabled);
        const Color color = widget.focused && widget.enabled ? kFocus : base;
        switch (widget.kind) {
            case UiKind::Header:
                renderer.DrawText2D(widget.label, screen.x, y, widget.size, base);
                break;
            case UiKind::Label:
                renderer.DrawText2D(widget.label, screen.x, y, widget.size, color);
                break;
            case UiKind::Button:
                renderer.DrawText2D((widget.focused ? "> " : "  ") + widget.label, screen.x, y,
                                    widget.size, color);
                break;
            case UiKind::Value:
                renderer.DrawText2D((widget.focused ? "> " : "  ") + widget.label, screen.x, y,
                                    widget.size, color);
                renderer.DrawText2D(widget.value, screen.x + screen.valueColumnX, y, widget.size,
                                    widget.focused ? kFocus : Color::White());
                break;
            case UiKind::Digits: {
                renderer.DrawText2D(widget.label, screen.x, y, widget.size, color);
                const float boxY = y + widget.size + 8.f;
                for (int i = 0; i < std::max(1, widget.digits); ++i) {
                    const bool sel = widget.focused && i == widget.subCursor;
                    const bool has = i < static_cast<int>(widget.value.size());
                    const char digit = has ? widget.value[static_cast<std::size_t>(i)] : '0';
                    renderer.DrawRect2D(screen.x + 64.f * i, boxY, 52.f, 60.f,
                                        sel ? kFieldFocus : kFieldBg);
                    const std::string one(1, digit);
                    renderer.DrawText2D(one, screen.x + 16.f + 64.f * i, boxY + 10.f,
                                        widget.size + 12, Color::White());
                }
                break;
            }
            case UiKind::TextField: {
                renderer.DrawText2D(widget.label, screen.x, y, widget.size, color);
                const float boxY = y + widget.size + 8.f;
                const float boxW = std::max(220.f, 18.f * static_cast<float>(widget.digits));
                renderer.DrawRect2D(screen.x, boxY, boxW, 44.f,
                                    widget.focused ? kFieldFocus : kFieldBg);
                renderer.DrawText2D(widget.value + (widget.focused ? "_" : ""),
                                    screen.x + 12.f, boxY + 10.f, widget.size, Color::White());
                break;
            }
        }
        y += screen.rowHeight;
    }

    if (!screen.hint.empty()) {
        renderer.DrawText2D(screen.hint, screen.x, h - 60.f, 18, Color::DarkGray());
    }
}

}  // namespace apex
