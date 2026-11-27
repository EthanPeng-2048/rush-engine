module;
#include <cstdint>
#include <cstring>
#include "raylib.h"

module rush.engine.input;

namespace apex {

namespace {

// Map our neutral KeyCode to raylib's KEY_* so the source never leaks raylib
// types into the module interface.
int ToRaylibKey(KeyCode k) {
    switch (k) {
        case KeyCode::A: return KEY_A;
        case KeyCode::B: return KEY_B;
        case KeyCode::C: return KEY_C;
        case KeyCode::D: return KEY_D;
        case KeyCode::E: return KEY_E;
        case KeyCode::F: return KEY_F;
        case KeyCode::G: return KEY_G;
        case KeyCode::H: return KEY_H;
        case KeyCode::I: return KEY_I;
        case KeyCode::J: return KEY_J;
        case KeyCode::K: return KEY_K;
        case KeyCode::L: return KEY_L;
        case KeyCode::M: return KEY_M;
        case KeyCode::N: return KEY_N;
        case KeyCode::O: return KEY_O;
        case KeyCode::P: return KEY_P;
        case KeyCode::Q: return KEY_Q;
        case KeyCode::R: return KEY_R;
        case KeyCode::S: return KEY_S;
        case KeyCode::T: return KEY_T;
        case KeyCode::U: return KEY_U;
        case KeyCode::V: return KEY_V;
        case KeyCode::W: return KEY_W;
        case KeyCode::X: return KEY_X;
        case KeyCode::Y: return KEY_Y;
        case KeyCode::Z: return KEY_Z;
        case KeyCode::Num0: return KEY_ZERO;
        case KeyCode::Num1: return KEY_ONE;
        case KeyCode::Num2: return KEY_TWO;
        case KeyCode::Num3: return KEY_THREE;
        case KeyCode::Num4: return KEY_FOUR;
        case KeyCode::Num5: return KEY_FIVE;
        case KeyCode::Num6: return KEY_SIX;
        case KeyCode::Num7: return KEY_SEVEN;
        case KeyCode::Num8: return KEY_EIGHT;
        case KeyCode::Num9: return KEY_NINE;
        case KeyCode::Space: return KEY_SPACE;
        case KeyCode::Enter: return KEY_ENTER;
        case KeyCode::Escape: return KEY_ESCAPE;
        case KeyCode::Tab: return KEY_TAB;
        case KeyCode::Backspace: return KEY_BACKSPACE;
        case KeyCode::Up: return KEY_UP;
        case KeyCode::Down: return KEY_DOWN;
        case KeyCode::Left: return KEY_LEFT;
        case KeyCode::Right: return KEY_RIGHT;
        case KeyCode::LShift: return KEY_LEFT_SHIFT;
        case KeyCode::RShift: return KEY_RIGHT_SHIFT;
        case KeyCode::LCtrl: return KEY_LEFT_CONTROL;
        case KeyCode::RCtrl: return KEY_RIGHT_CONTROL;
        case KeyCode::LAlt: return KEY_LEFT_ALT;
        case KeyCode::RAlt: return KEY_RIGHT_ALT;
        case KeyCode::F1: return KEY_F1;
        case KeyCode::F2: return KEY_F2;
        case KeyCode::F3: return KEY_F3;
        case KeyCode::F4: return KEY_F4;
        case KeyCode::F5: return KEY_F5;
        case KeyCode::F6: return KEY_F6;
        case KeyCode::F7: return KEY_F7;
        case KeyCode::F8: return KEY_F8;
        case KeyCode::F9: return KEY_F9;
        case KeyCode::F10: return KEY_F10;
        case KeyCode::F11: return KEY_F11;
        case KeyCode::F12: return KEY_F12;
        default: return 0;
    }
}

}  // namespace

void RaylibInputSource::Poll(RawFrame& out) {
    std::memset(out.keys, 0, sizeof(out.keys));
    for (int k = 1; k < static_cast<int>(KeyCode::Count); ++k) {
        const int rk = ToRaylibKey(static_cast<KeyCode>(k));
        if (rk != 0 && ::IsKeyDown(rk)) out.keys[k] = true;
    }
    out.mouseX = static_cast<float>(::GetMouseX());
    out.mouseY = static_cast<float>(::GetMouseY());
    out.mouseWheel = ::GetMouseWheelMove();
    out.mouseButtons[0] = ::IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    out.mouseButtons[1] = ::IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
    out.mouseButtons[2] = ::IsMouseButtonDown(MOUSE_BUTTON_MIDDLE);

    // Text channel: raylib queues codepoints for the current frame. Drain the
    // queue so a burst smaller than the buffer is never half-consumed.
    out.textCount = 0;
    while (out.textCount < sizeof(out.text) / sizeof(out.text[0])) {
        const int cp = ::GetCharPressed();
        if (cp <= 0) break;
        out.text[out.textCount++] = static_cast<std::uint32_t>(cp);
    }
}

}  // namespace apex
