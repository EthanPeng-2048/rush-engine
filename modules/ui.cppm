module;
#include <cstdint>
#include <string>
#include <vector>

export module rush.engine.ui;

import rush.engine.core;
import rush.engine.render;
import rush.engine.input;

// Declarative control layer. The game describes WHAT a screen contains — rows,
// buttons, value fields, digit wheels, text fields — and which action id each
// row reports; the engine owns layout, control rendering and navigation.
// The game only receives resolved UiActions and dispatches them on its own
// state machine, so screens stay data in L3 and control behaviour lives here.
//
// Ownership:
//   * Game: screen content (labels/ids/values), reaction to UiAction.
//   * Engine: focus movement, digit-wheel and text editing, drawing, palette.
// Nothing in this module knows about races, rooms or settings.
export namespace apex {

enum class UiKind : std::uint8_t {
    Label,      // static text row, not focusable
    Header,     // section caption, not focusable
    Button,     // focusable; Enter -> Activate
    Value,      // "<label> <value>"; Left/Right -> AdjustLeft/Right
    Digits,     // N-digit wheel: Left/Right pick a digit, Up/Down change it
    TextField,  // editable single-line text
};

struct UiWidget {
    UiKind kind = UiKind::Label;
    std::string id;      // echoed back in UiAction::id (empty for static rows)
    std::string label;   // Button text / Value + Digits + TextField caption
    std::string value;   // Value display / Digits digits ("0000") / TextField buffer
    int size = 22;       // font size
    int digits = 4;      // Digits: digit count; TextField: max byte length
    bool enabled = true;
    bool focused = false;   // written by UiController
    int subCursor = 0;      // Digits: focused digit index
    bool customColor = false;
    Color color = Color::White();
};

struct UiScreen {
    std::string title;
    std::string hint;   // footer hint line ("" = none)
    std::vector<UiWidget> widgets;
    int cursor = 0;     // focused widget index (engine-owned)

    // Layout knobs; defaults match the existing menu look.
    int titleSize = 42;
    float x = 80.f;
    float y = 70.f;
    float rowStartY = 170.f;
    float rowHeight = 44.f;
    float valueColumnX = 340.f;  // Value rows: right-hand column offset from x
    bool dimBackground = true;
    // Up/Down (and Left/Right on a digit wheel) wrap around the focusable rows.
    // Screens that should stop at the ends (a short action list, a form) clear
    // this — the game sets a flag, it never re-implements the movement.
    bool wrapFocus = true;
};

enum class UiActionKind : std::uint8_t {
    None,
    Activate,     // Enter on Button / Digits / TextField
    AdjustLeft,   // Value -1, Digits: digit moved
    AdjustRight,  // Value +1, Digits: digit moved
    Back,         // Escape
    TextChanged,  // TextField buffer edited (read widget.value)
};

struct UiAction {
    UiActionKind kind = UiActionKind::None;
    std::string id;   // widget id ("" for Back/None)
    int delta = 0;    // Adjust*: widget step; Digits: the new digit (0-9)
};

class UiController {
public:
    // Move focus to the first focusable row. Call when a screen opens.
    static void ResetFocus(UiScreen& screen);

    // Resolve one frame of input over `screen`. Mutates focus, the digit
    // sub-cursor and text buffers in place; returns the single action the game
    // must handle this frame (at most one — input events are already edges).
    //
    // Rule for text screens: while a TextField is focused the field consumes
    // every navigation key (so W/A/S/D type instead of moving focus); Enter
    // reports Activate and Escape reports Back. Put a text field on its own
    // screen instead of mixing it with navigable rows.
    UiAction Update(UiScreen& screen, const InputState& in);

    // Draw the screen: background, title, rows, focus highlight, hint.
    void Draw(const UiScreen& screen, IRenderer& renderer) const;
};

}  // namespace apex
