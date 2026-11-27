module;
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

export module rush.engine.input;

import rush.engine.core;
import rush.engine.events;

// Unified input: independent input thread + registration table + pure event
// bus. Modules register (event name, gesture, keys); the input thread samples
// devices, runs gesture recognizers, and Publish()es InputEvents. Consumers
// only Drain() on their own stage — there is no callback API.
//
// Gesture kinds and the events they emit:
//   Hold         -> <name>.begin / <name>.level / <name>.end   (throttle, brake)
//   PressRelease -> <name>.down / <name>.up                    (two edges)
//   Click        -> <name>.click                               (DRS, menus)
//   DoubleClick  -> <name>.dblclick
//   LongPress    -> <name>.longpress
export namespace apex {

// --- Device layer ------------------------------------------------------------

enum class KeyCode : uint16_t {
    Unknown = 0,
    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Space, Enter, Escape, Tab, Backspace,
    Up, Down, Left, Right,
    LShift, RShift, LCtrl, RCtrl, LAlt, RAlt,
    F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
    Count
};

enum class DeviceKind : uint8_t { Keyboard, Mouse, Gamepad };

struct DeviceKey {
    DeviceKind kind = DeviceKind::Keyboard;
    uint16_t code = 0;       // KeyCode / mouse button / gamepad button
    uint8_t deviceIndex = 0; // gamepad slot; 0 for keyboard/mouse

    bool operator==(const DeviceKey& o) const {
        return kind == o.kind && code == o.code && deviceIndex == o.deviceIndex;
    }
};

// One sampling tick produced by an IInputSource. Axes are generic: the
// registry binds them by index so keyboard keys can drive them too.
struct RawFrame {
    bool keys[static_cast<size_t>(KeyCode::Count)] = {};
    // Generic analog axes. Index convention (neutral, device-agnostic):
    // 0=throttle 1=brake 2=steer 3=look  — game maps these via bindings.
    float axes[8] = {};
    float mouseX = 0.f, mouseY = 0.f;
    float mouseWheel = 0.f;
    bool mouseButtons[3] = {};
    // Text typed this tick, as Unicode codepoints (raylib GetCharPressed).
    // Gesture recognizers ignore this channel; UI text fields consume it.
    std::uint32_t text[32] = {};
    std::uint8_t textCount = 0;
};

// Per-frame semantic snapshot the game builds from drained events + hold
// values. Same field set as the legacy InputState so tests and PlayerIntent
// migration stay mechanical. Lives in the input module as the "resolved
// action state" — it is NOT polled from devices; it is reconstructed from
// the event stream each frame.
struct InputState {
    float throttle = 0.f;      // 0..1
    float brake = 0.f;         // 0..1
    float steer = 0.f;         // -1..1, positive = steer right
    bool handbrake = false;
    bool reverse = false;
    bool drsToggle = false;
    bool wingModeToggle = false;
    bool overtake = false;
    bool cameraCycle = false;
    bool pause = false;
    bool restart = false;
    bool lookBack = false;

    bool menuUp = false;
    bool menuDown = false;
    bool menuLeft = false;
    bool menuRight = false;
    bool menuSelect = false;
    bool menuBack = false;

    // Typed text since the previous Consume (UTF-8 bytes, NUL-terminated by
    // textLen) and the one-frame backspace edge. Skip is a raw device key, not
    // a registered action: menus own it while a text field is focused, so it
    // is deliberately not routed through the gesture registry.
    char text[96] = {};
    std::uint8_t textLen = 0;
    bool textBackspace = false;
};

// --- Registration table ------------------------------------------------------

enum class Gesture : uint8_t {
    Hold,          // begin / level / end — continuous (throttle, brake, steer)
    PressRelease,  // down / up — two distinct events
    Click,         // click on quick tap
    DoubleClick,   // two clicks inside the window
    LongPress,     // held past the threshold
};

// One trigger for an action: either a physical key/button or a raw axis.
// axisIndex >= 0 wins over `key` (analog binding).
struct BindingSpec {
    DeviceKey key{};
    int axisIndex = -1;      // >=0: read RawFrame.axes[axisIndex]
    float axisScale = 1.f;   // multiplier / digital-key contribution
    bool invert = false;
};

struct Registration {
    std::string name;              // event base name, e.g. "throttle", "drs"
    Gesture gesture = Gesture::Click;
    std::vector<BindingSpec> keys; // one or more triggers (OR / max)
    float holdTickHz = 0.f;        // >0: emit .level at this rate while held
    float doubleClickWindowMs = 300.f;
    float longPressMs = 500.f;
    std::string context;           // "race" | "menu" | "" (always active)
};

// --- Events ------------------------------------------------------------------

struct InputEvent {
    uint32_t nameId = 0;   // interned id of the full event name (e.g. "throttle.level")
    Gesture gesture = Gesture::Click;
    float value = 0.f;     // Hold.level: axis value; edges: 1
    double tSec = 0.0;
    DeviceKey source{};
};

rush::EventBus<InputEvent>& InputBus();

uint32_t InternInputName(std::string_view name);
std::string_view InputNameOf(uint32_t nameId);

// --- Input source ------------------------------------------------------------

class IInputSource {
public:
    virtual ~IInputSource() = default;
    virtual void Poll(RawFrame& out) = 0;
};

// Headless / scripted source for tests and drive-test. Assign `scripted` to
// drive the recognizers; Poll() copies it out.
class NullInputSource final : public IInputSource {
public:
    RawFrame scripted;
    void Poll(RawFrame& out) override { out = scripted; }
};

// Raylib keyboard/mouse sampler. Requires the window to exist (RaylibBackend
// Init already ran). Compiled only when the host provides raylib.
class RaylibInputSource final : public IInputSource {
public:
    void Poll(RawFrame& out) override;
};

// --- Registry + System -------------------------------------------------------

class InputRegistry {
public:
    struct Entry {
        uint32_t nameId = 0;
        std::string name;
        Gesture gesture = Gesture::Click;
        std::vector<BindingSpec> keys;
        float holdTickHz = 0.f;
        float doubleClickWindowMs = 300.f;
        float longPressMs = 500.f;
        std::string context;
    };

    // Register (or replace same name+gesture). Returns interned base nameId.
    uint32_t Register(Registration reg);
    void Clear();
    const std::vector<Entry>& Entries() const { return entries_; }

    // Rebind from a Config section: for each registered entry, the key
    // `section + "." + name + ".keys"` (comma-separated key names, e.g.
    // "w,up") replaces that entry's triggers. Gesture, scales and context
    // stay as registered. Unknown keys are ignored; returns Err on an
    // entry whose config value contains only unrecognized names.
    rush::Result<void> ApplyOverrides(const apex::Config& config, const std::string& section);

private:
    std::vector<Entry> entries_;
};

// Runs gesture state machines over RawFrames and publishes InputEvents to
// InputBus(). Owns its thread in Start(); tests / WASM use PumpOnce().
class InputSystem {
public:
    explicit InputSystem(IInputSource& source);
    ~InputSystem();

    InputRegistry& Registry() { return registry_; }

    void Start();
    void Stop();
    bool Running() const;

    // Sample once + run recognizers + publish events (tests / WASM / thread).
    void PumpOnce();

    void SetActiveContext(std::string_view ctx);
    std::string ActiveContext() const;

    // Current continuous value of a Hold action (0 if unknown / released).
    float ActionValue(uint32_t nameId) const;
    float ActionValue(std::string_view name) const;

    // Drain InputBus and fold events + hold values into `out`.
    // One-frame flags (drsToggle, menuUp, ...) are set only by events that
    // landed since the previous Consume; hold axes are read from ActionValue.
    void Consume(InputState& out);

    double Now() const;

private:
    void ThreadMain();
    void RunRecognizers(const RawFrame& frame, double tSec);

    IInputSource& source_;
    InputRegistry registry_;

    struct RecState {
        bool held = false;
        float value = 0.f;
        double holdStart = 0.0;
        double lastLevel = 0.0;
        double lastRelease = -1e9;
        double pressStart = 0.0;
        bool longPressFired = false;
    };
    std::vector<RecState> states_;
    std::vector<std::pair<uint32_t, float>> actionMap_;

    std::string activeContext_;
    mutable std::mutex mutex_;
    // Text channel: collected on the input thread, drained by Consume.
    std::vector<std::uint32_t> pendingText_;
    bool pendingBackspace_ = false;
    bool backspacePrev_ = false;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
    std::chrono::steady_clock::time_point t0_;
    bool t0Set_ = false;
};

}  // namespace apex
