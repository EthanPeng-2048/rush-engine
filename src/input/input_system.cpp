module;
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <expected>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

module rush.engine.input;
import rush.engine.core;
import rush.engine.events;

namespace apex {

namespace {

std::mutex g_nameMutex;
std::unordered_map<std::string, uint32_t> g_nameToId;
std::vector<std::string> g_idToName;

}  // namespace

rush::EventBus<InputEvent>& InputBus() {
    static rush::EventBus<InputEvent> bus;
    return bus;
}

uint32_t InternInputName(std::string_view name) {
    std::lock_guard lock(g_nameMutex);
    std::string key(name);
    auto it = g_nameToId.find(key);
    if (it != g_nameToId.end()) return it->second;
    const uint32_t id = static_cast<uint32_t>(g_idToName.size());
    g_idToName.push_back(key);
    g_nameToId.emplace(std::move(key), id);
    return id;
}

std::string_view InputNameOf(uint32_t nameId) {
    std::lock_guard lock(g_nameMutex);
    if (nameId >= g_idToName.size()) return {};
    return g_idToName[nameId];
}

// --- Registry ----------------------------------------------------------------

uint32_t InputRegistry::Register(Registration reg) {
    const uint32_t id = InternInputName(reg.name);
    for (auto& e : entries_) {
        if (e.nameId == id && e.gesture == reg.gesture) {
            e.keys = std::move(reg.keys);
            e.holdTickHz = reg.holdTickHz;
            e.doubleClickWindowMs = reg.doubleClickWindowMs;
            e.longPressMs = reg.longPressMs;
            e.context = std::move(reg.context);
            return id;
        }
    }
    Entry e;
    e.nameId = id;
    e.name = std::move(reg.name);
    e.gesture = reg.gesture;
    e.keys = std::move(reg.keys);
    e.holdTickHz = reg.holdTickHz;
    e.doubleClickWindowMs = reg.doubleClickWindowMs;
    e.longPressMs = reg.longPressMs;
    e.context = std::move(reg.context);
    entries_.push_back(std::move(e));
    return id;
}

void InputRegistry::Clear() { entries_.clear(); }

namespace {

// Key name -> KeyCode for config rebinds ("w", "space", "up", ...).
std::optional<KeyCode> ParseKeyName(std::string_view s) {
    auto eq = [&](std::string_view other) { return s == other; };
    // Letters.
    if (s.size() == 1) {
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(s[0])));
        if (c >= 'a' && c <= 'z') return static_cast<KeyCode>(static_cast<int>(KeyCode::A) + (c - 'a'));
        if (c >= '0' && c <= '9') return static_cast<KeyCode>(static_cast<int>(KeyCode::Num0) + (c - '0'));
        return std::nullopt;
    }
    if (eq("space")) return KeyCode::Space;
    if (eq("enter")) return KeyCode::Enter;
    if (eq("escape") || eq("esc")) return KeyCode::Escape;
    if (eq("tab")) return KeyCode::Tab;
    if (eq("backspace")) return KeyCode::Backspace;
    if (eq("up")) return KeyCode::Up;
    if (eq("down")) return KeyCode::Down;
    if (eq("left")) return KeyCode::Left;
    if (eq("right")) return KeyCode::Right;
    if (eq("lshift")) return KeyCode::LShift;
    if (eq("rshift")) return KeyCode::RShift;
    if (eq("lctrl")) return KeyCode::LCtrl;
    if (eq("rctrl")) return KeyCode::RCtrl;
    if (eq("lalt")) return KeyCode::LAlt;
    if (eq("ralt")) return KeyCode::RAlt;
    if (s.size() == 3 && (s[0] == 'f' || s[0] == 'F') && s[1] >= '1' && s[1] <= '9') {
        return static_cast<KeyCode>(static_cast<int>(KeyCode::F1) + (s[1] - '1'));
    }
    return std::nullopt;
}

}  // namespace

rush::Result<void> InputRegistry::ApplyOverrides(const Config& config, const std::string& section) {
    for (Entry& e : entries_) {
        const std::string key = section + "." + e.name + ".keys";
        const std::string raw = config.GetString(section, e.name + ".keys", "");
        if (raw.empty()) continue;

        std::vector<BindingSpec> parsed;
        std::string token;
        bool any = false;
        for (const char c : raw) {
            if (c == ',') {
                if (!token.empty()) {
                    if (auto kc = ParseKeyName(token)) {
                        BindingSpec b;
                        b.key.kind = DeviceKind::Keyboard;
                        b.key.code = static_cast<uint16_t>(*kc);
                        // Preserve the registered scale/index at the same slot
                        // (steer keeps A=-1/D=+1 semantics when rebound).
                        if (parsed.size() < e.keys.size()) {
                            b.axisScale = e.keys[parsed.size()].axisScale;
                            b.axisIndex = e.keys[parsed.size()].axisIndex;
                            b.invert = e.keys[parsed.size()].invert;
                        }
                        parsed.push_back(b);
                        any = true;
                    }
                }
                token.clear();
                continue;
            }
            token.push_back(c);
        }
        if (!token.empty()) {
            if (auto kc = ParseKeyName(token)) {
                BindingSpec b;
                b.key.kind = DeviceKind::Keyboard;
                b.key.code = static_cast<uint16_t>(*kc);
                if (parsed.size() < e.keys.size()) {
                    b.axisScale = e.keys[parsed.size()].axisScale;
                    b.axisIndex = e.keys[parsed.size()].axisIndex;
                    b.invert = e.keys[parsed.size()].invert;
                }
                parsed.push_back(b);
                any = true;
            }
        }
        if (!any) {
            return std::unexpected(rush::Error{"[input] " + e.name + ".keys: no recognized key name"});
        }
        e.keys = std::move(parsed);
    }
    return {};
}

// --- InputSystem -------------------------------------------------------------

InputSystem::InputSystem(IInputSource& source) : source_(source) {
    t0_ = std::chrono::steady_clock::now();
    t0Set_ = true;
}

InputSystem::~InputSystem() { Stop(); }

void InputSystem::Start() {
    if (running_.load()) return;
    stop_.store(false);
    running_.store(true);
    thread_ = std::thread([this] { ThreadMain(); });
}

void InputSystem::Stop() {
    if (!running_.load()) return;
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

bool InputSystem::Running() const { return running_.load(); }

double InputSystem::Now() const {
    using namespace std::chrono;
    return duration<double>(steady_clock::now() - t0_).count();
}

void InputSystem::SetActiveContext(std::string_view ctx) {
    std::lock_guard lock(mutex_);
    activeContext_ = std::string(ctx);
}

std::string InputSystem::ActiveContext() const {
    std::lock_guard lock(mutex_);
    return activeContext_;
}

float InputSystem::ActionValue(uint32_t nameId) const {
    std::lock_guard lock(mutex_);
    for (const auto& [id, v] : actionMap_) {
        if (id == nameId) return v;
    }
    return 0.f;
}

float InputSystem::ActionValue(std::string_view name) const {
    return ActionValue(InternInputName(name));
}

void InputSystem::ThreadMain() {
    using namespace std::chrono;
    while (!stop_.load()) {
        const auto tick = steady_clock::now();
        PumpOnce();
        std::this_thread::sleep_until(tick + milliseconds(5));
    }
}

void InputSystem::PumpOnce() {
    RawFrame frame;
    source_.Poll(frame);
    if (!t0Set_) {
        t0_ = std::chrono::steady_clock::now();
        t0Set_ = true;
    }
    RunRecognizers(frame, Now());
}

namespace {

bool BindingHeld(const BindingSpec& b, const RawFrame& f) {
    if (b.axisIndex >= 0 && b.axisIndex < 8) {
        return std::fabs(f.axes[b.axisIndex]) > 1e-4f;
    }
    if (b.key.kind != DeviceKind::Keyboard) return false;
    const size_t idx = static_cast<size_t>(b.key.code);
    return idx < static_cast<size_t>(KeyCode::Count) && f.keys[idx];
}

float BindingValue(const BindingSpec& b, const RawFrame& f) {
    float v = 0.f;
    if (b.axisIndex >= 0 && b.axisIndex < 8) {
        v = f.axes[b.axisIndex];
    } else if (b.key.kind == DeviceKind::Keyboard) {
        const size_t idx = static_cast<size_t>(b.key.code);
        if (idx < static_cast<size_t>(KeyCode::Count) && f.keys[idx]) v = 1.f;
    }
    v *= b.axisScale;
    if (b.invert) v = -v;
    return v;
}

bool EntryHeld(const InputRegistry::Entry& e, const RawFrame& f) {
    for (const BindingSpec& b : e.keys) {
        if (BindingHeld(b, f)) return true;
    }
    return false;
}

// For Hold: signed value from the binding with the largest magnitude.
float EntryValue(const InputRegistry::Entry& e, const RawFrame& f) {
    float best = 0.f;
    for (const BindingSpec& b : e.keys) {
        const float v = BindingValue(b, f);
        if (std::fabs(v) > std::fabs(best)) best = v;
    }
    return best;
}

}  // namespace

void InputSystem::RunRecognizers(const RawFrame& frame, double tSec) {
    // Text channel first: typed codepoints and the backspace edge are raw
    // device state (not registered actions), so they are collected here and
    // drained by Consume on the caller's thread.
    {
        std::lock_guard lock(mutex_);
        for (std::uint8_t i = 0; i < frame.textCount; ++i) pendingText_.push_back(frame.text[i]);
        if (pendingText_.size() > 256) {
            pendingText_.erase(pendingText_.begin(),
                               pendingText_.begin() + static_cast<std::ptrdiff_t>(pendingText_.size() - 256));
        }
        const bool backspace = frame.keys[static_cast<size_t>(KeyCode::Backspace)];
        if (backspace && !backspacePrev_) pendingBackspace_ = true;
        backspacePrev_ = backspace;
    }

    std::vector<InputRegistry::Entry> entries;
    std::string ctx;
    {
        std::lock_guard lock(mutex_);
        entries = registry_.Entries();
        ctx = activeContext_;
    }
    if (states_.size() != entries.size()) states_.resize(entries.size());

    std::vector<InputEvent> batch;
    batch.reserve(8);

    auto emit = [&](const InputRegistry::Entry& e, const char* suffix, float value,
                    const DeviceKey& src) {
        InputEvent ev;
        ev.nameId = InternInputName(e.name + suffix);
        ev.gesture = e.gesture;
        ev.value = value;
        ev.tSec = tSec;
        ev.source = src;
        batch.push_back(ev);
    };

    for (size_t i = 0; i < entries.size(); ++i) {
        const InputRegistry::Entry& e = entries[i];
        RecState& st = states_[i];

        if (!e.context.empty() && e.context != ctx) {
            if (st.held) {
                st.held = false;
                if (e.gesture == Gesture::Hold) emit(e, ".end", 0.f, {});
            }
            st.value = 0.f;
            continue;
        }

        const bool held = EntryHeld(e, frame);
        const float value =
            (e.gesture == Gesture::Hold) ? EntryValue(e, frame) : (held ? 1.f : 0.f);
        DeviceKey src{};
        if (!e.keys.empty()) src = e.keys.front().key;

        switch (e.gesture) {
            case Gesture::Hold: {
                if (held && !st.held) {
                    st.held = true;
                    st.holdStart = tSec;
                    st.lastLevel = tSec;
                    emit(e, ".begin", value, src);
                    emit(e, ".level", value, src);
                } else if (held && st.held) {
                    const bool tickDue =
                        e.holdTickHz <= 0.f ||
                        (tSec - st.lastLevel) >= (1.0 / static_cast<double>(e.holdTickHz));
                    if (tickDue) {
                        st.lastLevel = tSec;
                        emit(e, ".level", value, src);
                    }
                } else if (!held && st.held) {
                    st.held = false;
                    emit(e, ".end", 0.f, src);
                }
                st.value = held ? value : 0.f;
                break;
            }
            case Gesture::PressRelease: {
                if (held && !st.held) {
                    st.held = true;
                    emit(e, ".down", 1.f, src);
                } else if (!held && st.held) {
                    st.held = false;
                    emit(e, ".up", 1.f, src);
                }
                break;
            }
            case Gesture::Click:
            case Gesture::DoubleClick:
            case Gesture::LongPress: {
                const double win = static_cast<double>(e.doubleClickWindowMs) / 1000.0;
                const double lp = static_cast<double>(e.longPressMs) / 1000.0;
                if (held && !st.held) {
                    st.held = true;
                    st.pressStart = tSec;
                    st.longPressFired = false;
                    // Click fires on the press edge (matches IsKeyPressed):
                    // a single scripted frame with the flag set must already
                    // toggle the action.
                    if (e.gesture == Gesture::DoubleClick &&
                        (tSec - st.lastRelease) <= win) {
                        st.lastRelease = -1e9;
                        emit(e, ".dblclick", 1.f, src);
                    } else if (e.gesture == Gesture::Click ||
                               e.gesture == Gesture::DoubleClick) {
                        st.lastRelease = tSec;
                        emit(e, ".click", 1.f, src);
                    }
                } else if (held && st.held) {
                    if (e.gesture == Gesture::LongPress && !st.longPressFired &&
                        (tSec - st.pressStart) >= lp) {
                        st.longPressFired = true;
                        emit(e, ".longpress", 1.f, src);
                    }
                } else if (!held && st.held) {
                    st.held = false;
                }
                break;
            }
        }
    }

    for (const InputEvent& ev : batch) InputBus().Publish(ev);

    {
        std::lock_guard lock(mutex_);
        actionMap_.clear();
        for (size_t i = 0; i < entries.size(); ++i) {
            if (entries[i].gesture != Gesture::Hold) continue;
            actionMap_.emplace_back(entries[i].nameId, states_[i].value);
        }
    }
}

void InputSystem::Consume(InputState& out) {
    // Continuous holds come from the action map (last recognized value).
    out.throttle = Clampf(ActionValue("throttle"), 0.f, 1.f);
    out.brake = Clampf(ActionValue("brake"), 0.f, 1.f);
    out.steer = Clampf(ActionValue("steer"), -1.f, 1.f);
    out.handbrake = ActionValue("handbrake") > 0.5f;
    out.reverse = ActionValue("reverse") > 0.5f;
    out.overtake = ActionValue("overtake") > 0.5f;
    out.lookBack = ActionValue("lookback") > 0.5f;

    // One-shot edges: fold every event since the last Consume.
    out.drsToggle = false;
    out.wingModeToggle = false;
    out.cameraCycle = false;
    out.pause = false;
    out.restart = false;
    out.menuUp = false;
    out.menuDown = false;
    out.menuLeft = false;
    out.menuRight = false;
    out.menuSelect = false;
    out.menuBack = false;
    out.textLen = 0;
    out.textBackspace = false;

    // Typed text: encode the codepoints collected on the input thread into the
    // UTF-8 buffer (dropped when it does not fit — a name field caps far
    // earlier, so this is only a safety net).
    {
        std::lock_guard lock(mutex_);
        for (const std::uint32_t cp : pendingText_) {
            char utf8[4];
            int len = 0;
            if (cp < 0x80u) {
                utf8[len++] = static_cast<char>(cp);
            } else if (cp < 0x800u) {
                utf8[len++] = static_cast<char>(0xC0 | (cp >> 6));
                utf8[len++] = static_cast<char>(0x80 | (cp & 0x3Fu));
            } else if (cp < 0x10000u) {
                utf8[len++] = static_cast<char>(0xE0 | (cp >> 12));
                utf8[len++] = static_cast<char>(0x80 | ((cp >> 6) & 0x3Fu));
                utf8[len++] = static_cast<char>(0x80 | (cp & 0x3Fu));
            } else if (cp <= 0x10FFFFu) {
                utf8[len++] = static_cast<char>(0xF0 | (cp >> 18));
                utf8[len++] = static_cast<char>(0x80 | ((cp >> 12) & 0x3Fu));
                utf8[len++] = static_cast<char>(0x80 | ((cp >> 6) & 0x3Fu));
                utf8[len++] = static_cast<char>(0x80 | (cp & 0x3Fu));
            }
            if (len == 0) continue;
            if (static_cast<int>(out.textLen) + len > static_cast<int>(sizeof(out.text)) - 1) break;
            for (int i = 0; i < len; ++i) out.text[out.textLen++] = utf8[i];
        }
        out.text[out.textLen] = '\0';
        pendingText_.clear();
        out.textBackspace = pendingBackspace_;
        pendingBackspace_ = false;
    }

    InputBus().Drain([&](const InputEvent& ev) {
        const std::string_view name = InputNameOf(ev.nameId);
        // Accept "foo", "foo.click", "foo.down", "foo.dblclick" or
        // "foo.longpress" as the press edge for the same action base.
        // Must verify the FULL base prefix: menu.up / menu.down / menu.back
        // share the "menu." head, so a suffix-only check wrongly matched
        // "menu.back.click" against "menu.down" (index 10 == '.').
        auto match = [&](std::string_view base) {
            if (name.size() < base.size()) return false;
            if (name.substr(0, base.size()) != base) return false;
            if (name.size() == base.size()) return true;
            if (name[base.size()] != '.') return false;
            const std::string_view suffix = name.substr(base.size() + 1);
            return suffix == "click" || suffix == "down" || suffix == "dblclick" ||
                   suffix == "longpress";
        };
        if (match("drs")) out.drsToggle = true;
        else if (match("wing")) out.wingModeToggle = true;
        else if (match("camera")) out.cameraCycle = true;
        else if (match("pause")) out.pause = true;
        else if (match("restart")) out.restart = true;
        else if (match("menu.up")) out.menuUp = true;
        else if (match("menu.down")) out.menuDown = true;
        else if (match("menu.left")) out.menuLeft = true;
        else if (match("menu.right")) out.menuRight = true;
        else if (match("menu.select")) out.menuSelect = true;
        else if (match("menu.back")) out.menuBack = true;
    });
}

}  // namespace apex
