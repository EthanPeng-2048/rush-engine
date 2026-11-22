module;
#include <algorithm>
#include <unordered_map>
#include <string>
#include <vector>
#include <cstdint>
#include "raylib.h"

module rush.engine.render;
import rush.engine.core;

namespace apex {

namespace {

::Color ToRaylib(const Color& c) {
    return {static_cast<unsigned char>(Clampf(c.r, 0.f, 1.f) * 255.f),
            static_cast<unsigned char>(Clampf(c.g, 0.f, 1.f) * 255.f),
            static_cast<unsigned char>(Clampf(c.b, 0.f, 1.f) * 255.f),
            static_cast<unsigned char>(Clampf(c.a, 0.f, 1.f) * 255.f)};
}

Vector3 ToRaylib(const Vec3& v) { return {v.x, v.y, v.z}; }

// Minimal Win32 query (avoids including windows.h): raylib's InitWindow aborts
// when the requested window does not fit any monitor, so clamp to the screen.
#ifdef _WIN32
extern "C" __declspec(dllimport) int __stdcall GetSystemMetrics(int nIndex);
constexpr int kSmCxScreen = 0;
constexpr int kSmCyScreen = 1;
// IME (Chinese/Japanese input methods) swallows game keys between keydown and
// keyup ("卡键"). Disable it for the game thread so keystrokes go straight
// through — the game takes the input system back.
extern "C" __declspec(dllimport) int __stdcall ImmDisableIME(unsigned long dwThreadId);
#pragma comment(lib, "imm32.lib")
#endif

// Clamp a windowed size to the visible screen: raylib's InitWindow terminates
// GLFW when the window does not fit any monitor ("Failed to determine Monitor
// to center").
void ClampToScreen(int& width, int& height) {
#ifdef _WIN32
    width = std::min(width, GetSystemMetrics(kSmCxScreen) - 40);
    height = std::min(height, GetSystemMetrics(kSmCyScreen) - 90);  // title bar + taskbar
#endif
}

}  // namespace

struct RaylibBackend::Impl {
    ::Camera3D camera{};
    std::unordered_map<MeshHandle, ::Mesh> meshes;
    std::unordered_map<MeshHandle, ::Model> models;
    MeshHandle nextMesh = 1;
    bool shadows = true;
};

RaylibBackend::~RaylibBackend() {
    Shutdown();
}

bool RaylibBackend::Init(int width, int height, const std::string& title) {
    if (impl_) return true;
    SetTraceLogLevel(LOG_WARNING);
    ClampToScreen(width, height);
    InitWindow(width, height, title.c_str());
    if (!IsWindowReady()) return false;
    // raylib closes the window on ESC by default, which hijacked the game's
    // pause/back-to-menu handling ("ESC quits instead of pausing"). Hand ESC
    // to the game instead.
    SetExitKey(KEY_NULL);
#ifdef _WIN32
    ImmDisableIME(0xFFFFFFFFul);  // current thread
#endif
#if defined(__EMSCRIPTEN__)
    // Web builds run the blocking loop through ASYNCIFY and must yield to the
    // browser once per frame — raylib does that inside WaitTime at the target
    // rate, so an uncapped loop would freeze the page.
    SetTargetFPS(60);
#else
    SetTargetFPS(0);  // uncapped; fixed-step physics is decoupled from render
#endif
    impl_ = new Impl();
    impl_->camera.position = {0.f, 8.f, -12.f};
    impl_->camera.target = {0.f, 0.f, 0.f};
    impl_->camera.up = {0.f, 1.f, 0.f};
    impl_->camera.fovy = 50.f;
    impl_->camera.projection = CAMERA_PERSPECTIVE;
    return true;
}

void RaylibBackend::Shutdown() {
    if (!impl_) return;
    for (auto& [handle, model] : impl_->models) UnloadModel(model);
    impl_->models.clear();
    // Meshes owned by models are freed by UnloadModel; standalone meshes that
    // never became models must be freed explicitly.
    for (auto& [handle, mesh] : impl_->meshes) {
        if (impl_->models.find(handle) == impl_->models.end()) UnloadMesh(mesh);
    }
    impl_->meshes.clear();
    delete impl_;
    impl_ = nullptr;
    if (IsWindowReady()) CloseWindow();
}

bool RaylibBackend::ShouldClose() const {
    return ::WindowShouldClose();
}

void RaylibBackend::BeginFrame() {
    ::BeginDrawing();
    // Sky blue so the grass/track read like an outdoor circuit.
    ::ClearBackground({132, 186, 230, 255});
}

void RaylibBackend::BeginScene() {
    ::BeginMode3D(impl_->camera);
}

void RaylibBackend::EndScene() {
    ::EndMode3D();
}

void RaylibBackend::EndFrame() {
    ::EndDrawing();
}

void RaylibBackend::SetCamera(const CameraDesc& camera) {
    impl_->camera.position = ToRaylib(camera.position);
    impl_->camera.target = ToRaylib(camera.target);
    impl_->camera.up = ToRaylib(camera.up);
    impl_->camera.fovy = camera.fovYDeg;
}

int RaylibBackend::ScreenWidth() const { return ::GetScreenWidth(); }
int RaylibBackend::ScreenHeight() const { return ::GetScreenHeight(); }

void RaylibBackend::ApplyVideo(const VideoConfig& video) {
    if (!impl_) return;
    impl_->shadows = video.shadows;
#if defined(__EMSCRIPTEN__)
    // Web builds must yield to the browser through WaitTime at a bounded rate
    // (ASYNCIFY), so the FPS cap is fixed and window mode is the canvas's.
    (void)video;
    SetTargetFPS(60);
#else
    SetTargetFPS(video.fpsLimit);
    const bool isBorderless = IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE);
    if (video.borderlessFullscreen != isBorderless) ToggleBorderlessWindowed();
    if (!video.borderlessFullscreen) {
        int width = video.width;
        int height = video.height;
        ClampToScreen(width, height);
        SetWindowSize(width, height);
    }
#endif
}

bool RaylibBackend::ShadowsEnabled() const { return impl_ && impl_->shadows; }

MeshHandle RaylibBackend::CreateMesh(const MeshData& data) {
    ::Mesh mesh{};
    mesh.vertexCount = static_cast<int>(data.positions.size());
    mesh.triangleCount = static_cast<int>(data.indices.size() / 3);

    mesh.vertices = static_cast<float*>(MemAlloc(mesh.vertexCount * 3 * sizeof(float)));
    for (int i = 0; i < mesh.vertexCount; ++i) {
        mesh.vertices[i * 3 + 0] = data.positions[i].x;
        mesh.vertices[i * 3 + 1] = data.positions[i].y;
        mesh.vertices[i * 3 + 2] = data.positions[i].z;
    }

    if (!data.normals.empty()) {
        mesh.normals = static_cast<float*>(MemAlloc(mesh.vertexCount * 3 * sizeof(float)));
        for (int i = 0; i < mesh.vertexCount; ++i) {
            mesh.normals[i * 3 + 0] = data.normals[i].x;
            mesh.normals[i * 3 + 1] = data.normals[i].y;
            mesh.normals[i * 3 + 2] = data.normals[i].z;
        }
    }

    if (!data.uvs.empty()) {
        mesh.texcoords = static_cast<float*>(MemAlloc(mesh.vertexCount * 2 * sizeof(float)));
        for (int i = 0; i < mesh.vertexCount * 2; ++i) mesh.texcoords[i] = data.uvs[i];
    }

    if (!data.colors.empty()) {
        mesh.colors = static_cast<unsigned char*>(MemAlloc(mesh.vertexCount * 4 * sizeof(unsigned char)));
        for (int i = 0; i < mesh.vertexCount * 4; ++i) mesh.colors[i] = data.colors[i];
    }

    mesh.indices = static_cast<unsigned short*>(MemAlloc(data.indices.size() * sizeof(unsigned short)));
    for (size_t i = 0; i < data.indices.size(); ++i) {
        mesh.indices[i] = static_cast<unsigned short>(data.indices[i]);
    }

    UploadMesh(&mesh, false);

    ::Model model = LoadModelFromMesh(mesh);
    const MeshHandle handle = impl_->nextMesh++;
    impl_->models[handle] = model;
    return handle;
}

void RaylibBackend::DestroyMesh(MeshHandle handle) {
    const auto it = impl_->models.find(handle);
    if (it == impl_->models.end()) return;
    UnloadModel(it->second);
    impl_->models.erase(it);
}

void RaylibBackend::DrawMesh(MeshHandle handle, const Transform& transform, const Color& tint) {
    const auto it = impl_->models.find(handle);
    if (it == impl_->models.end()) return;
    ::Model& model = it->second;

    // Rebuild the model matrix from our transform each draw. raylib's Matrix
    // aggregate order is the matrix written row by row (translation in the
    // 4th slot of the first three rows) — TransformToMatrix matches it.
    float m[16];
    TransformToMatrix(transform, m);
    model.transform = {m[0], m[1], m[2],  m[3],
                       m[4], m[5], m[6],  m[7],
                       m[8], m[9], m[10], m[11],
                       m[12], m[13], m[14], m[15]};
    DrawModel(model, {0.f, 0.f, 0.f}, 1.f, ToRaylib(tint));
    // Restore identity so the next frame's transform starts clean.
    model.transform = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                       0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
}

void RaylibBackend::DrawMeshShadow(MeshHandle handle, const Transform& transform, float groundY,
                                   const Color& tint) {
    const auto it = impl_->models.find(handle);
    if (it == impl_->models.end()) return;
    ::Model& model = it->second;

    // Squash Y about the ground plane: the mesh collapses into its top-down
    // footprint sitting just above groundY (see TransformToMatrix row layout).
    float m[16];
    TransformToMatrix(transform, m);
    m[4] *= 0.01f;
    m[5] *= 0.01f;
    m[6] *= 0.01f;
    m[7] = m[7] * 0.01f + groundY * 0.99f + 0.02f;

    model.transform = {m[0], m[1], m[2],  m[3],
                       m[4], m[5], m[6],  m[7],
                       m[8], m[9], m[10], m[11],
                       m[12], m[13], m[14], m[15]};
    DrawModel(model, {0.f, 0.f, 0.f}, 1.f, ToRaylib(tint));
    model.transform = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                       0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
}

void RaylibBackend::DrawBox(const Vec3& halfExtents, const Transform& transform, const Color& color) {
    // DrawCubeV is axis-aligned; use a wire cube rotated via DrawCubeWiresV
    // approximation: draw the box in local space by transforming the center
    // and using raylib's rotated cube through a temporary model-free path.
    const Vec3 world = transform.position;
    DrawCubeV(ToRaylib(world), {halfExtents.x * 2.f, halfExtents.y * 2.f, halfExtents.z * 2.f},
              ToRaylib(color));
}

void RaylibBackend::DrawLine3D(const Vec3& start, const Vec3& end, const Color& color) {
    ::DrawLine3D(ToRaylib(start), ToRaylib(end), ToRaylib(color));
}

void RaylibBackend::DrawGrid(int slices, float spacing) {
    ::DrawGrid(slices, spacing);
}

void RaylibBackend::DrawRect2D(float x, float y, float w, float h, const Color& color) {
    ::DrawRectangle(static_cast<int>(x), static_cast<int>(y), static_cast<int>(w),
                    static_cast<int>(h), ToRaylib(color));
}

void RaylibBackend::DrawText2D(const std::string& text, float x, float y, int size, const Color& color) {
    ::DrawText(text.c_str(), static_cast<int>(x), static_cast<int>(y), size, ToRaylib(color));
}

Vec2f RaylibBackend::MeasureText2D(const std::string& text, int size) const {
    return {static_cast<float>(::MeasureText(text.c_str(), size)), static_cast<float>(size)};
}

double RaylibBackend::TimeSeconds() const { return ::GetTime(); }
float RaylibBackend::FrameTime() const { return ::GetFrameTime(); }

void RaylibBackend::CaptureScreenshot(const std::string& path) {
    ::TakeScreenshot(path.c_str());
}

}  // namespace apex
