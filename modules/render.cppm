module;
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

export module rush.engine.render;

import rush.engine.core;

export namespace apex {


struct Vec2f {
    float x = 0.f, y = 0.f;
};

struct Color {
    float r = 1.f, g = 1.f, b = 1.f, a = 1.f;

    static Color White() { return {1.f, 1.f, 1.f, 1.f}; }
    static Color Black() { return {0.f, 0.f, 0.f, 1.f}; }
    static Color Red() { return {1.f, 0.2f, 0.2f, 1.f}; }
    static Color Green() { return {0.2f, 1.f, 0.3f, 1.f}; }
    static Color Blue() { return {0.25f, 0.5f, 1.f, 1.f}; }
    static Color Yellow() { return {1.f, 0.9f, 0.2f, 1.f}; }
    static Color Gray() { return {0.55f, 0.55f, 0.58f, 1.f}; }
    static Color DarkGray() { return {0.22f, 0.22f, 0.25f, 1.f}; }
    static Color Orange() { return {1.f, 0.55f, 0.15f, 1.f}; }
};

struct CameraDesc {
    Vec3 position{0.f, 5.f, -10.f};
    Vec3 target{0.f, 0.f, 0.f};
    Vec3 up{0.f, 1.f, 0.f};
    float fovYDeg = 65.f;
};

// Video settings applied at runtime from the settings menu. Backends
// implement what they support and ignore the rest (headless = everything).
struct VideoConfig {
    bool borderlessFullscreen = false;
    int width = 1280;
    int height = 720;
    int fpsLimit = 0;       // 0 = uncapped
    bool shadows = true;    // quality: simple ground shadows
};

// Triangle mesh in engine-agnostic form. Winding is counter-clockwise.
struct MeshData {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<float> uvs;              // 2 floats per vertex
    std::vector<uint8_t> colors;         // optional RGBA bytes per vertex
    std::vector<uint32_t> indices;
};

using MeshHandle = uint32_t;
constexpr MeshHandle kInvalidMesh = 0;

// CPU-side RGBA8 texture pixels handed to IRenderer::CreateTexture. The
// backend copies the data immediately, so the caller may free `pixels` after
// the call returns. Rows are top to bottom (origin = top-left), matching the
// UV convention (v=0 at the top of the texture).
struct TextureData {
    int width = 0;
    int height = 0;
    const uint8_t* pixels = nullptr;
};

using TextureHandle = uint32_t;
constexpr TextureHandle kInvalidTexture = 0;

// Unified render interface. The game layer only talks to this, so the
// backend (raylib today, something else later) is replaceable.
// Input lives in rush.engine.input — this interface is render-only.
class IRenderer {
public:
    virtual ~IRenderer() = default;

    virtual bool Init(int width, int height, const std::string& title) = 0;
    virtual void Shutdown() = 0;

    virtual bool ShouldClose() const = 0;
    // Frame lifecycle: BeginFrame/Clear -> BeginScene/EndScene (3D) ->
    // 2D overlay calls -> EndFrame.
    virtual void BeginFrame() = 0;
    virtual void BeginScene() = 0;
    virtual void EndScene() = 0;
    virtual void EndFrame() = 0;

    virtual void SetCamera(const CameraDesc& camera) = 0;
    virtual int ScreenWidth() const = 0;
    virtual int ScreenHeight() const = 0;

    virtual void ApplyVideo(const VideoConfig& video) = 0;
    virtual bool ShadowsEnabled() const = 0;

    virtual MeshHandle CreateMesh(const MeshData& data) = 0;
    virtual void DestroyMesh(MeshHandle handle) = 0;
    // Draw a mesh. When `texture` is set, the mesh's albedo is sampled from it
    // and a lit material is applied (diffuse + specular); when it is
    // kInvalidTexture the mesh falls back to the unlit vertex-color path.
    virtual void DrawMesh(MeshHandle handle, const Transform& transform, const Color& tint,
                          TextureHandle texture = kInvalidTexture) = 0;
    // Cheap quality-option shadow: squashes the mesh onto the ground plane at
    // groundY (top-down footprint) and draws it translucent dark.
    virtual void DrawMeshShadow(MeshHandle handle, const Transform& transform, float groundY,
                                const Color& tint) = 0;

    // Upload CPU RGBA8 pixels as a GPU texture; the backend copies the data.
    // Returns kInvalidTexture on failure.
    virtual TextureHandle CreateTexture(const TextureData& data) = 0;
    virtual void DestroyTexture(TextureHandle handle) = 0;

    // Immediate primitives for debugging, HUD and low-poly bodies.
    virtual void DrawBox(const Vec3& halfExtents, const Transform& transform, const Color& color) = 0;
    virtual void DrawLine3D(const Vec3& start, const Vec3& end, const Color& color) = 0;
    virtual void DrawGrid(int slices, float spacing) = 0;

    virtual void DrawRect2D(float x, float y, float w, float h, const Color& color) = 0;
    virtual void DrawText2D(const std::string& text, float x, float y, int size, const Color& color) = 0;
    virtual Vec2f MeasureText2D(const std::string& text, int size) const = 0;

    virtual double TimeSeconds() const = 0;
    virtual float FrameTime() const = 0;
    virtual void CaptureScreenshot(const std::string& path) = 0;
};




// Headless renderer with no window, so the game logic (physics, race rules,
// AI) can run in automated tests and on machines without a display. Doubles
// as proof that the render layer is replaceable. Input is separate — tests
// drive it via NullInputSource + InputSystem (rush.engine.input).
class NullBackend final : public IRenderer {
public:
    // Fixed timestep reported by FrameTime(); physics runs deterministically.
    float frameDt = 1.f / 60.f;

    bool Init(int width, int height, const std::string& title) override;
    void Shutdown() override;
    bool ShouldClose() const override;
    void BeginFrame() override;
    void BeginScene() override;
    void EndScene() override;
    void EndFrame() override;
    void SetCamera(const CameraDesc& camera) override;
    int ScreenWidth() const override;
    int ScreenHeight() const override;
    void ApplyVideo(const VideoConfig& video) override;
    bool ShadowsEnabled() const override;
    MeshHandle CreateMesh(const MeshData& data) override;
    void DestroyMesh(MeshHandle handle) override;
    void DrawMesh(MeshHandle handle, const Transform& transform, const Color& tint,
                  TextureHandle texture = kInvalidTexture) override;
    void DrawMeshShadow(MeshHandle handle, const Transform& transform, float groundY,
                        const Color& tint) override;
    TextureHandle CreateTexture(const TextureData& data) override;
    void DestroyTexture(TextureHandle handle) override;
    void DrawBox(const Vec3& halfExtents, const Transform& transform, const Color& color) override;
    void DrawLine3D(const Vec3& start, const Vec3& end, const Color& color) override;
    void DrawGrid(int slices, float spacing) override;
    void DrawRect2D(float x, float y, float w, float h, const Color& color) override;
    void DrawText2D(const std::string& text, float x, float y, int size, const Color& color) override;
    Vec2f MeasureText2D(const std::string& text, int size) const override;
    double TimeSeconds() const override;
    float FrameTime() const override;
    void CaptureScreenshot(const std::string& path) override;

    // Debug capture: record DrawMesh calls so the composed scene can be
    // exported (DumpObj) and inspected without a GPU/window.
    bool captureDraws = false;
    void DumpObj(const std::string& path) const;

private:
    struct DrawCmd {
        MeshHandle mesh = kInvalidMesh;
        Transform transform;
        Color tint;
        TextureHandle texture = kInvalidTexture;
    };

    int width_ = 1280;
    int height_ = 720;
    VideoConfig video_;
    double time_ = 0.0;
    MeshHandle nextMesh_ = 1;
    TextureHandle nextTexture_ = 1;
    std::unordered_map<MeshHandle, MeshData> meshes_;
    struct OwnedTexture {
        int width = 0;
        int height = 0;
        std::vector<uint8_t> pixels;  // RGBA8 copy (backend owns it)
    };
    std::unordered_map<TextureHandle, OwnedTexture> textures_;
    std::vector<DrawCmd> draws_;
    bool closeRequested_ = false;
};




// Raylib implementation of the render interface. All raylib types stay
// inside this class so the game layer never sees them. Input is sampled by
// RaylibInputSource (rush.engine.input), not here.
class RaylibBackend final : public IRenderer {
public:
    ~RaylibBackend() override;

    bool Init(int width, int height, const std::string& title) override;
    void Shutdown() override;
    bool ShouldClose() const override;
    void BeginFrame() override;
    void BeginScene() override;
    void EndScene() override;
    void EndFrame() override;
    void SetCamera(const CameraDesc& camera) override;
    int ScreenWidth() const override;
    int ScreenHeight() const override;
    void ApplyVideo(const VideoConfig& video) override;
    bool ShadowsEnabled() const override;
    MeshHandle CreateMesh(const MeshData& data) override;
    void DestroyMesh(MeshHandle handle) override;
    void DrawMesh(MeshHandle handle, const Transform& transform, const Color& tint,
                  TextureHandle texture = kInvalidTexture) override;
    void DrawMeshShadow(MeshHandle handle, const Transform& transform, float groundY,
                        const Color& tint) override;
    TextureHandle CreateTexture(const TextureData& data) override;
    void DestroyTexture(TextureHandle handle) override;
    void DrawBox(const Vec3& halfExtents, const Transform& transform, const Color& color) override;
    void DrawLine3D(const Vec3& start, const Vec3& end, const Color& color) override;
    void DrawGrid(int slices, float spacing) override;
    void DrawRect2D(float x, float y, float w, float h, const Color& color) override;
    void DrawText2D(const std::string& text, float x, float y, int size, const Color& color) override;
    Vec2f MeasureText2D(const std::string& text, int size) const override;
    double TimeSeconds() const override;
    float FrameTime() const override;
    void CaptureScreenshot(const std::string& path) override;

private:
    struct Impl;
    Impl* impl_ = nullptr;
};


}  // namespace apex
