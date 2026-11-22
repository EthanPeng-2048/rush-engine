module;
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <exstl/errors.hpp>
#include <exstl/error_macros.hpp>

export module rush.engine.core;

export namespace apex {


// Engine-agnostic math types. Render/physics backends convert to their own
// representations at the boundary so core code never depends on a library.
struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float LengthSq(const Vec3& v) { return Dot(v, v); }
inline float Length(const Vec3& v) { return std::sqrt(LengthSq(v)); }

inline Vec3 Normalize(const Vec3& v) {
    const float len = Length(v);
    return len > 1e-6f ? v / len : Vec3{0.f, 0.f, 0.f};
}

// Quaternion, Hamilton convention, rotates vectors: v' = q * v * q^-1.
struct Quat {
    float x = 0.f, y = 0.f, z = 0.f, w = 1.f;

    Quat() = default;
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat Identity() { return {}; }

    static Quat FromAxisAngle(const Vec3& axis, float angleRad) {
        const Vec3 n = Normalize(axis);
        const float h = angleRad * 0.5f;
        const float s = std::sin(h);
        return {n.x * s, n.y * s, n.z * s, std::cos(h)};
    }
};

inline Quat operator*(const Quat& a, const Quat& b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = Cross(u, v) * 2.f;
    return v + t * q.w + Cross(u, t);
}

// Inverse of a unit quaternion is its conjugate.
inline Quat Conjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

inline Vec3 RotateInv(const Quat& q, const Vec3& v) { return Rotate(Conjugate(q), v); }

struct Transform {
    Vec3 position;
    Quat rotation;
};

inline Vec3 TransformPoint(const Transform& t, const Vec3& p) {
    return Rotate(t.rotation, p) + t.position;
}

inline Vec3 TransformDirection(const Transform& t, const Vec3& d) {
    return Rotate(t.rotation, d);
}

// Stores the 4x4 matrix M with M * (p, 1) == TransformPoint(t, p) as 16
// floats in row-major order (out[row*4 + col]). This matches raylib's Matrix
// aggregate member order (m0, m4, m8, m12, m1, ...), so the values can be
// copied straight into it. Getting the transpose wrong inverts the rotation
// and hides the translation in the w row, which stretches vertices wildly.
inline void TransformToMatrix(const Transform& t, float out[16]) {
    const float x = t.rotation.x, y = t.rotation.y, z = t.rotation.z, w = t.rotation.w;
    out[0] = 1.f - 2.f * (y * y + z * z);
    out[1] = 2.f * (x * y - z * w);
    out[2] = 2.f * (x * z + y * w);
    out[3] = t.position.x;
    out[4] = 2.f * (x * y + z * w);
    out[5] = 1.f - 2.f * (x * x + z * z);
    out[6] = 2.f * (y * z - x * w);
    out[7] = t.position.y;
    out[8] = 2.f * (x * z - y * w);
    out[9] = 2.f * (y * z + x * w);
    out[10] = 1.f - 2.f * (x * x + y * y);
    out[11] = t.position.z;
    out[12] = 0.f;
    out[13] = 0.f;
    out[14] = 0.f;
    out[15] = 1.f;
}

constexpr float kPi = 3.14159265358979f;
constexpr float kDegToRad = kPi / 180.f;
constexpr float kRadToDeg = 180.f / kPi;

inline float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float Signf(float v) { return v >= 0.f ? 1.f : -1.f; }




// Key-value config persisted next to the EXE as config.rushconf (KVRec).
// Section/key API is unchanged for game code; storage is KVRec under the hood.
class Config {
public:
    // Test/automation mode: Load() and Save() become no-ops so CLI automation
    // (--drive-test / --bench / QA shots / --dump-scene) never reads or writes
    // the user's config.rushconf. Off by default — normal play persists as
    // usual. Set once at startup before any Load/Save.
    static void SetTestMode(bool on);

    bool Load(const std::string& path);
    bool Save(const std::string& path) const;

    std::string GetString(const std::string& section, const std::string& key,
                          const std::string& fallback = "") const;
    float GetFloat(const std::string& section, const std::string& key, float fallback = 0.f) const;
    int GetInt(const std::string& section, const std::string& key, int fallback = 0) const;
    bool GetBool(const std::string& section, const std::string& key, bool fallback = false) const;

    void SetString(const std::string& section, const std::string& key, const std::string& value);

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> values_;
};

// Data-driven map loaded from a .rushmap file (KVRec v2). Authoring still
// goes Python/CSV -> tools/mapconv -> <name>.rushmap; runtime only reads
// the binary form.
struct MapData {
    struct WayPoint {
        Vec3 position;
        float width = 12.f;
        float grassLeft = 0.f, gravelLeft = 0.f;
        float grassRight = 0.f, gravelRight = 0.f;
    };

    struct Decor {
        int type = 0;  // 0 = tree
        Vec3 position;
        float scale = 1.f;
    };

    // DRS activation zone as a lap-length fraction (0..1). Converted to
    // arc length by Track::Build.
    struct DrsZone {
        float start = 0.f;
        float end = 0.f;
    };

    std::string name;
    std::vector<WayPoint> centerline;
    std::vector<Decor> decor;
    std::vector<DrsZone> drsZones;

    // Regular height-post grid (sorted unique axes + row-major heights).
    std::vector<float> gridX, gridZ;
    std::vector<float> gridY;  // gridZ.size() rows x gridX.size() cols

    bool ok = false;

    // Bilinear terrain height (gridY outside clamps to the edge value).
    float TerrainHeight(float x, float z) const;
    float MinTerrainY() const;
};

// Load mapsRoot/<name>.rushmap (optional .rushmap suffix on name tolerated).
// Missing/corrupt file or centerline < 3 points => ok=false.
MapData LoadMapData(const std::string& mapsRoot, const std::string& name);

// Map basenames under mapsRoot (*.rushmap). Empty on failure.
std::vector<std::string> ListMaps(const std::string& mapsRoot);

// Write mapsRoot/<name>.rushmap from map (mapconv / editor).
bool SaveMapData(const std::string& mapsRoot, const std::string& name, const MapData& map);

// Read/write a single file path (mapconv CLI).
MapData LoadMapFromFile(const std::string& path);
bool SaveMapToFile(const std::string& path, const MapData& map);

// Authoring import: read a CSV map directory (track/terrain/decor/zones).
// Used by tools/mapconv; runtime game path uses LoadMapData only.
MapData LoadMapFromCsvDir(const std::string& dir);

// Data root for config.rushconf and assets/: resolved once at first use so the
// game finds its data regardless of the working directory. Search order:
//   1. current directory (normal in-tree runs)
//   2. the executable's directory (shipped layout: exe next to assets/)
//   3. the executable's parent (running build\apex-rush.exe in-tree)
// The first root containing assets/maps wins; config.rushconf lives in the same
// root so settings never split across directories.
const std::string& DataRoot();

// DataRoot() + "/" + rel ("config.rushconf", "assets/maps", ...).
std::string DataPath(const std::string& rel);


}  // namespace apex

export namespace rush {
using Error = exstl::Error;
template <typename T> using Result = exstl::Result<T>;
using exstl::check_result;
}  // namespace rush
