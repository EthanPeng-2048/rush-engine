module;
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <kvrec/kvrecord.hpp>

module rush.engine.core;

namespace apex {

namespace {

constexpr std::uint64_t kMapFormatVersion = 1;

std::uint64_t FloatBits(float f) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    return bits;
}

float BitsFloat(std::uint64_t v) {
    const std::uint32_t bits = static_cast<std::uint32_t>(v);
    float f = 0.f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

std::vector<float> UnpackFloats(const std::vector<std::uint64_t>& in) {
    std::vector<float> out;
    out.reserve(in.size());
    for (std::uint64_t v : in) out.push_back(BitsFloat(v));
    return out;
}

std::vector<std::uint64_t> PackFloats(const std::vector<float>& in) {
    std::vector<std::uint64_t> out;
    out.reserve(in.size());
    for (float f : in) out.push_back(FloatBits(f));
    return out;
}

std::string MapFilePath(const std::string& mapsRoot, const std::string& name) {
    std::string base = name;
    constexpr const char* kExt = ".rushmap";
    if (base.size() > 8 && base.compare(base.size() - 8, 8, kExt) == 0) {
        base.resize(base.size() - 8);
    }
    return (std::filesystem::path(mapsRoot) / (base + kExt)).string();
}

bool ReadWholeFile(const std::string& path, std::string& out) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) return false;
    out.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return true;
}

bool WriteWholeFile(const std::string& path, const std::string& bytes) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) return false;
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(file);
}

}  // namespace

float MapData::TerrainHeight(float x, float z) const {
    if (gridX.size() < 2 || gridZ.size() < 2 || gridY.empty()) return 0.f;
    auto locate = [](const std::vector<float>& axis, float v) {
        if (v <= axis.front()) return std::pair<size_t, float>{0, 0.f};
        if (v >= axis.back()) return std::pair<size_t, float>{axis.size() - 2, 1.f};
        const size_t i = static_cast<size_t>(
            std::upper_bound(axis.begin(), axis.end(), v) - axis.begin() - 1);
        const float t = (v - axis[i]) / std::max(1e-4f, axis[i + 1] - axis[i]);
        return std::pair<size_t, float>{i, t};
    };
    const auto [ix, tx] = locate(gridX, x);
    const auto [iz, tz] = locate(gridZ, z);
    const size_t cols = gridX.size();
    const float h00 = gridY[iz * cols + ix];
    const float h10 = gridY[iz * cols + ix + 1];
    const float h01 = gridY[(iz + 1) * cols + ix];
    const float h11 = gridY[(iz + 1) * cols + ix + 1];
    const float a = Lerp(h00, h10, tx);
    const float b = Lerp(h01, h11, tx);
    return Lerp(a, b, tz);
}

float MapData::MinTerrainY() const {
    float m = 0.f;
    for (float y : gridY) m = std::min(m, y);
    return m;
}

bool SaveMapToFile(const std::string& path, const MapData& map) {
    if (map.centerline.size() < 3) return false;

    kvrec::KeyValueRecord root;
    root.set("format_version", kMapFormatVersion);
    root.set("schema", std::string("apex.map"));
    root.set("name", map.name);

    {
        kvrec::KeyValueRecord cl;
        const size_t n = map.centerline.size();
        cl.set("count", static_cast<std::uint64_t>(n));
        std::vector<std::uint64_t> xs, ys, zs, widths, gl, vl, gr, vr;
        xs.reserve(n);
        ys.reserve(n);
        zs.reserve(n);
        widths.reserve(n);
        gl.reserve(n);
        vl.reserve(n);
        gr.reserve(n);
        vr.reserve(n);
        for (const MapData::WayPoint& w : map.centerline) {
            xs.push_back(FloatBits(w.position.x));
            ys.push_back(FloatBits(w.position.y));
            zs.push_back(FloatBits(w.position.z));
            widths.push_back(FloatBits(w.width));
            gl.push_back(FloatBits(w.grassLeft));
            vl.push_back(FloatBits(w.gravelLeft));
            gr.push_back(FloatBits(w.grassRight));
            vr.push_back(FloatBits(w.gravelRight));
        }
        cl.set("x", xs);
        cl.set("y", ys);
        cl.set("z", zs);
        cl.set("width", widths);
        cl.set("grass_left", gl);
        cl.set("gravel_left", vl);
        cl.set("grass_right", gr);
        cl.set("gravel_right", vr);
        root.set("centerline", cl);
    }

    if (!map.gridX.empty() && !map.gridZ.empty() && !map.gridY.empty()) {
        kvrec::KeyValueRecord terrain;
        terrain.set("grid_x", PackFloats(map.gridX));
        terrain.set("grid_z", PackFloats(map.gridZ));
        terrain.set("grid_y", PackFloats(map.gridY));
        root.set("terrain", terrain);
    }

    if (!map.decor.empty()) {
        kvrec::KeyValueRecord decor;
        const size_t n = map.decor.size();
        std::vector<std::uint64_t> types, xs, ys, zs, scales;
        types.reserve(n);
        xs.reserve(n);
        ys.reserve(n);
        zs.reserve(n);
        scales.reserve(n);
        for (const MapData::Decor& d : map.decor) {
            types.push_back(static_cast<std::uint64_t>(d.type));
            xs.push_back(FloatBits(d.position.x));
            ys.push_back(FloatBits(d.position.y));
            zs.push_back(FloatBits(d.position.z));
            scales.push_back(FloatBits(d.scale));
        }
        decor.set("type", types);
        decor.set("x", xs);
        decor.set("y", ys);
        decor.set("z", zs);
        decor.set("scale", scales);
        root.set("decor", decor);
    }

    if (!map.drsZones.empty()) {
        kvrec::KeyValueRecord zones;
        std::vector<std::uint64_t> starts, ends;
        starts.reserve(map.drsZones.size());
        ends.reserve(map.drsZones.size());
        for (const MapData::DrsZone& z : map.drsZones) {
            starts.push_back(FloatBits(z.start));
            ends.push_back(FloatBits(z.end));
        }
        zones.set("drs_start", starts);
        zones.set("drs_end", ends);
        root.set("zones", zones);
    }

    return WriteWholeFile(path, root.serialize_v2());
}

MapData LoadMapFromFile(const std::string& path) {
    MapData map;
    std::string bytes;
    if (!ReadWholeFile(path, bytes)) return map;

    auto parsed = kvrec::KeyValueRecord::parse(bytes);
    if (!parsed) return map;

    std::string schema;
    if (parsed->get("schema", schema) && schema != "apex.map") return map;

    std::string name;
    if (parsed->get("name", name)) map.name = name;

    kvrec::KeyValueRecord cl;
    if (!parsed->get("centerline", cl)) return map;

    std::vector<std::uint64_t> xs, ys, zs, widths, gl, vl, gr, vr;
    if (!cl.get("x", xs) || !cl.get("y", ys) || !cl.get("z", zs)) return map;
    (void)cl.get("width", widths);
    (void)cl.get("grass_left", gl);
    (void)cl.get("gravel_left", vl);
    (void)cl.get("grass_right", gr);
    (void)cl.get("gravel_right", vr);

    const size_t n = xs.size();
    if (n < 3 || ys.size() != n || zs.size() != n) return map;
    auto at = [](const std::vector<std::uint64_t>& a, size_t i, float fallback) {
        return i < a.size() ? BitsFloat(a[i]) : fallback;
    };

    map.centerline.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        MapData::WayPoint w;
        w.position = {BitsFloat(xs[i]), BitsFloat(ys[i]), BitsFloat(zs[i])};
        w.width = at(widths, i, 12.f);
        w.grassLeft = at(gl, i, 0.f);
        w.gravelLeft = at(vl, i, 0.f);
        w.grassRight = at(gr, i, 0.f);
        w.gravelRight = at(vr, i, 0.f);
        map.centerline.push_back(w);
    }

    kvrec::KeyValueRecord terrain;
    if (parsed->get("terrain", terrain)) {
        std::vector<std::uint64_t> gx, gy, gz;
        if (terrain.get("grid_x", gx) && terrain.get("grid_z", gz) && terrain.get("grid_y", gy)) {
            map.gridX = UnpackFloats(gx);
            map.gridZ = UnpackFloats(gz);
            map.gridY = UnpackFloats(gy);
        }
    }

    kvrec::KeyValueRecord decor;
    if (parsed->get("decor", decor)) {
        std::vector<std::uint64_t> types, dx, dy, dz, scales;
        if (decor.get("type", types) && decor.get("x", dx) && decor.get("y", dy) &&
            decor.get("z", dz)) {
            (void)decor.get("scale", scales);
            const size_t dn = dx.size();
            map.decor.reserve(dn);
            for (size_t i = 0; i < dn; ++i) {
                MapData::Decor d;
                d.type = i < types.size() ? static_cast<int>(types[i]) : 0;
                d.position = {BitsFloat(dx[i]), BitsFloat(dy[i]), BitsFloat(dz[i])};
                d.scale = at(scales, i, 1.f);
                map.decor.push_back(d);
            }
        }
    }

    kvrec::KeyValueRecord zones;
    if (parsed->get("zones", zones)) {
        std::vector<std::uint64_t> starts, ends;
        if (zones.get("drs_start", starts) && zones.get("drs_end", ends)) {
            const size_t zn = std::min(starts.size(), ends.size());
            map.drsZones.reserve(zn);
            for (size_t i = 0; i < zn; ++i) {
                MapData::DrsZone z;
                z.start = BitsFloat(starts[i]);
                z.end = BitsFloat(ends[i]);
                if (z.end > z.start) map.drsZones.push_back(z);
            }
        }
    }

    map.ok = map.centerline.size() >= 3;
    return map;
}

bool SaveMapData(const std::string& mapsRoot, const std::string& name, const MapData& map) {
    return SaveMapToFile(MapFilePath(mapsRoot, name), map);
}

MapData LoadMapData(const std::string& mapsRoot, const std::string& name) {
    return LoadMapFromFile(MapFilePath(mapsRoot, name));
}

std::vector<std::string> ListMaps(const std::string& mapsRoot) {
    std::vector<std::string> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(mapsRoot, ec)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != ".rushmap") continue;
        out.push_back(entry.path().stem().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace apex
