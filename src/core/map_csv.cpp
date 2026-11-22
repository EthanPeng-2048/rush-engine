module;
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

module rush.engine.core;

// CSV authoring import for tools/mapconv. Runtime game path uses LoadMapData
// (.rushmap) only — this file exists so the converter can share MapData.

namespace apex {

namespace {

std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        const size_t b = field.find_first_not_of(" \t\r\n");
        const size_t e = field.find_last_not_of(" \t\r\n");
        out.push_back(b == std::string::npos ? "" : field.substr(b, e - b + 1));
    }
    return out;
}

bool IsCommentOrEmpty(const std::string& line) {
    for (char c : line) {
        if (c == '#') return true;
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') return false;
    }
    return true;
}

}  // namespace

MapData LoadMapFromCsvDir(const std::string& dir) {
    MapData map;
    const std::filesystem::path root(dir);
    map.name = root.filename().string();

    // track.csv — required.
    {
        std::ifstream in(root / "track.csv");
        if (!in) return map;
        std::string line;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 8) continue;
            MapData::WayPoint w;
            // Author columns: x, z, y, ... → Vec3{x,y,z}
            w.position = {std::stof(f[0]), std::stof(f[2]), std::stof(f[1])};
            w.width = std::stof(f[3]);
            w.grassLeft = std::stof(f[4]);
            w.gravelLeft = std::stof(f[5]);
            w.grassRight = std::stof(f[6]);
            w.gravelRight = std::stof(f[7]);
            map.centerline.push_back(w);
        }
        if (map.centerline.size() < 3) return map;
    }

    // terrain.csv — optional height posts.
    {
        std::ifstream in(root / "terrain.csv");
        std::string line;
        std::vector<float> xs, zs, ys;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 3) continue;
            xs.push_back(std::stof(f[0]));
            zs.push_back(std::stof(f[1]));
            ys.push_back(std::stof(f[2]));
        }
        if (xs.size() >= 4) {
            map.gridX = xs;
            map.gridZ = zs;
            std::sort(map.gridX.begin(), map.gridX.end());
            map.gridX.erase(std::unique(map.gridX.begin(), map.gridX.end()), map.gridX.end());
            std::sort(map.gridZ.begin(), map.gridZ.end());
            map.gridZ.erase(std::unique(map.gridZ.begin(), map.gridZ.end()), map.gridZ.end());
            const size_t cols = map.gridX.size();
            map.gridY.assign(cols * map.gridZ.size(), 0.f);
            for (size_t r = 0; r < map.gridZ.size(); ++r) {
                for (size_t c = 0; c < cols; ++c) {
                    float best = 1e30f, bestY = 0.f;
                    for (size_t k = 0; k < xs.size(); ++k) {
                        const float d = std::fabs(xs[k] - map.gridX[c]) +
                                        std::fabs(zs[k] - map.gridZ[r]);
                        if (d < best) {
                            best = d;
                            bestY = ys[k];
                        }
                    }
                    map.gridY[r * cols + c] = bestY;
                }
            }
        }
    }

    // decor.csv — optional.
    {
        std::ifstream in(root / "decor.csv");
        std::string line;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 5) continue;
            MapData::Decor d;
            d.type = (f[0] == "tree") ? 0 : 1;
            d.position = {std::stof(f[1]), std::stof(f[3]), std::stof(f[2])};
            d.scale = std::stof(f[4]);
            map.decor.push_back(d);
        }
    }

    // zones.csv — optional DRS activation zones (lap-length fractions).
    {
        std::ifstream in(root / "zones.csv");
        std::string line;
        while (std::getline(in, line)) {
            if (IsCommentOrEmpty(line)) continue;
            const std::vector<std::string> f = SplitCsv(line);
            if (f.size() < 3) continue;
            std::string type = f[0];
            for (char& c : type) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (type != "drs") continue;
            MapData::DrsZone zone;
            zone.start = Clampf(std::stof(f[1]), 0.f, 1.f);
            zone.end = Clampf(std::stof(f[2]), 0.f, 1.f);
            if (zone.end > zone.start) map.drsZones.push_back(zone);
        }
    }

    map.ok = true;
    return map;
}

}  // namespace apex
