module;
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>

#include <kvrec/kvrecord.hpp>

module rush.engine.core;

namespace apex {

namespace {

constexpr std::uint64_t kConfigFormatVersion = 1;

std::string SectionKey(const std::string& section, const std::string& key) {
    return section.empty() ? key : section + "." + key;
}

}  // namespace

bool Config::Load(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) return false;

    const std::string bytes((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
    auto parsed = kvrec::KeyValueRecord::parse(bytes);
    if (!parsed) return false;

    values_.clear();
    // Flat dotted keys: "section.key" -> Str. format_version is UInt and skipped.
    for (const std::string& full : parsed->string_keys_with_prefix("")) {
        std::string raw;
        if (!parsed->get(full, raw)) continue;
        const size_t dot = full.find('.');
        if (dot == std::string::npos) {
            values_[""][full] = raw;
        } else {
            values_[full.substr(0, dot)][full.substr(dot + 1)] = raw;
        }
    }
    return true;
}

bool Config::Save(const std::string& path) const {
    kvrec::KeyValueRecord rec;
    rec.set("format_version", kConfigFormatVersion);
    for (const auto& [section, pairs] : values_) {
        for (const auto& [key, value] : pairs) {
            rec.set(SectionKey(section, key), value);
        }
    }

    const std::string bytes = rec.serialize_v2();
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) return false;
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(file);
}

std::string Config::GetString(const std::string& section, const std::string& key,
                              const std::string& fallback) const {
    const auto sectionIt = values_.find(section);
    if (sectionIt == values_.end()) return fallback;
    const auto keyIt = sectionIt->second.find(key);
    if (keyIt == sectionIt->second.end()) return fallback;
    return keyIt->second;
}

float Config::GetFloat(const std::string& section, const std::string& key, float fallback) const {
    const std::string raw = GetString(section, key);
    if (raw.empty()) return fallback;
    try {
        return std::stof(raw);
    } catch (...) {
        return fallback;
    }
}

int Config::GetInt(const std::string& section, const std::string& key, int fallback) const {
    const std::string raw = GetString(section, key);
    if (raw.empty()) return fallback;
    try {
        return std::stoi(raw);
    } catch (...) {
        return fallback;
    }
}

bool Config::GetBool(const std::string& section, const std::string& key, bool fallback) const {
    const std::string raw = GetString(section, key);
    if (raw.empty()) return fallback;
    return raw == "1" || raw == "true" || raw == "True" || raw == "yes";
}

void Config::SetString(const std::string& section, const std::string& key, const std::string& value) {
    values_[section][key] = value;
}

}  // namespace apex
