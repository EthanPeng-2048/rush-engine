module;
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

module rush.engine.core;



namespace apex {

namespace {

std::string Trim(const std::string& s) {
    size_t begin = 0;
    while (begin < s.size() && (s[begin] == ' ' || s[begin] == '\t' || s[begin] == '\r')) ++begin;
    size_t end = s.size();
    while (end > begin && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r')) --end;
    return s.substr(begin, end - begin);
}

}  // namespace

bool Config::Load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    values_.clear();
    std::string line;
    std::string section;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;
        if (line.front() == '[' && line.back() == ']') {
            section = Trim(line.substr(1, line.size() - 2));
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        values_[section][Trim(line.substr(0, eq))] = Trim(line.substr(eq + 1));
    }
    return true;
}

bool Config::Save(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) return false;

    for (const auto& [section, pairs] : values_) {
        if (!section.empty()) file << '[' << section << "]\n";
        for (const auto& [key, value] : pairs) {
            file << key << '=' << value << '\n';
        }
        file << '\n';
    }
    return true;
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

