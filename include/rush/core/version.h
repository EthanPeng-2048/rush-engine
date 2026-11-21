#pragma once

namespace rush {

// Semantic version of the engine library (not the game).
constexpr int kVersionMajor = 0;
constexpr int kVersionMinor = 1;
constexpr int kVersionPatch = 0;

// Runtime string, e.g. "0.1.0". Defined in version.cpp.
const char* VersionString();

}  // namespace rush
