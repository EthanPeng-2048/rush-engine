module;
#include <sys/stat.h>
#include <string>

#ifdef _WIN32
extern "C" __declspec(dllimport) unsigned long __stdcall GetModuleFileNameA(
    void* module, char* filename, unsigned long size);
#else
#include <unistd.h>
#endif

module rush.engine.core;




namespace apex {

namespace {

bool DirExists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFDIR) != 0;
}

std::string ExeDir() {
#ifdef _WIN32
    char buffer[1024];
    const unsigned long len = GetModuleFileNameA(nullptr, buffer, sizeof(buffer));
    if (len == 0 || len >= sizeof(buffer)) return "";
    std::string path(buffer, len);
    const size_t slash = path.find_last_of("\\/");
    return slash == std::string::npos ? std::string() : path.substr(0, slash);
#else
    // /proc/self/exe works on Linux; emscripten has no exe to find and the
    // current directory ("/" with preloaded assets) resolves fine.
    char buffer[1024];
    const ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len <= 0) return "";
    buffer[len] = '\0';
    std::string path(buffer);
    const size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? std::string() : path.substr(0, slash);
#endif
}

}  // namespace

const std::string& DataRoot() {
    static const std::string root = [] {
        const std::string exeDir = ExeDir();
        const std::string candidates[] = {".", exeDir, exeDir.empty() ? "" : exeDir + "/.."};
        for (const std::string& candidate : candidates) {
            if (!candidate.empty() && DirExists(candidate + "/assets/maps")) return candidate;
        }
        return std::string(".");
    }();
    return root;
}

std::string DataPath(const std::string& rel) {
    const std::string& root = DataRoot();
    return root == "." ? rel : root + "/" + rel;
}

}  // namespace apex

