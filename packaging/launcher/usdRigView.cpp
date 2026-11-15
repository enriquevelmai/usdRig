// usdRigView: native launcher for the bundled usdRig download.
//
// The bundle is a directory, not a single binary: a full OpenUSD install,
// a matching CPython, the RigExec install, and the usdview plug-ins must
// stay discoverable by USD's Plug registry at run time. This executable
// is the one file a user runs. It finds its own location, derives every
// bundled path from it (never from the working directory or a checkout),
// exports the environment bin/usdview.sh exports for a source tree, and
// replaces itself with the bundled Python running the bundled usdview.
//
// Modes:
//   usdRigView [stage.usda] [usdview flags...]   launch (default)
//   usdRigView --usdRig-check                    verify the bundle only
//   usdRigView --usdRig-print-env                print the computed env
//
// With no arguments a blank stage is written to the temp directory and
// opened, matching bin/launch_usdview.bat / bin/usdview.sh.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <limits.h>
#endif

namespace {

namespace fs = std::filesystem;

struct Bundle {
    fs::path root;
    fs::path app;
    fs::path python;
    fs::path pythonDir;
    fs::path usdview;
    fs::path usdSitePackages;
    fs::path imagingLibrary;
    std::vector<std::string> problems;
};

bool
IsFile(const fs::path &path)
{
    std::error_code ec;
    return fs::is_regular_file(path, ec);
}

bool
IsDir(const fs::path &path)
{
    std::error_code ec;
    return fs::is_directory(path, ec);
}

fs::path
ExecutablePath()
{
#if defined(_WIN32)
    std::vector<wchar_t> buffer(32768);
    const DWORD length = GetModuleFileNameW(nullptr, buffer.data(),
                                            static_cast<DWORD>(buffer.size()));
    if (length == 0) {
        return fs::path();
    }
    return fs::path(std::wstring(buffer.data(), length));
#elif defined(__APPLE__)
    char raw[PATH_MAX];
    uint32_t size = sizeof(raw);
    if (_NSGetExecutablePath(raw, &size) != 0) {
        return fs::path();
    }
    std::error_code ec;
    fs::path canonical = fs::canonical(fs::path(raw), ec);
    return ec ? fs::path(raw) : canonical;
#else
    std::error_code ec;
    fs::path target = fs::read_symlink(fs::path("/proc/self/exe"), ec);
    return ec ? fs::path() : target;
#endif
}

const char *
ImagingFileName()
{
#if defined(_WIN32)
    return "rigExecImaging.dll";
#elif defined(__APPLE__)
    return "librigExecImaging.dylib";
#else
    return "librigExecImaging.so";
#endif
}

// First existing USD Python package directory, in the layouts the three
// platform installers produce (Lib/site-packages, lib/python3.x/
// site-packages, lib/python).
fs::path
FindUsdSitePackages(const fs::path &usd)
{
    const fs::path candidates[] = {
        usd / "Lib" / "site-packages",
        usd / "lib" / "python",
    };
    for (const fs::path &candidate : candidates) {
        if (IsDir(candidate)) {
            return candidate;
        }
    }
    std::error_code ec;
    const fs::path lib = usd / "lib";
    for (const fs::directory_entry &entry : fs::directory_iterator(lib, ec)) {
        if (!entry.is_directory(ec)) {
            continue;
        }
        const fs::path site = entry.path() / "site-packages";
        if (entry.path().filename().string().rfind("python", 0) == 0 &&
            IsDir(site)) {
            return site;
        }
    }
    return candidates[1];
}

fs::path
FindPython(const fs::path &app)
{
#if defined(_WIN32)
    return app / "python" / "python.exe";
#else
    const fs::path candidates[] = {
        app / "python" / "bin" / "python3",
        app / "python" / "bin" / "python",
        app / "python" / "python3",
    };
    for (const fs::path &candidate : candidates) {
        if (IsFile(candidate)) {
            return candidate;
        }
    }
    return candidates[0];
#endif
}

Bundle
LocateBundle()
{
    Bundle bundle;
    const fs::path exe = ExecutablePath();
    if (exe.empty()) {
        bundle.problems.push_back("cannot determine the executable path");
        return bundle;
    }
    const fs::path exeDir = exe.parent_path();
#if defined(__APPLE__)
    // <root>/usdRigView.app/Contents/MacOS/usdRigView with the payload in
    // Contents/Resources/app; a plain directory bundle keeps the POSIX
    // layout below, so local test bundles work on macOS too.
    if (exeDir.filename() == "MacOS" &&
        exeDir.parent_path().filename() == "Contents") {
        const fs::path contents = exeDir.parent_path();
        bundle.root = contents.parent_path().parent_path();
        bundle.app = contents / "Resources" / "app";
    } else {
        bundle.root = exeDir;
        bundle.app = exeDir / "app";
    }
#else
    bundle.root = exeDir;
    bundle.app = exeDir / "app";
#endif
    const fs::path &app = bundle.app;
    bundle.python = FindPython(app);
    bundle.pythonDir = bundle.python.parent_path();
    bundle.usdview = app / "usd" / "bin" / "usdview";
    bundle.usdSitePackages = FindUsdSitePackages(app / "usd");
    bundle.imagingLibrary = app / "rig" / "lib" / ImagingFileName();

    struct Requirement {
        fs::path path;
        const char *what;
    };
    const Requirement required[] = {
        {bundle.python, "bundled Python interpreter"},
        {bundle.usdview, "bundled usdview script"},
        {bundle.usdSitePackages, "USD Python packages (pxr)"},
        {bundle.imagingLibrary, "RigExec imaging library"},
        {app / "rig" / "lib" / "usd" / "rigExecSchema" / "resources" /
             "plugInfo.json",
         "RigExec schema plugInfo"},
        {app / "rig" / "lib" / "usd" / "rigExecImaging" / "resources" /
             "plugInfo.json",
         "RigExec imaging plugInfo"},
        {app / "plugins" / "touchPose" / "plugInfo.json", "TouchPose plug-in"},
        {app / "plugins" / "shapeEditor" / "plugInfo.json",
         "Shape Editor plug-in"},
    };
    for (const Requirement &item : required) {
        if (!IsFile(item.path) && !IsDir(item.path)) {
            bundle.problems.push_back(std::string("missing ") + item.what +
                                      ": " + item.path.string());
        }
    }
    return bundle;
}

std::string
Separator()
{
#if defined(_WIN32)
    return ";";
#else
    return ":";
#endif
}

void
AppendExisting(std::vector<std::string> &entries, const fs::path &path)
{
    if (IsDir(path) || IsFile(path)) {
        entries.push_back(path.string());
    }
}

std::string
Join(const std::vector<std::string> &entries, const std::string &tail)
{
    std::string joined;
    for (const std::string &entry : entries) {
        if (!joined.empty()) {
            joined += Separator();
        }
        joined += entry;
    }
    if (!tail.empty()) {
        if (!joined.empty()) {
            joined += Separator();
        }
        joined += tail;
    }
    return joined;
}

std::string
ExistingEnv(const char *name)
{
#if defined(_WIN32)
    std::vector<char> buffer(32768);
    const DWORD length = GetEnvironmentVariableA(
        name, buffer.data(), static_cast<DWORD>(buffer.size()));
    if (length == 0 || length >= buffer.size()) {
        return std::string();
    }
    return std::string(buffer.data(), length);
#else
    const char *value = std::getenv(name);
    return value ? std::string(value) : std::string();
#endif
}

using Env = std::vector<std::pair<std::string, std::string>>;

Env
BuildEnv(const Bundle &bundle)
{
    const fs::path &app = bundle.app;
    const fs::path rigLib = app / "rig" / "lib";
    const fs::path rigPython = rigLib / "python";
    const fs::path usd = app / "usd";

    std::vector<std::string> pythonPath;
    AppendExisting(pythonPath, rigPython);
    if (IsDir(bundle.usdSitePackages)) {
        pythonPath.push_back(bundle.usdSitePackages.string());
    }
    AppendExisting(pythonPath, app / "plugins" / "touchPose");
    AppendExisting(pythonPath, app / "plugins" / "shapeEditor");

    std::vector<std::string> pluginPath;
    AppendExisting(pluginPath, rigLib / "usd" / "rigExecSchema" / "resources");
    AppendExisting(pluginPath, rigLib / "usd" / "rigExecImaging" / "resources");
    AppendExisting(pluginPath, rigLib / "usd" / "rigExecMoverPlugins");
    AppendExisting(pluginPath, app / "plugins" / "touchPose");
    AppendExisting(pluginPath, app / "plugins" / "shapeEditor");
    AppendExisting(pluginPath, rigPython / "UsdNoodles");

    std::vector<std::string> path;
    AppendExisting(path, bundle.pythonDir);
    AppendExisting(path, rigLib);
    AppendExisting(path, usd / "bin");
    AppendExisting(path, usd / "lib");

    std::vector<std::string> loaderPath;
    AppendExisting(loaderPath, rigLib);
    AppendExisting(loaderPath, usd / "lib");

    Env env;
    env.emplace_back("PYTHONPATH", Join(pythonPath, ExistingEnv("PYTHONPATH")));
    env.emplace_back("PXR_PLUGINPATH_NAME",
                     Join(pluginPath, ExistingEnv("PXR_PLUGINPATH_NAME")));
    env.emplace_back("PATH", Join(path, ExistingEnv("PATH")));
#if defined(__APPLE__)
    env.emplace_back("DYLD_LIBRARY_PATH",
                     Join(loaderPath, ExistingEnv("DYLD_LIBRARY_PATH")));
#elif !defined(_WIN32)
    env.emplace_back("LD_LIBRARY_PATH",
                     Join(loaderPath, ExistingEnv("LD_LIBRARY_PATH")));
#endif
    env.emplace_back("RIGEXEC_IMAGING_DLL", bundle.imagingLibrary.string());
    env.emplace_back("TOUCHPOSE_PLUGIN_DIR",
                     (app / "plugins" / "touchPose").string());
    env.emplace_back("PYTHONNOUSERSITE", "1");
    return env;
}

void
ApplyEnv(const Env &env)
{
    for (const std::pair<std::string, std::string> &pair : env) {
#if defined(_WIN32)
        SetEnvironmentVariableA(pair.first.c_str(), pair.second.c_str());
#else
        setenv(pair.first.c_str(), pair.second.c_str(), 1);
#endif
    }
}

fs::path
WriteBlankStage()
{
    fs::path dir;
#if defined(_WIN32)
    char buffer[MAX_PATH];
    const DWORD length = GetTempPathA(MAX_PATH, buffer);
    dir = length ? fs::path(std::string(buffer, length)) : fs::path(".");
#else
    const char *tmp = std::getenv("TMPDIR");
    dir = (tmp && tmp[0]) ? fs::path(tmp) : fs::path("/tmp");
#endif
    const fs::path stage = dir / "rigexec-blank.usda";
#if defined(_WIN32)
    FILE *file = nullptr;
    fopen_s(&file, stage.string().c_str(), "w");
#else
    FILE *file = std::fopen(stage.string().c_str(), "w");
#endif
    if (file) {
        std::fputs("#usda 1.0\ndef Xform \"World\" {\n}\n", file);
        std::fclose(file);
    }
    return stage;
}

int
ReportProblems(const Bundle &bundle)
{
    for (const std::string &problem : bundle.problems) {
        std::fprintf(stderr, "usdRigView: %s\n", problem.c_str());
    }
    std::fprintf(
        stderr, "usdRigView: this download is incomplete; re-extract the whole "
                "archive and keep usdRigView beside its app folder.\n");
    return 2;
}

#if defined(_WIN32)

std::wstring
Widen(const std::string &text)
{
    if (text.empty()) {
        return std::wstring();
    }
    const int size = MultiByteToWideChar(
        CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    std::vector<wchar_t> buffer(static_cast<size_t>(size));
    MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
                        buffer.data(), size);
    return std::wstring(buffer.data(), buffer.size());
}

void
QuoteInto(std::wstring &out, const std::wstring &arg)
{
    if (!arg.empty() && arg.find_first_of(L" \t\"") == std::wstring::npos) {
        out += arg;
        return;
    }
    out += L'"';
    size_t backslashes = 0;
    for (const wchar_t ch : arg) {
        if (ch == L'\\') {
            ++backslashes;
            continue;
        }
        if (ch == L'"') {
            out.append(backslashes * 2 + 1, L'\\');
            out += L'"';
        } else {
            out.append(backslashes, L'\\');
            out += ch;
        }
        backslashes = 0;
    }
    out.append(backslashes * 2, L'\\');
    out += L'"';
}

int
LaunchWindows(const Bundle &bundle, const Env &env,
              const std::vector<std::string> &args)
{
    ApplyEnv(env);
    std::wstring cmdline;
    QuoteInto(cmdline, Widen(bundle.python.string()));
    cmdline += L' ';
    QuoteInto(cmdline, Widen(bundle.usdview.string()));
    for (const std::string &arg : args) {
        cmdline += L' ';
        QuoteInto(cmdline, Widen(arg));
    }
    std::vector<wchar_t> mutableCmd(cmdline.begin(), cmdline.end());
    mutableCmd.push_back(L'\0');
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};
    if (!CreateProcessW(nullptr, mutableCmd.data(), nullptr, nullptr, TRUE, 0,
                        nullptr, nullptr, &startup, &process)) {
        std::fprintf(stderr,
                     "usdRigView: failed to start bundled Python "
                     "(error %lu).\n",
                     static_cast<unsigned long>(GetLastError()));
        return 1;
    }
    WaitForSingleObject(process.hProcess, INFINITE);
    DWORD code = 1;
    GetExitCodeProcess(process.hProcess, &code);
    CloseHandle(process.hThread);
    CloseHandle(process.hProcess);
    return static_cast<int>(code);
}

#endif

} // namespace

int
main(int argc, char **argv)
{
    std::vector<std::string> args(argv + 1, argv + argc);
    const bool checkMode = !args.empty() && args.front() == "--usdRig-check";
    const bool printEnvMode =
        !args.empty() && args.front() == "--usdRig-print-env";

    const Bundle bundle = LocateBundle();
    if (checkMode) {
        if (!bundle.problems.empty()) {
            return ReportProblems(bundle);
        }
        std::printf("usdRigView bundle OK\n  python: %s\n  usdview: %s\n",
                    bundle.python.string().c_str(),
                    bundle.usdview.string().c_str());
        return 0;
    }
    if (!bundle.problems.empty()) {
        return ReportProblems(bundle);
    }

    const Env env = BuildEnv(bundle);
    if (printEnvMode) {
        for (const std::pair<std::string, std::string> &pair : env) {
            std::printf("%s=%s\n", pair.first.c_str(), pair.second.c_str());
        }
        return 0;
    }

    if (args.empty()) {
        args.push_back(WriteBlankStage().string());
    }

#if defined(_WIN32)
    return LaunchWindows(bundle, env, args);
#else
    ApplyEnv(env);
    std::vector<std::string> execArgs;
    execArgs.push_back(bundle.python.string());
    execArgs.push_back(bundle.usdview.string());
    execArgs.insert(execArgs.end(), args.begin(), args.end());
    std::vector<char *> raw;
    raw.reserve(execArgs.size() + 1);
    for (std::string &arg : execArgs) {
        raw.push_back(arg.data());
    }
    raw.push_back(nullptr);
    execv(raw.front(), raw.data());
    std::fprintf(stderr, "usdRigView: failed to exec bundled Python: %s\n",
                 std::strerror(errno));
    return 1;
#endif
}
