#!/usr/bin/env python3
"""Assemble a relocatable usdRig download bundle.

Copies a complete OpenUSD install, the RigExec ``cmake --install`` prefix,
and the CPython home the USD build was compiled against into one directory
with the ``usdRigView`` launcher at its head. Nothing in the output refers
to the build machine: the launcher derives every path from its own
location at run time (see packaging/launcher/usdRigView.cpp).

Layout (Windows/Linux)::

    <output>/
      usdRigView[.exe]          launcher
      bundle-manifest.json
      README-bundle.md, LICENSE, NOTICE, THIRD_PARTY_NOTICES.md
      app/
        usd/                    OpenUSD install
        rig/                    RigExec install (lib, lib/python, lib/usd)
        python/                 bundled CPython home
        plugins/touchPose, plugins/shapeEditor
        examples/rigexec_flat.usda, examples/ArmShotAnim.usda

On macOS the launcher and payload are wrapped as an app instead:
``usdRigView.app/Contents/MacOS/usdRigView`` with the same ``app`` tree in
``Contents/Resources/app``, so the whole bundle can be code-signed and
notarized as one unit. Manifest, README and legal texts sit beside the
``.app``.

Stdlib only. Run ``--help`` for the inputs; every required input is
validated before anything is copied and all missing pieces are reported
at once. ``packaging/test_assemble_bundle.py`` exercises this script
against synthetic trees.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import shutil
import sys
import tarfile
import zipfile
from pathlib import Path

PLATFORMS = ("windows", "macos", "linux")
IMAGING_NAMES = ("rigExecImaging.dll", "librigExecImaging.dylib",
                 "librigExecImaging.so")
TOOL_NAMES = ("rigExecPose", "rigExecBake", "rigExecReplayInputs")
EXAMPLE_NAMES = ("rigexec_flat.usda", "ArmShotAnim.usda")
LEGAL_NAMES = ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md")

_IGNORE_RUNTIME = shutil.ignore_patterns(
    "__pycache__", "*.pyc", "*.pyo", "*.pdb")
_IGNORE_TRIM_DEV = shutil.ignore_patterns(
    "__pycache__", "*.pyc", "*.pyo", "*.pdb", "*.lib", "*.a",
    "include", "cmake")


def _ignore(trim_dev: bool):
    return _IGNORE_TRIM_DEV if trim_dev else _IGNORE_RUNTIME


def python_executable(python_home: Path, platform: str) -> Path:
    if platform == "windows":
        return python_home / "python.exe"
    for candidate in (python_home / "bin" / "python3",
                      python_home / "bin" / "python",
                      python_home / "python3"):
        if candidate.is_file():
            return candidate
    return python_home / "bin" / "python3"


def validate(args) -> list[str]:
    problems = []
    usd = Path(args.usd_install)
    rig = Path(args.rig_install)
    python_home = Path(args.python_home)
    source = Path(args.source_root)
    if not (usd / "bin" / "usdview").is_file():
        problems.append(f"OpenUSD install has no bin/usdview: {usd}")
    if not any((rig / "lib" / name).is_file() for name in IMAGING_NAMES):
        problems.append(f"RigExec install has no imaging library: {rig}/lib")
    for plug in ("rigExecSchema", "rigExecImaging"):
        info = rig / "lib" / "usd" / plug / "resources" / "plugInfo.json"
        if not info.is_file():
            problems.append(f"RigExec install is missing {info}")
    if not python_executable(python_home, args.platform).is_file():
        problems.append(
            f"Python home has no interpreter for {args.platform}: "
            f"{python_home} (the bundle needs the interpreter the "
            "OpenUSD build was compiled against, e.g. setup-python's "
            "pythonLocation)")
    if not Path(args.launcher).is_file():
        problems.append(f"launcher not found: {args.launcher}")
    for plugin in ("touchPose", "shapeEditor"):
        if not (source / "plugin" / plugin / "plugInfo.json").is_file():
            problems.append(f"source tree is missing plugin/{plugin}")
    if args.rig_build_dir and not Path(args.rig_build_dir).is_dir():
        problems.append(f"rig build dir not found: {args.rig_build_dir}")
    if args.usdnoodles_dir and not Path(args.usdnoodles_dir).is_dir():
        problems.append(f"usdNoodles dir not found: {args.usdnoodles_dir}")
    return problems


def copy_tree(src: Path, dst: Path, trim_dev: bool,
              preserve_symlinks: bool = False) -> None:
    # macOS framework bundles (Python.framework, Qt) are symlink webs;
    # dereferencing them breaks the bundle structure codesign seals.
    shutil.copytree(src, dst, dirs_exist_ok=True,
                    ignore=_ignore(trim_dev), symlinks=preserve_symlinks)


def make_executable(path: Path) -> None:
    if os.name != "nt":
        path.chmod(path.stat().st_mode | 0o755)


def info_plist(version: str) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" \
"http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key><string>usdRigView</string>
  <key>CFBundleIdentifier</key><string>org.usdrig.usdRigView</string>
  <key>CFBundleName</key><string>usdRigView</string>
  <key>CFBundleDisplayName</key><string>usdRig Viewer</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>{version}</string>
  <key>CFBundleVersion</key><string>{version}</string>
  <key>LSMinimumSystemVersion</key><string>13.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
"""


def readme_text(version: str, platform: str) -> str:
    if platform == "windows":
        run = ("Double-click `usdRigView.exe`, or from a terminal:\n\n"
               "```bat\nusdRigView.exe path\\to\\stage.usda\n```")
    elif platform == "macos":
        run = ("Double-click `usdRigView.app`, or from a terminal:\n\n"
               "```sh\nopen usdRigView.app --args \"$PWD/stage.usda\"\n```\n\n"
               "An unsigned build is stopped by Gatekeeper on first "
               "launch: right-click the app, choose Open, and confirm. "
               "Signed release builds do not show this.")
    else:
        run = ("From a terminal in this directory:\n\n"
               "```sh\n./usdRigView path/to/stage.usda\n```")
    return f"""# usdRig {version} ({platform})

A self-contained usdview with the RigExec evaluation engine. No separate
OpenUSD, Python, or Qt install is needed, and nothing is installed
system-wide: extract the archive anywhere and run the launcher.

{run}

With no stage argument the viewer opens a blank stage. Every other
argument is passed to usdview unchanged.

## Checking the download

```sh
usdRigView --usdRig-check
```

prints `usdRigView bundle OK` when every bundled component is in place,
and names the missing file otherwise. Keep the launcher beside its `app`
folder (inside the `.app` on macOS); the pair only works together.

## Contents

- `app/usd` - OpenUSD install the viewer runs on
- `app/python` - the CPython those OpenUSD bindings were built for
- `app/rig` - RigExec libraries, schemas, imaging plug-ins, and the
  `rigExecPose` / `rigExecBake` command-line tools under `app/rig/bin`
- `app/plugins` - the TouchPose and Shape Editor usdview plug-ins
- `app/examples` - `rigexec_flat.usda` and `ArmShotAnim.usda` to try

Licences for usdRig and every bundled component are in the legal texts
beside this file and under `app/rig/share/rigExec`.
"""


def assemble(args) -> Path:
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()) and not args.force:
        raise SystemExit(f"output is not empty (pass --force): {output}")
    version = args.version.lstrip("v")
    platform = args.platform

    if platform == "macos":
        app_bundle = output / "usdRigView.app"
        contents = app_bundle / "Contents"
        launcher_dst = contents / "MacOS" / "usdRigView"
        app_dir = contents / "Resources" / "app"
    else:
        suffix = ".exe" if platform == "windows" else ""
        launcher_dst = output / f"usdRigView{suffix}"
        app_dir = output / "app"
    app_dir.mkdir(parents=True, exist_ok=True)
    launcher_dst.parent.mkdir(parents=True, exist_ok=True)

    shutil.copy2(args.launcher, launcher_dst)
    make_executable(launcher_dst)
    if platform == "macos":
        (contents / "Info.plist").write_text(info_plist(version),
                                             encoding="utf-8")
        (contents / "PkgInfo").write_text("APPL????", encoding="utf-8")

    keep_links = platform == "macos"
    copy_tree(Path(args.usd_install), app_dir / "usd", args.trim_dev,
              preserve_symlinks=keep_links)
    copy_tree(Path(args.rig_install), app_dir / "rig", args.trim_dev,
              preserve_symlinks=keep_links)
    copy_tree(Path(args.python_home), app_dir / "python", args.trim_dev,
              preserve_symlinks=keep_links)
    make_executable(python_executable(app_dir / "python", platform))

    source = Path(args.source_root)
    for plugin in ("touchPose", "shapeEditor"):
        copy_tree(source / "plugin" / plugin,
                  app_dir / "plugins" / plugin, trim_dev=False)

    noodles_dst = app_dir / "rig" / "lib" / "python" / "UsdNoodles"
    if args.usdnoodles_dir and not (noodles_dst / "plugInfo.json").is_file():
        copy_tree(Path(args.usdnoodles_dir), noodles_dst, trim_dev=False)

    if args.rig_build_dir:
        bin_dir = app_dir / "rig" / "bin"
        for tool in TOOL_NAMES:
            for name in (tool, tool + ".exe"):
                candidate = Path(args.rig_build_dir) / name
                if candidate.is_file():
                    bin_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(candidate, bin_dir / name)
                    make_executable(bin_dir / name)

    examples_dir = app_dir / "examples"
    for name in EXAMPLE_NAMES:
        candidate = source / "examples" / name
        if candidate.is_file():
            examples_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(candidate, examples_dir / name)

    for name in LEGAL_NAMES:
        candidate = source / name
        if candidate.is_file():
            shutil.copy2(candidate, output / name)
    (output / "README-bundle.md").write_text(
        readme_text(version, platform), encoding="utf-8")

    manifest = {
        "bundle": "usdRig",
        "version": version,
        "platform": args.platform_slug or platform,
        "git_sha": args.git_sha or None,
        "generated_utc": datetime.datetime.now(
            datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "launcher": launcher_dst.name,
        "components": {
            "openusd": "app/usd",
            "python": "app/python",
            "rigexec": "app/rig",
            "plugins": "app/plugins",
        },
    }
    manifest_path = output / "bundle-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n",
                             encoding="utf-8")
    return output


def create_archive(output: Path, archive: Path) -> Path:
    archive = Path(archive).resolve()
    archive.parent.mkdir(parents=True, exist_ok=True)
    if archive.suffix == ".zip":
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zf:
            for path in sorted(output.rglob("*")):
                if path.is_file():
                    zf.write(path, path.relative_to(output.parent))
    elif archive.name.endswith((".tar.gz", ".tgz")):
        with tarfile.open(archive, "w:gz") as tf:
            tf.add(output, arcname=output.name)
    else:
        raise SystemExit(f"unsupported archive type: {archive}")
    return archive


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd-install", required=True)
    parser.add_argument("--rig-install", required=True)
    parser.add_argument("--python-home", required=True,
                        help="CPython home to bundle (its bin/python3 or "
                             "python.exe is copied with the tree)")
    parser.add_argument("--launcher", required=True,
                        help="built usdRigView binary for this platform")
    parser.add_argument("--output", required=True)
    parser.add_argument("--version", required=True,
                        help="release version, with or without leading v")
    parser.add_argument("--platform", choices=PLATFORMS,
                        default={"win32": "windows", "darwin": "macos"}.get(
                            sys.platform, "linux"))
    parser.add_argument("--platform-slug", default="",
                        help="manifest/archive label, e.g. windows-x64")
    parser.add_argument("--source-root", default=".",
                        help="usdRig checkout holding plugin/, examples/ "
                             "and the legal texts")
    parser.add_argument("--rig-build-dir", default="",
                        help="build tree to copy rigExecPose/rigExecBake/"
                             "rigExecReplayInputs from")
    parser.add_argument("--usdnoodles-dir", default="",
                        help="staged UsdNoodles package, when the RigExec "
                             "install does not already carry it")
    parser.add_argument("--git-sha", default="")
    parser.add_argument("--archive", default="",
                        help="also write this .zip / .tar.gz of the bundle")
    parser.add_argument("--trim-dev", action="store_true",
                        help="drop headers, import/static libraries and "
                             "CMake package files (smaller, runtime-only)")
    parser.add_argument("--force", action="store_true",
                        help="assemble into a non-empty output directory")
    args = parser.parse_args(argv)

    problems = validate(args)
    if problems:
        for problem in problems:
            print(f"assemble_bundle: {problem}", file=sys.stderr)
        return 2
    output = assemble(args)
    print(f"bundle: {output}")
    if args.archive:
        print(f"archive: {create_archive(output, Path(args.archive))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
