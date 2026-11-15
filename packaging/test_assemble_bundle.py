#!/usr/bin/env python3
"""Synthetic-tree tests for packaging/assemble_bundle.py.

Builds fake OpenUSD / RigExec / CPython trees in a temp directory, assembles
bundles for each platform layout, and -- when a real usdRigView binary is
passed -- runs its --usdRig-check against the bundle for the host platform.

Run directly:  python3 packaging/test_assemble_bundle.py [usdRigView-path]
Or via ctest:  testRigExecBundlePackaging.
"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ASSEMBLE = HERE / "assemble_bundle.py"
# Optional first positional argument: a built usdRigView binary (ctest
# passes $<TARGET_FILE:usdRigView>). Consumed here, not by unittest.
LAUNCHER = None
if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
    LAUNCHER = Path(sys.argv[1])
    del sys.argv[1]
HOST_PLATFORM = {"win32": "windows", "darwin": "macos"}.get(
    sys.platform, "linux")
IMAGING = {"windows": "rigExecImaging.dll",
           "macos": "librigExecImaging.dylib",
           "linux": "librigExecImaging.so"}[HOST_PLATFORM]


def touch(path: Path, text: str = "x\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def make_trees(base: Path, platform: str) -> dict:
    usd = base / "usd-install"
    touch(usd / "bin" / "usdview", "# usdview\n")
    touch(usd / "lib" / "usd_usd.dll", "lib\n")
    if platform == "windows":
        touch(usd / "Lib" / "site-packages" / "pxr" / "__init__.py")
    else:
        touch(usd / "lib" / "python3.11" / "site-packages" / "pxr" /
              "__init__.py")
    rig = base / "rig-install"
    imaging = {"windows": "rigExecImaging.dll",
               "macos": "librigExecImaging.dylib",
               "linux": "librigExecImaging.so"}[platform]
    touch(rig / "lib" / imaging, "lib\n")
    for plug in ("rigExecSchema", "rigExecImaging"):
        touch(rig / "lib" / "usd" / plug / "resources" / "plugInfo.json",
              "{}\n")
    touch(rig / "lib" / "python" / "rigexec" / "__init__.py")
    python_home = base / "python-home"
    if platform == "windows":
        touch(python_home / "python.exe", "py\n")
        touch(python_home / "Lib" / "os.py")
    else:
        touch(python_home / "bin" / "python3", "py\n")
        touch(python_home / "lib" / "python3.11" / "os.py")
    source = base / "checkout"
    for plugin in ("touchPose", "shapeEditor"):
        touch(source / "plugin" / plugin / "plugInfo.json", "{}\n")
        touch(source / "plugin" / plugin / "module.py")
    for name in ("rigexec_flat.usda", "ArmShotAnim.usda"):
        touch(source / "examples" / name, "#usda 1.0\n")
    for name in ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md"):
        touch(source / name, f"{name}\n")
    build = base / "build"
    tool = "rigExecPose.exe" if platform == "windows" else "rigExecPose"
    touch(build / tool, "tool\n")
    return {"usd": usd, "rig": rig, "python": python_home,
            "source": source, "build": build}


def assemble(base: Path, platform: str, launcher: Path, archive=True):
    trees = make_trees(base, platform)
    output = base / f"bundle-{platform}"
    cmd = [sys.executable, str(ASSEMBLE),
           "--usd-install", str(trees["usd"]),
           "--rig-install", str(trees["rig"]),
           "--python-home", str(trees["python"]),
           "--launcher", str(launcher),
           "--output", str(output),
           "--version", "v1.2.0",
           "--platform", platform,
           "--platform-slug", f"{platform}-x64",
           "--source-root", str(trees["source"]),
           "--rig-build-dir", str(trees["build"]),
           "--git-sha", "deadbeef"]
    if archive:
        suffix = ".zip" if platform == "windows" else ".tar.gz"
        cmd += ["--archive", str(base / f"bundle-{platform}{suffix}")]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc, output, trees


class AssembleBundleTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.base = Path(self._tmp.name)
        self.launcher = LAUNCHER
        self.dummy_launcher = self.base / "dummy-launcher"
        self.dummy_launcher.write_bytes(b"launcher\n")

    def tearDown(self):
        self._tmp.cleanup()

    def _launcher(self):
        return self.launcher or self.dummy_launcher

    def test_windows_layout_and_zip(self):
        proc, output, _ = assemble(self.base, "windows", self._launcher())
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertTrue((output / "usdRigView.exe").is_file())
        self.assertTrue((output / "app" / "usd" / "bin" / "usdview").is_file())
        self.assertTrue((output / "app" / "python" / "python.exe").is_file())
        self.assertTrue((output / "app" / "rig" / "bin" /
                         "rigExecPose.exe").is_file())
        self.assertTrue((output / "app" / "plugins" / "touchPose" /
                         "plugInfo.json").is_file())
        self.assertTrue((output / "app" / "examples" /
                         "rigexec_flat.usda").is_file())
        manifest = json.loads(
            (output / "bundle-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["version"], "1.2.0")
        self.assertEqual(manifest["platform"], "windows-x64")
        self.assertEqual(manifest["git_sha"], "deadbeef")
        archive = self.base / "bundle-windows.zip"
        self.assertTrue(archive.is_file())
        with zipfile.ZipFile(archive) as zf:
            names = zf.namelist()
        self.assertTrue(any(n.endswith("usdRigView.exe") for n in names))
        self.assertTrue(any("bundle-manifest.json" in n for n in names))

    def test_macos_app_layout_and_tarball(self):
        proc, output, _ = assemble(self.base, "macos", self._launcher())
        self.assertEqual(proc.returncode, 0, proc.stderr)
        app = output / "usdRigView.app" / "Contents"
        self.assertTrue((app / "MacOS" / "usdRigView").is_file())
        plist = (app / "Info.plist").read_text(encoding="utf-8")
        self.assertIn("org.usdrig.usdRigView", plist)
        self.assertIn("<string>1.2.0</string>", plist)
        self.assertTrue((app / "Resources" / "app" / "usd" / "bin" /
                         "usdview").is_file())
        with tarfile.open(self.base / "bundle-macos.tar.gz") as tf:
            names = tf.getnames()
        self.assertTrue(any("usdRigView.app/Contents/MacOS/usdRigView" in n
                            for n in names))

    def test_linux_layout_and_tarball(self):
        proc, output, _ = assemble(self.base, "linux", self._launcher())
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertTrue((output / "usdRigView").is_file())
        self.assertTrue((output / "app" / "python" / "bin" /
                         "python3").is_file())
        with tarfile.open(self.base / "bundle-linux.tar.gz") as tf:
            self.assertTrue(any(n.endswith("/usdRigView")
                                for n in tf.getnames()))

    @unittest.skipUnless(os.name == "posix", "symlinks need POSIX")
    def test_copy_tree_preserves_symlinks_when_asked(self):
        spec = importlib.util.spec_from_file_location("assemble_bundle",
                                                      ASSEMBLE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        src = self.base / "fw-src" / "Python.framework" / "Versions"
        touch(src / "3.11" / "Python", "binary\n")
        (src / "Current").symlink_to("3.11", target_is_directory=True)
        kept = self.base / "kept"
        module.copy_tree(src.parent.parent, kept, trim_dev=False,
                         preserve_symlinks=True)
        self.assertTrue(
            (kept / "Python.framework" / "Versions" / "Current").is_symlink())
        copied = self.base / "copied"
        module.copy_tree(src.parent.parent, copied, trim_dev=False)
        self.assertFalse(
            (copied / "Python.framework" / "Versions" / "Current").is_symlink())

    def test_missing_inputs_report_every_problem(self):
        empty = self.base / "empty"
        empty.mkdir()
        cmd = [sys.executable, str(ASSEMBLE),
               "--usd-install", str(empty), "--rig-install", str(empty),
               "--python-home", str(empty), "--launcher", str(empty / "no"),
               "--output", str(self.base / "out"), "--version", "1.2.0",
               "--platform", "linux", "--source-root", str(empty)]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(proc.returncode, 2)
        for needle in ("bin/usdview", "imaging library", "plugInfo",
                       "Python home", "launcher not found", "touchPose"):
            self.assertIn(needle, proc.stderr)

    def test_real_launcher_check_mode(self):
        if not self.launcher:
            self.skipTest("no usdRigView binary passed")
        proc, output, _ = assemble(self.base, HOST_PLATFORM, self.launcher,
                                   archive=False)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        if HOST_PLATFORM == "macos":
            exe = (output / "usdRigView.app" / "Contents" / "MacOS" /
                   "usdRigView")
        elif HOST_PLATFORM == "windows":
            exe = output / "usdRigView.exe"
        else:
            exe = output / "usdRigView"
        ok = subprocess.run([str(exe), "--usdRig-check"],
                            capture_output=True, text=True)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertIn("bundle OK", ok.stdout)

        if HOST_PLATFORM == "macos":
            imaging = (output / "usdRigView.app" / "Contents" /
                       "Resources" / "app" / "rig" / "lib" / IMAGING)
        else:
            imaging = output / "app" / "rig" / "lib" / IMAGING
        imaging.unlink()
        broken = subprocess.run([str(exe), "--usdRig-check"],
                                capture_output=True, text=True)
        self.assertEqual(broken.returncode, 2)
        self.assertIn("imaging library", broken.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
