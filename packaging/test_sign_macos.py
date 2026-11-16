#!/usr/bin/env python3
"""Tests for packaging/sign-macos.sh's signing-requirement gate.

Published releases must never ship unsigned or unnotarized, while
local/test builds may. Synthetic bundle directories and mocked signing
tools exercise the gate and notarization outcomes without credentials.

Run directly:  python3 packaging/test_sign_macos.py
Or via ctest:  testRigExecMacSigning
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / "sign-macos.sh"


def run_sign(bundle: Path, **env_overrides):
    env = dict(os.environ)
    for key in ("MACOS_SIGNING_IDENTITY", "AC_API_KEY_ID",
                "AC_API_ISSUER_ID", "AC_API_KEY_PATH",
                "RIGEXEC_REQUIRE_NOTARIZATION"):
        env.pop(key, None)
    env.update(env_overrides)
    return subprocess.run(["bash", str(SCRIPT), str(bundle)],
                          capture_output=True, text=True, env=env)


@unittest.skipUnless(sys.platform == "darwin", "sign-macos.sh is macOS-only")
class SignMacosGateTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.bundle = Path(self._tmp.name) / "bundle"
        (self.bundle / "usdRigView.app").mkdir(parents=True)

    def tearDown(self):
        self._tmp.cleanup()

    def test_unsigned_allowed_for_test_builds(self):
        proc = run_sign(self.bundle)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("UNSIGNED", proc.stderr)

    def test_unsigned_refused_when_notarization_required(self):
        proc = run_sign(self.bundle, RIGEXEC_REQUIRE_NOTARIZATION="1")
        self.assertEqual(proc.returncode, 1)
        self.assertIn("refusing to publish an UNSIGNED bundle",
                      proc.stderr)

    def test_bogus_identity_fails(self):
        proc = run_sign(self.bundle, RIGEXEC_REQUIRE_NOTARIZATION="1",
                        MACOS_SIGNING_IDENTITY="Developer ID Application: X (TEAM)")
        self.assertEqual(proc.returncode, 1)

    def test_macho_symlink_escaping_bundle_is_refused(self):
        outside = Path(self._tmp.name) / "outside-binary"
        outside.write_bytes(Path("/usr/bin/true").read_bytes()
                            if Path("/usr/bin/true").exists()
                            else Path("/bin/true").read_bytes())
        link = self.bundle / "usdRigView.app" / "Contents" / "MacOS" / "escape"
        link.parent.mkdir(parents=True, exist_ok=True)
        link.symlink_to(outside)
        proc = run_sign(self.bundle,
                        MACOS_SIGNING_IDENTITY="Developer ID Application: X (TEAM)")
        self.assertEqual(proc.returncode, 1)
        self.assertIn("escapes the bundle", proc.stderr)

    def _mock_notarization(self, status, submit_exit="0", malformed=False,
                           relative_bundle=False):
        tools = Path(self._tmp.name) / "mock-tools"
        tools.mkdir()
        calls = Path(self._tmp.name) / "calls.jsonl"
        mock = """#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

name = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["MOCK_CALLS"], "a", encoding="utf-8") as stream:
    stream.write(json.dumps([name] + args) + "\\n")
if name == "xcrun" and args[:2] == ["notarytool", "submit"]:
    if os.environ["MOCK_MALFORMED"] == "1":
        print("unavailable response")
    else:
        print(json.dumps({"id": "test-submission", "status": os.environ["MOCK_STATUS"]}))
    sys.exit(int(os.environ["MOCK_SUBMIT_EXIT"]))
if name == "xcrun" and args[:2] == ["notarytool", "log"]:
    print('{"issues": [{"message": "test notarization diagnostic"}]}')
"""
        for name in ("codesign", "ditto", "xcrun", "spctl"):
            tool = tools / name
            tool.write_text(mock, encoding="utf-8")
            tool.chmod(0o755)
        bundle_arg = Path(os.path.relpath(self.bundle)) if relative_bundle else self.bundle
        proc = run_sign(
            bundle_arg, PATH=str(tools) + os.pathsep + os.environ["PATH"],
            MOCK_CALLS=str(calls), MOCK_STATUS=status,
            MOCK_SUBMIT_EXIT=submit_exit, MOCK_MALFORMED="1" if malformed else "0",
            MACOS_SIGNING_IDENTITY="test identity", AC_API_KEY_ID="test-key",
            AC_API_ISSUER_ID="test-issuer", AC_API_KEY_PATH="test-key.p8",
            RIGEXEC_REQUIRE_NOTARIZATION="1")
        return proc, [json.loads(line) for line in calls.read_text().splitlines()]

    def test_framework_library_has_no_entitlements_but_python_host_does(self):
        app = self.bundle / "usdRigView.app"
        python = app / "Contents" / "Resources" / "app" / "python"
        library = python / "Python"
        host = python / "Resources" / "Python.app" / "Contents" / "MacOS" / "Python"
        stub = python / "bin" / "python3"
        framework_library = (app / "Contents" / "Frameworks" / "Python.framework" /
                             "Versions" / "3.11" / "Python")
        for path in (library, host, stub, framework_library):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(Path("/usr/bin/true").read_bytes())
        proc, calls = self._mock_notarization("Accepted")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        for path, entitled in ((library, False), (framework_library, False),
                               (host, True), (stub, True)):
            signing = [call for call in calls if call[0] == "codesign" and
                       "--sign" in call and call[-1] == str(path.resolve())]
            self.assertEqual(len(signing), 1, signing)
            self.assertEqual("--entitlements" in signing[0], entitled)

    def test_relative_bundle_signs_nested_python_app_before_framework_library(self):
        python = (self.bundle / "usdRigView.app" / "Contents" / "Resources" /
                  "app" / "python")
        library = python / "Python"
        nested = python / "Resources" / "Python.app"
        host = nested / "Contents" / "MacOS" / "Python"
        host.parent.mkdir(parents=True)
        for path in (host, library):
            path.write_bytes(Path("/usr/bin/true").read_bytes())
        proc, calls = self._mock_notarization("Accepted", relative_bundle=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        targets = [call[-1] for call in calls if call[0] == "codesign" and
                   "--sign" in call]
        self.assertLess(targets.index(str(host.resolve())),
                        targets.index(str(nested.resolve())))
        self.assertLess(targets.index(str(nested.resolve())),
                        targets.index(str(library.resolve())))
        self.assertTrue(any(call[0] == "codesign" and "--verify" in call and
                            Path(call[-1]).resolve() == library.resolve()
                            for call in calls))

    def test_accepted_notarization_staples_and_validates(self):
        proc, calls = self._mock_notarization("Accepted")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertTrue(any(call[:3] == ["xcrun", "stapler", "staple"]
                            for call in calls))
        self.assertTrue(any(call[:3] == ["xcrun", "stapler", "validate"]
                            for call in calls))
        self.assertFalse(any(call[:3] == ["xcrun", "notarytool", "log"]
                             for call in calls))

    def test_invalid_notarization_fetches_log_without_stapling(self):
        proc, calls = self._mock_notarization("Invalid")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("test notarization diagnostic", proc.stdout)
        self.assertIn("status: Invalid", proc.stderr)
        self.assertFalse(any(call[:2] == ["xcrun", "stapler"] for call in calls))

    def test_submit_error_fetches_available_log_without_stapling(self):
        proc, calls = self._mock_notarization("In Progress", submit_exit="75")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("test notarization diagnostic", proc.stdout)
        self.assertIn("submit exit: 75", proc.stderr)
        self.assertFalse(any(call[:2] == ["xcrun", "stapler"] for call in calls))

    def test_malformed_response_fails_without_stapling(self):
        proc, calls = self._mock_notarization("", malformed=True)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("notarization not accepted", proc.stderr)
        self.assertFalse(any(call[:2] == ["xcrun", "stapler"] for call in calls))
        self.assertFalse(any(call[:3] == ["xcrun", "notarytool", "log"]
                             for call in calls))

    def test_missing_app_is_an_error(self):
        empty = Path(self._tmp.name) / "empty"
        empty.mkdir()
        proc = run_sign(empty,
                        MACOS_SIGNING_IDENTITY="Developer ID Application: X (TEAM)")
        self.assertEqual(proc.returncode, 1)
        self.assertIn("no usdRigView.app", proc.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
