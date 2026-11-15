#!/usr/bin/env python3
"""Tests for packaging/sign-macos.sh's signing-requirement gate.

Published releases must never ship unsigned or unnotarized, while
local/test builds may. The gate runs before any codesign call, so it
is exercised here with synthetic bundle dirs and no credentials.

Run directly:  python3 packaging/test_sign_macos.py
Or via ctest:  testRigExecMacSigning
"""

from __future__ import annotations

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

    def test_missing_app_is_an_error(self):
        empty = Path(self._tmp.name) / "empty"
        empty.mkdir()
        proc = run_sign(empty,
                        MACOS_SIGNING_IDENTITY="Developer ID Application: X (TEAM)")
        self.assertEqual(proc.returncode, 1)
        self.assertIn("no usdRigView.app", proc.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
