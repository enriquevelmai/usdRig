# Release bundles

Every `v<major>.<minor>.0` tag builds a self-contained usdRig download for
Windows, macOS (Apple Silicon), and Linux (Ubuntu x86-64) and attaches it
to a GitHub Release: `.zip` on Windows, `.tar.gz` elsewhere, plus a
combined `SHA256SUMS.txt`. A user extracts the archive and runs the
launcher — no OpenUSD, Python, or Qt install is needed and nothing is
written outside the extracted directory.

- Workflow: `.github/workflows/release.yml`
- Launcher: `packaging/launcher/usdRigView.cpp` (built as `usdRigView`)
- Assembly: `packaging/assemble_bundle.py` (stdlib only)
- Tests: `packaging/test_assemble_bundle.py` (ctest:
  `testRigExecBundlePackaging`, needs `-DRIGEXEC_BUILD_VIEWER_LAUNCHER=ON`)

## Bundle layout

Windows/Linux: `usdRigView[.exe]` beside an `app/` tree. macOS:
`usdRigView.app` with the launcher in `Contents/MacOS` and the same
`app/` tree in `Contents/Resources/app`, so the bundle signs and
notarizes as one unit. Inside `app/`:

| Entry | Contents |
|---|---|
| `usd/` | complete OpenUSD install (`bin/usdview`, libraries, `pxr`) |
| `python/` | CPython home OpenUSD was compiled against |
| `rig/` | RigExec `cmake --install` prefix, plus CLI tools in `rig/bin` |
| `plugins/` | `touchPose`, `shapeEditor` usdview plug-ins |
| `examples/` | `rigexec_flat.usda`, `ArmShotAnim.usda` |

The launcher derives all of these paths from its own location, sets
`PYTHONPATH`, `PXR_PLUGINPATH_NAME`, `PATH`, and the platform loader
path, then runs `app/usd/bin/usdview` with the bundled Python.
`usdRigView --usdRig-check` verifies the bundle without starting the
viewer; `--usdRig-print-env` shows the computed environment.

## Cutting a release

1. Bump `project(rigExec VERSION ...)` in `CMakeLists.txt` to match.
   The workflow's gate job refuses a `vX.Y.0` tag that disagrees.
2. Tag and push: `git tag v0.3.0 && git push origin v0.3.0`.
3. The release appears when all three platform jobs finish. Patch tags
   (`v0.3.1`) do not trigger bundles.

`workflow_dispatch` builds the same artifacts without publishing:
give it a `vX.Y.0` version, and set `publish` only when the run should
create/update the Release for that tag.

## Signing

Signing happens on the platform runner, after assembly, before
archiving; each script warns and skips when its credential is absent, so
forks and test runs produce clearly unsigned bundles.

| Platform | Mechanism | Secrets |
|---|---|---|
| Windows | Authenticode (`packaging/sign-windows.ps1`) on every `.exe`/`.dll`/`.pyd`, then the archive is listed in the signed checksums | `WINDOWS_SIGNING_CERT_BASE64`, `WINDOWS_SIGNING_CERT_PASSWORD`, optional `WINDOWS_SIGNING_THUMBPRINT` |
| macOS | `codesign` (hardened runtime) on every Mach-O, then notarization + stapling (`packaging/sign-macos.sh`) | `MACOS_SIGNING_IDENTITY`, `MACOS_CERTIFICATE_BASE64`, `MACOS_CERTIFICATE_PASSWORD`, `AC_API_KEY_ID`, `AC_API_ISSUER_ID`, `AC_API_KEY_BASE64` |
| Linux | No per-`.so` OS signing exists; the publish job GPG-detach-signs `SHA256SUMS.txt` when a key is configured | `LINUX_GPG_PRIVATE_KEY`, `LINUX_GPG_PASSPHRASE` |

## Building a bundle by hand

Given an OpenUSD install built with usdview + Python, a RigExec build,
and that build's Python home:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR=$USD -DCMAKE_PREFIX_PATH=$USD \
  -DRIGEXEC_BUILD_VIEWER_LAUNCHER=ON
cmake --build build --target usdRigView rigExecPose rigExecBake rigExecReplayInputs
cmake --install build --prefix rig-install
python3 packaging/assemble_bundle.py \
  --usd-install $USD --rig-install rig-install \
  --python-home /path/to/python-home \
  --launcher build/usdRigView --output dist/usdRig-0.3.0-linux-x64 \
  --version v0.3.0 --platform linux --platform-slug linux-x64 \
  --source-root . --rig-build-dir build --trim-dev \
  --archive dist/usdRig-0.3.0-linux-x64.tar.gz
dist/usdRig-0.3.0-linux-x64/usdRigView --usdRig-check
```

`--trim-dev` drops headers, import/static libraries, and CMake package
files; keep it for release archives (GitHub caps a release asset at
2 GiB) and drop it only for a developer bundle.
