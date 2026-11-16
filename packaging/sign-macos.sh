#!/bin/bash
# Code-sign a usdRig macOS bundle (the usdRigView.app inside $1) and,
# when notary credentials are present, notarize and staple it.
#
#   packaging/sign-macos.sh dist/usdRig-1.2.0-macos-arm64
#
# Inputs (environment):
#   MACOS_SIGNING_IDENTITY   "Developer ID Application: Name (TEAMID)";
#                            unset -> warn and exit 0 (unsigned build),
#                            unless RIGEXEC_REQUIRE_NOTARIZATION is 1
#   AC_API_KEY_ID, AC_API_ISSUER_ID, AC_API_KEY_PATH
#                            App Store Connect API key; all three set ->
#                            the .app is zipped, notarized, and stapled
#   RIGEXEC_REQUIRE_NOTARIZATION
#                            1 -> missing signing identity or notary
#                            credentials are an error, not a warning.
#                            The release workflow sets this for every
#                            published release, so a release can never
#                            ship unsigned or unnotarized.
#
# Every Mach-O and every nested bundle (.framework, nested .app, .xpc,
# .appex) in the bundle is signed with the hardened runtime, innermost
# first, then the .app itself with the bundle entitlements. Symlinked
# binaries (Python.framework, Qt frameworks) are signed once, at their
# real path. The bundled Python executables also get the entitlements:
# they host usdview, so they need the JIT / library-validation
# allowances, not just the launcher. Any patching (install names,
# rpath, stripping) must happen before this runs: it invalidates
# signatures.
set -euo pipefail

bundle="${1:?usage: sign-macos.sh <bundle-dir>}"
app="$bundle/usdRigView.app"
entitlements="$(cd "$(dirname "$0")" && pwd)/usdRigView.entitlements"
require="${RIGEXEC_REQUIRE_NOTARIZATION:-0}"

if [ -z "${MACOS_SIGNING_IDENTITY:-}" ]; then
    if [ "$require" = "1" ]; then
        echo "error: MACOS_SIGNING_IDENTITY not set: refusing to" \
            "publish an UNSIGNED bundle" >&2
        exit 1
    fi
    echo "warning: MACOS_SIGNING_IDENTITY not set: bundle stays UNSIGNED" >&2
    exit 0
fi
test -d "$app" || { echo "no usdRigView.app in $bundle" >&2; exit 1; }

sign_plain() {
    codesign --force --options runtime --timestamp \
        --sign "$MACOS_SIGNING_IDENTITY" "$1"
}
sign_entitled() {
    codesign --force --options runtime --timestamp \
        --entitlements "$entitlements" \
        --sign "$MACOS_SIGNING_IDENTITY" "$1"
}

# Executables that host usdview need the entitlements, not just the
# launcher: the bundled Python interpreter (bin/python* and the
# Python.framework binary) dlopens the plug-in libraries and runs Qt.
needs_entitlements() {
    case "$1" in
        */app/python/bin/python*) return 0 ;;
        */app/python/Python) return 0 ;;
        */app/python/Resources/Python.app/Contents/MacOS/Python) return 0 ;;
        */Python.framework/Versions/*/Python) return 0 ;;
        *.app) return 0 ;;
        *) return 1 ;;
    esac
}

# Collect every signable, innermost first. Regular Mach-O files and
# symlinks to them (signed once, at the resolved path), plus nested
# bundle directories, which must be signed as bundles to seal their
# resources. Sorting by descending path length signs nested code
# before the bundle that contains it.
{
    app_real="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$app")"
    find "$app" \( -type f -o -type l \) | while IFS= read -r file; do
        resolved="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$file")"
        if [ -f "$resolved" ] && file -b "$resolved" | grep -q "Mach-O"; then
            case "$resolved" in
                "$app_real"/*) echo "$resolved" ;;
                *) echo "error: Mach-O symlink escapes the bundle:" \
                       "$file -> $resolved" >&2; exit 1 ;;
            esac
        fi
    done
    find "$app" -type d \( -name "*.framework" -o -name "*.app" \
        -o -name "*.xpc" -o -name "*.appex" \) | while IFS= read -r dir; do
        if [ "$dir" != "$app" ]; then
            echo "$dir"
        fi
    done
} | sort -u | awk '{ print length, $0 }' | sort -rn | cut -d' ' -f2- | \
while IFS= read -r target; do
    if needs_entitlements "$target"; then
        sign_entitled "$target"
    else
        sign_plain "$target"
    fi
done

sign_entitled "$app"
codesign --verify --deep --strict --verbose=2 "$app"

if [ -n "${AC_API_KEY_ID:-}" ] && [ -n "${AC_API_ISSUER_ID:-}" ] && \
   [ -n "${AC_API_KEY_PATH:-}" ]; then
    zip_file="$(mktemp -d)/usdRigView.zip"
    ditto -c -k --keepParent "$app" "$zip_file"
    xcrun notarytool submit "$zip_file" \
        --key "$AC_API_KEY_PATH" --key-id "$AC_API_KEY_ID" \
        --issuer "$AC_API_ISSUER_ID" --wait
    xcrun stapler staple "$app"
    xcrun stapler validate "$app"
    spctl -a -t exec -vvv "$app"
    echo "notarized and stapled"
elif [ "$require" = "1" ]; then
    echo "error: App Store Connect key incomplete: refusing to" \
        "publish a signed but NOT notarized bundle" >&2
    exit 1
else
    echo "warning: App Store Connect key incomplete: signed but NOT notarized" >&2
fi
