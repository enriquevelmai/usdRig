#!/bin/bash
# Code-sign a usdRig macOS bundle (the usdRigView.app inside $1) and,
# when notary credentials are present, notarize and staple it.
#
#   packaging/sign-macos.sh dist/usdRig-1.2.0-macos-arm64
#
# Inputs (environment):
#   MACOS_SIGNING_IDENTITY   "Developer ID Application: Name (TEAMID)";
#                            unset -> warn and exit 0 (unsigned build)
#   AC_API_KEY_ID, AC_API_ISSUER_ID, AC_API_KEY_PATH
#                            App Store Connect API key; all three set ->
#                            the .app is zipped, notarized, and stapled
#
# Every Mach-O in the bundle is signed with the hardened runtime,
# innermost first, then the .app itself with the bundle entitlements.
# Any patching (install names, rpath, stripping) must happen before
# this runs: it invalidates signatures.
set -euo pipefail

bundle="${1:?usage: sign-macos.sh <bundle-dir>}"
app="$bundle/usdRigView.app"
entitlements="$(cd "$(dirname "$0")" && pwd)/usdRigView.entitlements"

if [ -z "${MACOS_SIGNING_IDENTITY:-}" ]; then
    echo "warning: MACOS_SIGNING_IDENTITY not set: bundle stays UNSIGNED" >&2
    exit 0
fi
test -d "$app" || { echo "no usdRigView.app in $bundle" >&2; exit 1; }

# Mach-O files, deepest first so nested code is signed before its container.
find "$app" -type f | while IFS= read -r file; do
    if file -b "$file" | grep -q "Mach-O"; then
        echo "$file"
    fi
done | awk '{ print length, $0 }' | sort -rn | cut -d' ' -f2- | \
while IFS= read -r file; do
    codesign --force --options runtime --timestamp \
        --sign "$MACOS_SIGNING_IDENTITY" "$file"
done

codesign --force --options runtime --timestamp \
    --entitlements "$entitlements" \
    --sign "$MACOS_SIGNING_IDENTITY" "$app"
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
    echo "notarized and stapled"
else
    echo "warning: App Store Connect key incomplete: signed but NOT notarized" >&2
fi
