#!/bin/bash
# find_profile.sh — Find a provisioning profile by bundle ID and team ID.
# Usage: find_profile.sh <team_id> <bundle_id> <output_path> [type]
#   type: "dev" (default) or "appstore"
#
# Searches both Xcode-managed and manually-installed profile directories.
# Matches by application-identifier (team_id.bundle_id) in the profile's
# entitlements, not by profile name.
#
# Profile type is detected by the presence of ProvisionedDevices key:
#   - Development profiles have ProvisionedDevices + get-task-allow=true
#   - App Store distribution profiles do NOT have ProvisionedDevices

set -eo pipefail

TEAM_ID="$1"
BUNDLE_ID="$2"
OUTPUT="$3"
TYPE="${4:-dev}"
EXPECTED_APPID="${TEAM_ID}.${BUNDLE_ID}"

USER_HOME="${HOME:-$(eval echo ~)}"

PROFILE_DIRS=(
    "$USER_HOME/Library/Developer/Xcode/UserData/Provisioning Profiles"
    "$USER_HOME/Library/MobileDevice/Provisioning Profiles"
)

# Collect SHA-1 fingerprints of installed signing identities (certs with private keys)
INSTALLED_FPS=$(security find-identity -v -p codesigning 2>/dev/null | \
    awk '{ print $2 }' | grep -E '^[A-F0-9]{40}$' || true)

FALLBACK=""

for dir in "${PROFILE_DIRS[@]}"; do
    [ -d "$dir" ] || continue
    for f in "$dir"/*.mobileprovision "$dir"/*.provisionprofile; do
        [ -f "$f" ] || continue
        decoded=$(security cms -D -i "$f" 2>/dev/null) || continue
        appid=$(printf '%s' "$decoded" | \
            xmllint --xpath '//key[text()="application-identifier"]/following-sibling::string[1]/text()' - 2>/dev/null) || continue
        if [ "$appid" != "$EXPECTED_APPID" ]; then
            continue
        fi
        # Filter by profile type:
        #   dev:      has ProvisionedDevices AND get-task-allow=true
        #   appstore: no ProvisionedDevices (Ad Hoc profiles have ProvisionedDevices
        #             but get-task-allow=false, so they're excluded from "dev")
        if printf '%s' "$decoded" | grep -q '<key>ProvisionedDevices</key>'; then
            if printf '%s' "$decoded" | grep -A1 '<key>get-task-allow</key>' | grep -q '<true/>'; then
                profile_type="dev"
            else
                profile_type="adhoc"
            fi
        else
            profile_type="appstore"
        fi
        if [ "$profile_type" != "$TYPE" ]; then
            continue
        fi
        # Prefer profiles whose cert matches an installed signing identity.
        profile_fps=$(printf '%s' "$decoded" | python3 -c '
import sys, plistlib, hashlib
try:
    d = plistlib.loads(sys.stdin.buffer.read())
    for cert_der in d.get("DeveloperCertificates", []):
        print(hashlib.sha1(cert_der).hexdigest().upper())
except Exception:
    pass
' 2>/dev/null)
        matched=false
        for pfp in $profile_fps; do
            if echo "$INSTALLED_FPS" | grep -q "$pfp"; then
                matched=true
                break
            fi
        done
        if [ "$matched" = true ]; then
            cp "$f" "$OUTPUT"
            exit 0
        fi
        # Keep as fallback if no cert-matching profile found
        [ -z "$FALLBACK" ] && FALLBACK="$f"
    done
done

if [ -n "$FALLBACK" ]; then
    echo "WARNING: No $TYPE profile found whose cert matches an installed identity." >&2
    echo "  Using fallback: $FALLBACK" >&2
    cp "$FALLBACK" "$OUTPUT"
    exit 0
fi

echo "ERROR: No $TYPE provisioning profile found for $EXPECTED_APPID" >&2
echo "  Install a $TYPE profile for bundle ID '$BUNDLE_ID' (team $TEAM_ID)" >&2
exit 1
