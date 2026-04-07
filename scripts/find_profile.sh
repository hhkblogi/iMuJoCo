#!/bin/bash
# find_profile.sh — Find a provisioning profile by bundle ID and team ID.
# Usage: find_profile.sh <team_id> <bundle_id> <output_path> [type]
#   type: "dev" (default) or "appstore"
#
# Searches both Xcode-managed and manually-installed profile directories.
# Matches by application-identifier (team_id.bundle_id) in the profile's
# entitlements, not by profile name. For dev profiles, also accepts
# wildcard Xcode-managed profiles (team_id.*), preferring exact matches.
#
# Profile type is detected by the presence of ProvisionedDevices key:
#   - Development profiles have ProvisionedDevices + get-task-allow=true
#   - App Store distribution profiles do NOT have ProvisionedDevices
#
# When multiple profiles match, the one expiring latest is chosen
# (most recently renewed), giving deterministic results.

set -eo pipefail

TEAM_ID="$1"
BUNDLE_ID="$2"
OUTPUT="$3"
TYPE="${4:-dev}"
EXPECTED_APPID="${TEAM_ID}.${BUNDLE_ID}"
WILDCARD_APPID="${TEAM_ID}.*"

USER_HOME="${HOME:-$(eval echo ~)}"

PROFILE_DIRS=(
    "$USER_HOME/Library/Developer/Xcode/UserData/Provisioning Profiles"
    "$USER_HOME/Library/MobileDevice/Provisioning Profiles"
)

# Collect SHA-1 fingerprints of installed signing identities (certs with private keys)
INSTALLED_FPS=$(security find-identity -v -p codesigning 2>/dev/null | \
    awk '{ print $2 }' | grep -E '^[A-F0-9]{40}$' || true)

# Best candidates, split by exact vs wildcard match (cert-matched and fallback).
# When multiple profiles match, the one expiring latest wins.
# Priority: cert-exact > cert-wildcard > fallback-exact > fallback-wildcard.
BEST_EXACT=""
BEST_EXACT_EXPIRY=0
BEST_WILD=""
BEST_WILD_EXPIRY=0
FALLBACK_EXACT=""
FALLBACK_EXACT_EXPIRY=0
FALLBACK_WILD=""
FALLBACK_WILD_EXPIRY=0

for dir in "${PROFILE_DIRS[@]}"; do
    [ -d "$dir" ] || continue
    for f in "$dir"/*.mobileprovision "$dir"/*.provisionprofile; do
        [ -f "$f" ] || continue
        decoded=$(security cms -D -i "$f" 2>/dev/null) || continue
        appid=$(printf '%s' "$decoded" | \
            xmllint --xpath '//key[text()="application-identifier"]/following-sibling::string[1]/text()' - 2>/dev/null) || continue
        # Match exact bundle ID, or wildcard (TEAMID.*) for dev profiles.
        is_wildcard=false
        if [ "$appid" = "$EXPECTED_APPID" ]; then
            : # exact match
        elif [ "$appid" = "$WILDCARD_APPID" ] && [ "$TYPE" = "dev" ]; then
            is_wildcard=true
        else
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
        # Extract expiration date (epoch) and cert fingerprints in one pass.
        eval_output=$(printf '%s' "$decoded" | python3 -c '
import sys, plistlib, hashlib
try:
    d = plistlib.loads(sys.stdin.buffer.read())
    exp = d.get("ExpirationDate")
    print(int(exp.timestamp()) if exp else 0)
    for cert_der in d.get("DeveloperCertificates", []):
        print(hashlib.sha1(cert_der).hexdigest().upper())
except Exception:
    print(0)
' 2>/dev/null)
        exp_epoch=$(echo "$eval_output" | head -1)
        profile_fps=$(echo "$eval_output" | tail -n +2)

        # Check if any cert in the profile matches an installed signing identity.
        cert_matched=false
        for pfp in $profile_fps; do
            if echo "$INSTALLED_FPS" | grep -q "$pfp"; then
                cert_matched=true
                break
            fi
        done
        if [ "$cert_matched" = true ]; then
            if [ "$is_wildcard" = true ]; then
                if [ "$exp_epoch" -gt "$BEST_WILD_EXPIRY" ] 2>/dev/null; then
                    BEST_WILD="$f"
                    BEST_WILD_EXPIRY="$exp_epoch"
                fi
            else
                if [ "$exp_epoch" -gt "$BEST_EXACT_EXPIRY" ] 2>/dev/null; then
                    BEST_EXACT="$f"
                    BEST_EXACT_EXPIRY="$exp_epoch"
                fi
            fi
        else
            if [ "$is_wildcard" = true ]; then
                if [ "$exp_epoch" -gt "$FALLBACK_WILD_EXPIRY" ] 2>/dev/null; then
                    FALLBACK_WILD="$f"
                    FALLBACK_WILD_EXPIRY="$exp_epoch"
                fi
            else
                if [ "$exp_epoch" -gt "$FALLBACK_EXACT_EXPIRY" ] 2>/dev/null; then
                    FALLBACK_EXACT="$f"
                    FALLBACK_EXACT_EXPIRY="$exp_epoch"
                fi
            fi
        fi
    done
done

# Priority: cert-exact > cert-wildcard > fallback-exact > fallback-wildcard
for candidate in "$BEST_EXACT" "$BEST_WILD"; do
    if [ -n "$candidate" ]; then
        cp "$candidate" "$OUTPUT"
        exit 0
    fi
done

for candidate in "$FALLBACK_EXACT" "$FALLBACK_WILD"; do
    if [ -n "$candidate" ]; then
        echo "WARNING: No $TYPE profile found whose cert matches an installed identity." >&2
        echo "  Using fallback: $candidate" >&2
        cp "$candidate" "$OUTPUT"
        exit 0
    fi
done

echo "ERROR: No $TYPE provisioning profile found for $EXPECTED_APPID" >&2
echo "  Install a $TYPE profile for bundle ID '$BUNDLE_ID' (team $TEAM_ID)" >&2
exit 1
