# Provisioning Profiles

This directory holds Apple provisioning profiles (`.mobileprovision`) used for
code signing during App Store builds. These files are **gitignored** because
they contain team-specific credentials.

## Setup

1. Go to [Apple Developer Portal](https://developer.apple.com/account/resources/profiles/list)
   → Certificates, Identifiers & Profiles → Profiles.
2. Create (or download) an **App Store Connect** distribution profile for
   bundle ID `com.hhkblogi.imujoco.app`.
3. Copy the `.mobileprovision` file into this directory:
   ```
   cp ~/Library/Developer/Xcode/UserData/Provisioning\ Profiles/<UUID>.mobileprovision imujoco/app/profiles/
   ```
4. Update `imujoco/app/team_config.bzl` with the profile name (see `team_config.bzl.template`).
