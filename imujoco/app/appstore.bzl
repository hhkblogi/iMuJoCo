"""Conditionally registers App Store distribution targets."""

load("@rules_apple//apple:ios.bzl", "ios_application")

def appstore_targets(name = "appstore", appstore_identity = "", team_id = "", app_bundle_id = "", find_profile_tool = ""):
    """Creates App Store distribution targets when APPSTORE_IDENTITY is configured.

    When APPSTORE_IDENTITY is empty (the default), no distribution targets
    are created. This allows `bazel build //...` to work for developers
    who only need dev builds.

    Args:
        name: Macro name (unused, required by Bazel convention).
        appstore_identity: Signing identity string (e.g. "Apple Distribution: Name (TEAM)").
                           When empty, no targets are created.
        team_id: Apple Developer Team ID.
        app_bundle_id: App bundle identifier (e.g. "com.hhkblogi.imujoco.app").
        find_profile_tool: Label for the find_profile.sh script.
    """
    _ = name  # unused, required by convention
    if not appstore_identity:
        return

    native.genrule(
        name = "appstore_profile",
        outs = ["iMuJoCo_AppStore.mobileprovision"],
        cmd = "$(location " + find_profile_tool + ") " + team_id + " " + app_bundle_id + " $@ appstore",
        tools = [find_profile_tool],
        local = True,
        tags = [
            "manual",
            "no-cache",
        ],
    )

    ios_application(
        name = "app_ios_appstore",
        app_icons = native.glob(["app/AppIcon.icon/**"]),
        bundle_id = app_bundle_id,
        entitlements = "app_ios.entitlements",
        families = [
            "iphone",
            "ipad",
        ],
        infoplists = ["Info.plist"],
        minimum_os_version = "26.0",
        provisioning_profile = ":appstore_profile",
        resources = native.glob(["app/Assets.xcassets/**"]) + [
            "app/PrivacyInfo.xcprivacy",
            "//models",
            "//imujoco/render:mujoco_metallib",
        ],
        tags = ["manual"],
        visibility = ["//visibility:public"],
        deps = [":app_lib"],
    )
