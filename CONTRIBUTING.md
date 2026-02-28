# Contributing to iMuJoCo

Thanks for your interest in contributing to iMuJoCo!

## Getting Started

1. Fork the repository and clone your fork.
2. Set up git hooks:
   ```bash
   git config core.hooksPath .githooks
   ```
3. Set up your Apple Developer Team ID for device builds:
   ```bash
   cp imujoco/app/team_config.bzl.template imujoco/app/team_config.bzl
   # Edit team_config.bzl with your TEAM_ID
   ```
4. Build and run tests:
   ```bash
   bazel build //...
   bazel test //driver:driver_test
   ```

## Commit Convention

We use [Conventional Commits](https://www.conventionalcommits.org/):

| Prefix       | Use                                  |
|--------------|--------------------------------------|
| `feat:`      | New feature                          |
| `fix:`       | Bug fix                              |
| `perf:`      | Performance improvement              |
| `refactor:`  | Code restructuring (no behavior change) |
| `docs:`      | Documentation only                   |
| `chore:`     | Build, CI, tooling changes           |
| `release:`   | Version bump / release               |

Include the scope when it's clear: `feat(render): add shadow mapping`.

## Branch Naming

Use descriptive branch names with a prefix:

- `feat/short-description` — new feature
- `fix/short-description` — bug fix
- `perf/short-description` — performance improvement
- `refactor/short-description` — refactoring
- `docs/short-description` — documentation

## Pull Request Process

1. Create a branch from `main`.
2. Make your changes, keeping commits focused and well-described.
3. Ensure your changes build and pass tests:
   ```bash
   bazel build //...
   bazel test //driver:driver_test
   ```
4. If you changed the build graph, regenerate the Xcode project:
   ```bash
   bazel run //:xcodeproj
   ```
5. Open a pull request against `main` and fill out the PR template.
6. Address review feedback.

## Code Style

- **Swift / Metal:** 2-space indent
- **C / C++ / ObjC++:** 2-space indent
- **Python:** 4-space indent
- **Starlark (Bazel):** 4-space indent

An `.editorconfig` is included in the repo — most editors will pick this up automatically.

### Naming Conventions

- `mjc_*` prefix for C/C++ code
- `MJ*` prefix for Swift types wrapping C++
- FlatBuffers file identifiers: `"CTPK"` (control), `"STPK"` (state)

## License

This project is licensed under the Apache License 2.0. By contributing, you agree that your contributions will be licensed under the same license.
