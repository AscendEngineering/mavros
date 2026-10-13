# Upstream PR notes

These are preparation notes, not a completed PR description.

## Dependency packaging and release

- Mention the new `libevents_vendor` ROS package included in this repository.
  MAVROS declares it in `package.xml` and links its exported parser target through
  `find_package(libevents_vendor REQUIRED)`.
- Explain that the wrapper uses `ExternalProject_Add` to download and build
  unmodified libevents, pinned to commit
  `0ef305faa4cffdb09f01ca862ebfc108b7ff0cd5` with SHA256 verification.
- State the CMake 3.19 minimum for the vendor package and the initial build's
  network requirement. Include the upstream BSD license and bundled
  nlohmann/json MIT license notices.
- Request maintainer review of the in-repository vendor package arrangement.
  Existing external dependencies do not use this arrangement; do not describe it
  as an established MAVROS packaging pattern.
- Call out that source-workspace builds can build the included package with
  colcon. Build and installed-consumer validation remain pending.
- Before a binary MAVROS release depends on it, include `libevents_vendor` in the
  relevant ROS distribution release metadata and ensure its binary packages are
  available. This is package registration/release, not a change to ROS itself.
- Do not imply that an APT package already exists or that decoder behavior has
  been implemented merely by adding this dependency.

## Local decoding stage

- Describe the disabled-by-default, restart-only `sys` parameters and local
  JSON/XZ support. Automatic acquisition and caching remain future work.
- Mention `liblzma-dev` for bounded XZ decoding and the additional installation
  of libevents' bundled JSON header; no second JSON revision is introduced.
- Preserve the raw StatusEvent interface and fallback; use full IDs internally.
- Report builds/tests actually run, separating the previously passed Lyrical
  dependency/installed-consumer checks from the new decoder and plugin tests.
- Regenerate the plugin reference using the repository tooling after an
  authorized build. Do not hand-edit generated reference files.
