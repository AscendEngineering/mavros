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

- Describe the disabled-by-default, restart-only `sys` parameters, local
  JSON/XZ support, automatic acquisition, and optional persistent caching.
- Mention `xz_embedded_vendor` for bounded XZ decoding and the additional
  installation of libevents' bundled JSON header; no second JSON revision is
  introduced. The decoder pins XZ Embedded commit
  `ae63ae3a36ed01724674e8f3d750dc47bf125410` (0BSD), verifies the source archive,
  and builds unchanged upstream C sources with a small CMake wrapper.
- Explain why XZ Embedded single-call mode replaces liblzma at runtime: output
  storage doubles as the dictionary, avoiding an independent 64 MiB dictionary
  allocation for the measured PX4 preset-9 metadata. The XZ index determines the
  exact bounded output allocation; complete decoding and integrity validation
  remain mandatory. No decompression-memory estimate is a whole-process limit.
- Request review and ROS registration/release of `xz_embedded_vendor` alongside
  `libevents_vendor`. It is not assumed to have an existing binary ROS package.
- `liblzma-dev` temporarily remains test-only for existing fixture encoders;
  removing those uses and adding XZ Embedded/index regression tests are a
  separate, pending patch. Do not claim replacement build/runtime validation
  until it has actually run.
- Preserve the raw StatusEvent interface and fallback; use full IDs internally.
- Explain compact bitfield output: only PX4's `navigation_mode_group_t` argument
  named `modes` uses `all` for an all-bits-set applicability mask. Other masks
  retain named flags; undefined set bits appear once as `unknown_bits=0x...`.
  This does not interpret health-check applicability as universal mode denial.
- Report builds/tests actually run, separating the previously passed Lyrical
  dependency/installed-consumer checks from the new decoder and plugin tests.
- Regenerate the plugin reference using the repository tooling after an
  authorized build. Do not hand-edit generated reference files.

## User-reported validation (2026-10-16)

These results were reported by the contributor during bench testing; they are
not independently executed checks by the coding agent.

- Unit tests pass after updating the disconnect assertion to account for the
  expected acquisition-cancellation warning. The associated rcutils diagnostic
  was reported resolved.
- Real Seeker automatic acquisition: cold cache downloads definitions and
  enables decoded logging; a subsequent warm run reuses cached definitions.
- Cache fallback checks succeeded for missing/corrupt event definitions with
  valid general metadata, missing/corrupt general metadata with valid event
  definitions, and both entries corrupt. Acquisition recovers and decoding works.
- An unusable cache directory (`/dev/null/px4_events`, an explicit temporary
  parameter override) produces persistence failures while automatic acquisition
  and decoded logging continue.
- The contributor reports performing the above runtime cache checks with
  ASan/UBSan enabled. Preserve the distinction between successful feature
  behavior and a sanitizer-clean process: previously reproduced upstream packed
  alignment findings and the shutdown use-after-free remain separate findings.
- One failed fallback run had two MAVROS instances running simultaneously;
  FTP sequence mismatches occurred. The cache fallback checks subsequently
  succeeded with one instance. Concurrent FTP acquisition by multiple instances
  is not established by these results.

Remaining before opening the PR: local-file runtime checks, default-disabled
behavior, and the remaining disconnect/reconnect lifecycle checks. Then review
the complete diff, consolidate validation evidence, and finish the PR description.
