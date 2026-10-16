# PX4 event decoding plan

Status: agreed design with implementation details still open. This document is
a development plan, not a description of functionality already shipped.

## Purpose and scope

Decode PX4 MAVLink EVENT messages inside MAVROS using libevents and the event
definitions for the connected firmware. Replace the raw event log line with
human-readable text when decoding succeeds. Preserve existing raw logging when
decoding is disabled or unavailable.

Implement this for upstream MAVROS. Keep changes small and reviewable, follow
existing plugin and asynchronous service patterns, and avoid a separate MAVLink
connection or a new global startup scheduler.

ArduPilot support, Neuron consumption, and health/arming sequence aggregation
are outside this initial change. Neuron integration can follow after the ROS
interface has been finalized and reviewed.

## Agreed configuration

The following identifiers are planned, not existing public parameters.

| Setting | Agreed behavior |
| --- | --- |
| Plugin ownership | Configuration belongs under the system-status plugin node, normally `/mavros/sys`. |
| `event_decoding_mode` | `disabled`, `local`, or `auto`; default `disabled`. |
| Parameter changes | Require a restart. Reject runtime changes to decoding configuration with an explanatory reason. |
| `event_metadata_file` | Accept a local JSON or XZ-compressed JSON path in `local` mode only. A nonempty path in another mode is invalid. |
| Automatic source | Use firmware-advertised metadata locations and identity. Do not substitute definitions from current PX4 master or an unrelated local file. |
| Download delay | Wait 30 seconds after FCU connection before a full automatic download, after the fixed mission/rally/geofence startup pulls at 15/20/25 seconds. This staggers optional work but does not guarantee an idle FCU. The delay does not apply to cache validation. |
| `event_metadata_download_delay` | Default `30.0` seconds, finite and nonnegative, read-only after startup. Applies to automatic FTP acquisition; zero permits starting as soon as discovery succeeds. |
| Transport | MAVLink FTP only initially; no HTTP/HTTPS downloads, including advertised fallbacks. Report unsupported advertised transports and retain raw logging. |
| Transfer pacing | No added pacing. Only one request may be outstanding; send the next read immediately after the previous successful response. No read-interval parameter is planned. |
| Transfer and recovery controls | One retry after explicit EBUSY per operation, with bounded backoff. Bound service waits, file sizes, progress, and complete acquisition attempts; no fixed total acquisition deadline. Stop on uncertain failures. |
| `event_metadata_cache_enabled` | Default `true`; applies to automatic mode. Disabled decoding adds no cache-related network traffic. |
| `event_metadata_cache_dir` | Empty selects `$HOME/.cache/mavros/px4_events`; an explicit directory overrides it. No XDG override. If HOME is unavailable, report the problem and continue without persistence. |
| `event_metadata_cache_max_bytes` | Default `16777216` (16 MiB), restart-only. Best-effort bound on finalized entry content; not a filesystem quota. Zero/negative values report a configuration error and disable only persistence. A valid positive limit smaller than an entry skips saving that entry without disabling decoding. |

Reject invalid decoding configuration without stopping the MAVROS vehicle
connection: report a clear configuration error, leave decoding inactive, and
continue existing raw event logging. Examples include an unknown mode, a
missing required local path, and an unreadable or invalid local metadata file.
Do not repeatedly emit the same setup error for every event.

Disabled mode must introduce no additional metadata discovery or download
traffic. Automatic acquisition must remain asynchronous; metadata work must
not block event handling or ordinary vehicle callbacks.

## Agreed output behavior

- Log one readable event line after successful decoding; do not also log the
  original raw EVENT line for that same event.
- Use the original full EVENT ID internally, before MAVROS removes its namespace
  bits for the existing ROS representation.
- Preserve the existing raw `StatusEvent` topic, message fields, and publication
  behavior. Initial decoded output is logging only, without a new readable ROS
  topic.
- Retain raw logging for events received before metadata is ready, unknown IDs,
  unsupported metadata, and decoding failures. Do not buffer and replay early
  events in the initial implementation.
- Use the short event message. Do not append the optional longer description
  to routine log lines; descriptions can contain troubleshooting guidance and
  links unsuitable for single-line logs.
- Use a fixed developer formatter profile and render plain text. Metadata
  profile tags select audience-specific text; they do not select log severity.
- Preserve named argument information when the message itself does not express
  it, particularly for protocol and summary events. Avoid duplicating ordinary
  arguments already represented in the formatted message.
- Preserve numeric information for unknown enum values or bitfield bits.
- Keep protocol/debug information available at DEBUG severity rather than
  presenting protocol summaries as user warnings.
- Preserve existing ordinary-event severity behavior. Confirm exact treatment
  of the protocol's Disabled severity during decoder integration.

Use libevents for argument decoding and message formatting. Do not add a second
custom implementation of its numeric, enum, or bitfield parsing.

## Lifecycle, cache, and resource boundaries

Prefer cached definitions to repeated full downloads, but validate them against
the connected firmware's advertised metadata identity before activation.
Connecting to the same vehicle is insufficient proof: firmware can change.

On disconnect, cancel pending acquisition, invalidate active definitions, and
retain the persistent cache. On reconnect, validate the advertised identity and
reuse matching cached definitions. Cache validation is not subject to the
30-second full-download delay when no download is needed.

Cache both the general metadata file and the event definitions. Request the
firmware's metadata advertisement immediately after connection. If its advertised
general-file checksum matches the validated cached general file, reuse that file
without FTP. Its event URI and checksum identify the cached event definitions.
Validate the cached event file against that identity before activation. Download
only missing or changed files. Do not assume vehicle identity alone proves that
firmware metadata is unchanged.

Persist only checksum-validated, successfully parsed metadata, using atomic file
writes. Corrupt or mismatched cache entries must not be activated; reacquire the
necessary files under the normal acquisition policy. If disk persistence fails,
warn clearly and continue decoding with validated definitions in memory.

Bound finalized entry sizes using the configurable limit above. Scan once per
save, sort entries oldest-saved first, and delete one at a time until the
snapshot estimate fits. Count bytes freed only after successful removal. Ignore
unrelated files, symlinks, directories, and temporary files. Concurrent saves may
temporarily exceed the limit or cause extra cache misses; the next save rechecks.
No interprocess lock or download reservation is introduced. Duplicate downloads
are acceptable, and active in-memory definitions are unaffected by eviction.

Each versioned cache entry contains the full identity (role, original URI, and
advertised checksum) plus original JSON/XZ bytes in one file. Deterministic
filenames use role, a URI hash, and the advertised checksum; always compare the
full stored identity, so filename collisions cause misses rather than reuse of
unrelated definitions. Revalidate checksum and parsing on every load.

Publish with a unique owner-only `mkstemp` file in the cache directory, checked
writes/close, and same-directory atomic rename. Readers never consume temporary
files. Remove our temporary file on ordinary failures; do not delete another
instance's temporary files. A process crash can leave an ignored temporary file.
Atomic publication does not promise power-loss durability; invalid entries are
misses and can be reacquired. No fsync or global cache coordination is planned.

Keep cache settings under the same plugin node and require restart for changes.
Container deployments must persist the configured cache directory with a volume
or bind mount if cache reuse is expected after container replacement.

A complete cache hit requires only the metadata advertisement exchange and
local validation/loading, not a full FTP download or the startup download delay.
Early events arriving before validation remain raw and are not replayed.

MAVROS parameter values are a different concern: the parameter plugin clears
them on disconnect and schedules another pull after reconnect. They are mutable
vehicle state, whereas event definitions describe firmware. Do not change the
parameter plugin or attach metadata acquisition to its completion. The
30-second delay provides a head start, not a guarantee that parameter loading
has finished.

Bound download size, decompressed metadata size, and cache resource use. Begin
with generous limits and reduce them if evidence warrants it; exact values and
which limits are configurable remain open.

## Agreed transfer and recovery

Use reads of at most 239 bytes, with no burst reads initially. Allow only one
outstanding request. Send the next read immediately after the previous response
has completed successfully, matching normal MAVROS FTP behavior. The previously
proposed 250 ms/100 ms pacing and read-interval parameter are removed by user
agreement. Keep the 30-second cold-download delay; cache hits avoid the download
and that wait. Validate telemetry/command responsiveness and concurrent FTP use
on the vehicle before upstream submission; one outstanding request is not a
bandwidth reservation or a guarantee of fair link sharing.

Use bounded retry/recovery with at most three complete acquisition attempts,
waiting 2 seconds before the second attempt and 5 seconds before the third.
No fixed total acquisition deadline is introduced. Bound service availability
and response waits, file sizes, and progress instead. Successful reads must
return at least one byte without exceeding the request or advertised remaining
size. After exhausting the attempt budget, retain raw logging and stop automatic
attempts until the next connection. Uncertain failures stop acquisition without
starting another attempt. Do not reset or seize other users' FTP sessions as a
recovery mechanism. The combined acquisition helper owns both per-operation
retries and the outer three-attempt policy.

| Failure | Agreed recovery behavior |
| --- | --- |
| Explicit EBUSY | Retry the same operation once after backoff, without resetting or seizing another session. A successful read starts a fresh per-operation budget for the next offset. |
| Other confirmed read failure | Fail the download, discard partial bytes, and attempt close only when an open succeeded and operation state is not uncertain. |
| Session/protocol errors classified as uncertain by the downloader | Discard the partial download and stop acquisition for this connection; do not reopen or issue recovery requests. |
| Timeout or sequence mismatch | Stop acquisition with uncertain transport state. Do not retry, close, or reset immediately. Removing a ROS client pending request does not cancel the server-side operation. |
| Missing file or unsupported transport | Stop acquisition for this connection and retain raw logging. |
| Checksum mismatch | Do not activate metadata; permit a fresh attempt within the complete acquisition budget. |
| Failed or timed-out close | Report cleanup separately. Complete bytes remain eligible for checksum/parsing validation and decoder activation. An unresolved close is not permission to issue further recovery requests. |

The existing MAVROS FTP plugin has internal deadlines and late-response/reset
behavior. A longer ROS client wait does not extend those internal deadlines.
Leave the existing late-response/session-reset behavior unchanged. Our client
does not guarantee isolation from it. Verify behavior across supported PX4
versions before claiming that recovery cannot affect competing transfers. This
does not authorize unrelated FTP refactoring, startup reservations, or disabling
external FTP consumers.

## Open decisions

Resolve these in the relevant stage rather than silently selecting defaults:

1. Startup delay configuration: exact parameter name and validation bounds.
   The default is 30 seconds. No read-interval parameter is planned.
2. Outer acquisition integration: implement the three-attempt policy without
   retrying uncertain failures. The file helper has one EBUSY retry per operation,
   two-second backoff, and ten-second service waits. No total deadline is planned;
   FTP's own wire-operation deadlines and late-response behavior are unchanged.
3. Cache implementation: implement the agreed directory, identity, single-entry
   storage, limits, atomic publication, and lock-free best-effort eviction above.
   Then integrate acquisition, startup/reconnect handling, parameters, and docs
   as separate reviewable patches.
4. Local resource limits are fixed at 1 MiB compressed input and 8 MiB JSON.
   XZ Embedded single-call decoding uses exact-sized output as its dictionary;
   there is no separate allocation driven by the advertised dictionary size.
   Validate the XZ index before allocation and require stream end, exact output,
   complete input consumption, and integrity verification afterward. The file
   helper reuses 1 MiB for referenced XZ files and 8 MiB for plain JSON.
   Cache storage limits and automatic limit configurability remain open.
5. Dependency verification: the sibling `libevents_vendor` package and pinned
   revision are recorded below. Dependency build/install/consumer validation
   passed on Lyrical; other distributions and ROS binary release registration
   remain pending. No optional build gate is introduced.
6. Metadata discovery uses current `COMPONENT_METADATA`, with legacy
   `COMPONENT_INFORMATION` only on explicit `DENIED` or `UNSUPPORTED`. Each
   format has at most three total sends; silence does not select legacy.
   Runtime integration and tests are added in stage 4 patch B below; vehicle
   validation remains pending.
7. Local log formatting follows the agreed dev profile: short single-line text,
   unused named arguments in brackets, and event-name fallback for an empty
   message. Successfully decoded Disabled severity (8) maps to DEBUG; raw
   fallback and ordinary severity mapping are unchanged. The user reports the
   focused and full MAVROS tests passing on Lyrical for the local-mode patch.
8. FTP compatibility: focused regression coverage for the agreed normal `EBUSY`
   response change, plus the separately reviewed timeout, late-response, and
   cleanup behavior needed for safe recovery.

Do not introduce a transfer reservation or a global FTP scheduler in this
initial design. Existing per-operation protection does not guarantee an entire
multi-request download finishes uninterrupted; handle acquisition failure
without affecting ordinary MAVROS functionality.

## Implementation stages

Each stage starts with a focused review of its open decisions and ends with a
reviewable result and recorded validation. A stage heading is not authorization
to edit files, build, commit, push, or post upstream.

| Stage | Work | Acceptance criteria |
| --- | --- | --- |
| 1. Behavior contract | Record configuration, output, lifecycle, boundaries, and unresolved choices here. | Agreed choices are distinct from open decisions and implemented behavior. |
| 2. Isolated decoder | Choose upstream-compatible C++ dependency packaging, pin libevents, verify dependency/license requirements, and add a small isolated decoder with focused tests. | Tests use the pinned revision and cover full IDs, numeric/enum/bitfield arguments, plain-text developer-profile formatting, unknown events, and malformed metadata/payloads without ROS/FTP acquisition. Dependency provenance and required license notices are recorded. |
| 3. Local mode | Connect validated local JSON/XZ loading to the existing EVENT logging path, preserving severity and argument details. | Disabled behavior and raw ROS publication remain unchanged; successful decoding produces one readable log line; failures retain raw output. Severity, protocol/summary arguments, and invalid-file fallback have focused coverage. |
| 4. Metadata discovery | First add isolated general-metadata parsing and checksum helpers; then separately connect firmware-advertised discovery to the sys plugin. | Advertised FTP locations and checksums are represented correctly; missing, malformed, ambiguous, or unsupported metadata is rejected. Runtime request/reconnect behavior is reviewed in the following patch. |
| 5. FTP rejection responses | Replace busy-path exceptions with unsuccessful EBUSY responses in a separately reviewable change. Leave existing late-response/reset behavior unchanged; introduce no reservation or general scheduler. | Two focused cases cover busy rejection and duplicate open while preserving existing state. Our downloader stops on uncertain failures; EBUSY handling is not proof of isolation from existing FTP behavior. |
| 6. Automatic acquisition | Add one isolated acquisition helper owning general/event retrieval, then wire it to sys with a configurable cold-download delay defaulting to 30 seconds and in-memory activation. Use 239-byte reads with one outstanding request and no pacing, one EBUSY retry per operation, and the agreed complete-attempt budget with cancellation. | No blocking acquisition; complete downloads are validated before activation in memory; service waits/size/progress/attempt use are bounded without a fixed total deadline. Close failures do not invalidate otherwise validated complete bytes. Disconnect cancels work and invalidates active definitions; competing FTP operations are tested; disabled mode adds no traffic. Stage 7 adds prompt cache validation and bypasses downloads and their delay on a complete hit. |
| 7. Persistent cache | Add cache loading, atomic saving, matching, invalidation, and bounded storage after retrieval works in memory. | Matching definitions are validated and reused; stale/corrupt metadata is rejected. A complete hit bypasses downloads and their delay. Persistence failures retain validated in-memory definitions; reconnect behavior is covered. |
| 8. Vehicle validation | Exercise PX4 SITL and VOXL3: cold/warm cache, firmware changes, startup events, reconnects, unavailable FTP, interrupted transfers, concurrent FTP clients, corrupt cache files, unwritable cache directories, and persistence across container replacement. | Record decoded output, raw fallback, validated cache reuse, network/CPU observations, and telemetry/command responsiveness. Confirm raw StatusEvent fields and publication behavior remain unchanged. Tune timing only from recorded results and update the agreed plan if defaults change. |
| 9. Upstream handoff | Update the relevant MAVROS configuration/plugin documentation and prepare the upstream change. | Documentation describes implemented behavior; authorized build/test/style checks are recorded; unresolved checks and limitations are explicit. |

### Stage 4, patch A: isolated metadata helpers

Authorized scope: `px4_events/metadata.hpp/.cpp`, focused metadata unit tests,
CMake source/test registration, and this plan. No runtime discovery requests,
FTP operations, timers, cache reads/writes, parameters, or event logging changes.
In patch A the helpers were not connected to the sys plugin. Persistent caching follows
automatic retrieval; its identity/checksum contract is established first.

- Represent a file with its original advertised URI, normalized FTP path, and
  advertised CRC. Accept PX4's `mftp://etc/...` and hostless `mftp:///etc/...`
  forms. Reject other transports and ambiguous paths; do not select fallback
  or translation URLs.
- Parse general JSON version 1, selecting exactly one metadata type 4 (EVENTS)
  with a string URI and unsigned 32-bit `fileCrc`. A missing checksum is not a
  validated static identity; reject it for this initial feature. Zero is a valid
  checksum, not an absence marker. Other metadata types are not downloaded.
- Reuse the existing 8 MiB JSON input bound for the isolated parser. Automatic
  download and cache resource configuration remains a later decision.
- Compute MAVLink CRC32 with reflected polynomial `0xEDB88320`, initial zero,
  and no final XOR, matching MAVROS's existing NuttX CRC utility. This differs
  from the usual initial/final-XOR CRC32 defaults.
- PX4's general advertisement CRC covers the exact **uncompressed general JSON**
  bytes. Its event entry's `fileCrc` covers the **referenced event file bytes**,
  compressed when the URI references `.xz`. Do not reserialize JSON before
  comparing checksums or use a compressed-general CRC in place of the advertised
  one. Later acquisition must decompress general JSON before checking that CRC,
  and check an event `.xz` CRC before passing it through the local loader.
- Helpers throw on invalid input and contain no ROS logging. The later runtime
  acquisition boundary must catch failures and retain raw logging. No change is
  made to the existing decoder's exception containment.

Protocol references checked for this patch:
[general schema](https://github.com/mavlink/mavlink/blob/master/component_metadata/general.schema.json),
[PX4 advertisement](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/streams/COMPONENT_METADATA.hpp),
[PX4 metadata generation](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/component_information/generate_component_general.py),
[PX4 checksum generation](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/component_information/generate_crc.py),
and [PX4 build inputs](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/component_information/CMakeLists.txt).
Current PX4 also exposes the legacy `COMPONENT_INFORMATION` advertisement;
stage 4 patch B connects both formats under the policy below.

### Stage 4, patch B: advertisement discovery

Authorized scope: `px4_events/discovery.hpp/.cpp`, small sys-plugin connection
and advertisement hooks, recognition of `auto` mode without local definitions,
focused controller/plugin tests, CMake registration, and documentation. No FTP
operations, downloads, cache writes, or decoder activation are added. `auto`
clearly reports that retrieval remains unimplemented and retains raw events.

- Discovery starts only in valid automatic mode for a connected PX4 target.
  Disabled/local modes and non-PX4 vehicles add no discovery traffic. A local
  path supplied in automatic mode remains invalid and prevents discovery.
- Request current `COMPONENT_METADATA` using `MAV_CMD_REQUEST_MESSAGE` through
  a persistent asynchronous command-service client. Request legacy
  `COMPONENT_INFORMATION` only after explicit `DENIED` or `UNSUPPORTED` on the
  modern request. PX4 v1.12/v1.13 use `DENIED` when a requested message cannot
  be sent. No firmware-version gating, silent-timeout fallback, broadcast, or
  fallback URLs are introduced.
- Each format allows three total sends, not three retries after an initial
  request. Silence ends discovery after the modern budget; explicit rejection
  allows a separate legacy budget. Legacy rejection and other terminal results
  stop discovery. Temporary rejection consumes the current budget.
- Three seconds is the minimum interval between starts within a format; wait
  for each command-service response before another send. Explicit format
  fallback can start immediately. A positive ACK is not discovery completion.
  An unsuccessful command-service response with its default `ACCEPTED` result
  represents ACK timeout and is treated as silence, not legacy rejection.
- Implementation bounds service availability and each outstanding ROS service
  response to ten seconds, checked every 250 ms while discovery is active.
  The response bound also includes time queued behind other command services.
  Exceeding it stops discovery instead of sending another unresolved request.
  Removing a pending client request does not cancel its server-side operation;
  this patch does not change command-plugin serialization or ACK handling.
- Accept a valid advertisement from either format while discovery is active,
  including an advertisement that arrives before its ACK. Require the configured
  system/component and a terminated, nonempty, valid FTP URI. Invalid
  advertisements stop with a single warning for the connection. Receipt stores
  the advertised identity; actual file checksum verification comes with retrieval.
- Disconnect cancels the timer, removes local pending-client bookkeeping, and
  clears the descriptor. Reconnect starts a fresh generation and budget. Weak
  callback captures protect object lifetime; generation/request tokens discard
  callbacks from an earlier connection or request. State is mutex-protected
  because routed MAVLink messages, connection callbacks, and ROS callbacks can
  run on different threads. No new worker thread is created.
- The MAVLink advertisement has no transaction token. An old wire advertisement
  received during a new active connection cannot be distinguished solely by its
  fields; the generation guard applies to ROS service callbacks. File checksum
  validation and vehicle/reconnect testing remain necessary in later stages.

Controller tests use a synthetic steady clock to exercise the actual retry
state without waiting through each interval. Plugin tests use the existing UAS
fixture and a mock command service to cover command encoding, both advertisement
handlers, target filtering, raw output in automatic mode, invalid automatic
configuration, and zero discovery requests in disabled/local/non-PX4 modes.
Builds and test execution require the user's explicit request and remain pending.

### Stage 6, patch C: system-status runtime wiring

Scope: small lifecycle/advertisement/EVENT hooks in `sys_status.cpp`, focused
plugin tests, test linkage, and configuration documentation. Persistent cache
work remains stage 7. Discovery and acquisition retain their existing ownership
of command/FTP operations and validation; no transfer logic moves into sys.

- Declare read-only `event_metadata_download_delay`, default 30.0 seconds.
  Automatic mode rejects nonfinite/negative values without stopping MAVROS.
  Compare elapsed steady-clock seconds, avoiding conversion overflow for large
  finite values. Zero is valid. Start discovery immediately on PX4 connection;
  start acquisition only after both the delay and accepted advertisement.
- Share a start gate between the timer and both advertisement handlers. Start
  at most one acquisition per connection; internal retries retain their budget.
  Cancel the startup timer once its delay has elapsed; a later advertisement
  can start acquisition without polling indefinitely.
- Invalidate the connection generation before cancelling on disconnect, clear
  automatic definitions, and reject old completions/timer callbacks. Reconnect
  repeats discovery and the delay until cache reuse is implemented. Local
  definitions remain loaded. No early-event buffering or replay is added.
- Protect decoder replacement and connection state with one recursive mutex:
  acquisition setup/cancellation can complete inline and re-enter its owner.
  Completion checks generation, connection, and PX4 eligibility before install.
  Owner callbacks use weak plugin references. Diagnostic callbacks capture the
  logger rather than a raw plugin pointer.
- EVENT handling takes a shared decoder snapshot under that mutex and decodes
  after releasing it. Definitions are immutable after installation; an existing
  snapshot stays alive across disconnect. Raw StatusEvent publication is unchanged.
- Focused tests cover the default delay using a synthetic clock (no override),
  late/legacy advertisements, explicitly configured zero delay, invalid/large
  delay values, once-per-connection failure, reconnect/cancel/stale replies,
  concurrent completion/disconnect, local-mode lifetime, and raw topic fields.
  Source formatting/lint checks do not replace pending build/runtime validation.

### Stage 6, patch B: isolated two-file acquisition helper

Scope: `px4_events/acquisition.hpp/.cpp`, focused mock-service tests, CMake
registration, and shared in-memory loading in the existing decoder. The helper
is registered for compilation but not instantiated by `sys_status`. Runtime wiring, startup-delay
configuration, persistent caching, and end-to-end MAVLink testing remain later
patches. The caller owns PX4/automatic-mode gating and must cancel on disconnect.

- Download the advertised general file, unpack bounded JSON/XZ, and verify the
  advertised CRC against the exact uncompressed JSON. Parse only its EVENTS
  entry, then download the event file and verify its CRC against transferred
  bytes before unpacking and loading a fresh decoder. Return a decoder only
  after all checks pass; do not write temporary files or activate it in sys.
- Reuse local-file decompression/resource limits and event-definition parsing.
  Invalid JSON/XZ, unsupported versions/transports, missing event entries, and
  invalid definitions terminate acquisition with diagnostics and raw fallback.
- The agreed simplification merges the earlier single-file downloader into
  `Acquisition`. One class owns the three FTP service clients, one timer, one
  state mutex, and one final owner callback. File identity (general/events) is
  separate from transfer phase (services/open/read/close/retry). No inbox,
  intermediate completion callback, or separate downloader object remains.
- Permit at most three complete attempts, with 2/5-second backoffs, for checksum
  mismatches or confirmed transient FTP failures (exhausted EBUSY or EIO).
  Missing files and other errors stop. Retry eligibility remains internal;
  acquisition does not parse diagnostic strings.
- Uncertain general-file read/close stops before opening the event file. A
  confirmed general close failure permits the next download. Complete event
  bytes remain eligible for local validation after an uncertain final close,
  but no further request/retry is issued, even after checksum failure.
- Keep one file transfer active, contain completion-callback exceptions, and
  invoke owner callbacks outside locks. Pending-request checks and monotonic
  request tokens reject late service replies after cancellation/restart. The
  future sys owner must also guard its connection generation before activating
  any delivered result. One cancelled timer checks service availability,
  timeouts, and backoff every 250 ms only during acquisition; successful reads
  advance directly from their response callbacks. No worker thread or overall
  deadline is added.
- Tests cover plain/XZ success with a working decoder, both CRC domains,
  bounded checksum retries, malformed/unsupported metadata, missing-file and
  transient failures, uncertain reads/closes, cancellation, and late responses.
  The consolidated test target also retains chunk/offset, busy-budget, size,
  read-progress, service-availability, and callback-reentry/exception coverage
  from the removed downloader test. This is ROS-service integration coverage,
  not a MAVLink FTP end-to-end test.

### Stage 6, patch A: isolated single-file download helper

Historical scope, superseded by the agreed patch B simplification: the separate
`ftp_download.hpp/.cpp` and test file are removed. Their transfer behavior and
meaningful tests now live in acquisition. The following records the original
patch's boundaries, not the current file layout.

Authorized scope: `px4_events/ftp_download.hpp/.cpp`, focused mock-service tests,
CMake registration, and this plan. The helper is compiled but not created or
called by the sys plugin yet. No decoder activation, two-file acquisition,
startup-delay parameter, persistent cache, FTP protocol changes, or ownership
reservation is introduced in this patch.

- Use persistent asynchronous clients for the existing `ftp/open`, `ftp/read`,
  and `ftp/close` services; discover all three before opening. No new worker
  thread or synchronous service wait is added. A cancelled 250 ms watchdog
  timer runs only during a download for readiness, timeout, and busy-backoff
  checks; normal reads advance directly from their response callback.
- Open read-only and reject zero or oversized advertised sizes before reading.
  Require a valid advertised FTP path and reject lengths beyond the existing
  plugin's 238-byte path limit rather than accept silent truncation.
- Use offset-addressed reads of at most 239 bytes. Only successful response
  bytes advance the offset. Reject zero progress and responses larger than
  requested or remaining. Return complete bytes only; checksum validation,
  decompression, and metadata parsing remain the next integration patch.
- Allow one retry after explicit EBUSY for each open/read/close operation,
  after two seconds. Service readiness and each service response have a
  ten-second bound. There is no total transfer deadline and no artificial
  interval between successful reads. Helper errors and uncertain transport
  state are returned to the future acquisition owner, which must stop the
  connection's acquisition on uncertain errors instead of blindly restarting.
- After a confirmed read/size failure, close only a successfully opened file,
  when no operation is unresolved. Timeout/sequence errors stop without an
  immediate close or reset. Existing FTP errors for mismatched session/offset
  or invalid response opcode are also treated as uncertain. Cancellation sends
  no new FTP request and discards
  partial bytes; the owner must also guard its connection generation before
  using any completion result. Removing pending ROS requests does not cancel
  already queued/running FTP server operations.
- Return download errors separately from close errors. If the bytes are
  complete, a failed/timed-out close preserves them for subsequent checksum and
  parser validation, with a cleanup warning. Close failure may leave an FCU
  session open; it does not imply file corruption. An uncertain close also
  reports uncertain transport state. The helper never calls `ftp/reset`.
- Weak captures and request serials ignore stale service callbacks; mutable
  state is mutex-protected. Completion callbacks run outside the mutex and
  exceptions from them are contained. No overlapping start can replace an
  active download.

Tests use mock open/read/close services for multi-chunk transfer, one busy retry,
busy exhaustion, size and read-progress failures, nonfatal close failure and
timeout, cancellation, and late replies. Synthetic clock steps exercise backoff
and response bounds without sleeping through those waits. Builds and test
execution remain pending; written tests are not evidence of passing behavior.

Follow repository generated-documentation workflows when public configuration
is implemented. This plan must not make proposed parameters appear shipped.
Builds, commits, pushes, and upstream messages require explicit user instruction.

## Maintaining this plan

Before working on a stage, read this file, inspect the current branch, and state
which unresolved choices that stage needs. Update the file when a design change
is agreed; do not silently change an agreed default or expand scope.

Record stage progress and actual verification below. Distinguish completed
implementation from planned work, and report skipped or failing checks.

| Stage | Current status | Verification |
| --- | --- | --- |
| 1 | Design, pacing, recovery, and cache policies recorded; listed implementation details remain open. | Planning document only; no runtime behavior changed. |
| 2 | In-repository `libevents_vendor` pins commit `0ef305faa4cffdb09f01ca862ebfc108b7ff0cd5` with SHA-256 verification. Isolated local decoder and argument/metadata tests added. | User reported successful Lyrical dependency/MAVROS builds, an installed-consumer compile/link/run, and passing decoder tests. |
| 3 | Local JSON/XZ loading and sys EVENT logging integration added, with read-only startup parameters, raw fallback, and focused plugin tests. | User reported all MAVROS tests passing after style corrections. The remaining logger-restoration warning was corrected by the user; its rebuild is not recorded. Generated plugin-reference validation and exact vehicle-file verification remain pending. |
| 4, patch A | Isolated general-metadata parser, FTP file descriptor, CRC helper, and focused tests added. | CRC fixture constants checked against the existing MAVROS Python CRC utility. User is performing build/test validation; no result reported yet. |
| 4, patch B | Bounded asynchronous advertisement discovery and small sys hooks added. Automatic mode discovers only; retrieval remains pending. Focused controller and plugin tests added. | Source/API review and formatting checks only; builds and test execution pending. |
| 5, rejection responses | Busy-operation and duplicate-open exceptions replaced with unsuccessful EBUSY responses, leaving shared operation/session state untouched. Existing logging retained. Two focused tests added. Existing late-response/reset behavior intentionally retained. | Build and test execution pending; this change alone does not establish safe automatic recovery. |
| 6, patch A | Earlier isolated single-file downloader; now folded into acquisition, with the separate source/header/test removed. | User reported a successful build and zero test errors/failures before consolidation. |
| 6, patch B | One acquisition class owns FTP transfers, two-file validation/retries, one timer/mutex, and one owner completion. Shared bounded in-memory loading and consolidated tests remain. No runtime wiring or cache. | Build and test execution pending after consolidation. |
| 6, patch C | Runtime wiring added with delayed once-per-connection acquisition, generation guards, shared decoder snapshots, and disconnect cancellation. Local-mode lifetime and raw publication preserved. | Focused source formatting/lint checks; build and test execution pending for this patch. |
| XZ dependency replacement | Replace runtime liblzma with pinned `xz_embedded_vendor`. New isolated XZ helper validates a single-stream index, caps summed output at 8 MiB, allocates exact output, and requires stream end, matching size, complete input consumption, and integrity checks. Single-call mode avoids a separate advertised-size dictionary. Existing liblzma fixture encoders remain test-only. No FTP/discovery/runtime-hook changes in this patch. | Scoped uncrustify/cpplint, package XML parsing, source-archive checksum, and diff whitespace checks passed. Test-source updates, replacement builds, sanitizer checks, and real-file measurements with the replacement are deferred to the next stage. Earlier liblzma measurements and vehicle tests are not validation of the new implementation. |
| Bitfield output | Compact all-bits-set masks only for PX4's `navigation_mode_group_t` argument named `modes`. Preserve named flags and combine unknown set bits into one width-aware hexadecimal remainder. Retain zero as `0`, ordinary enum formatting, raw arguments, and the raw ROS topic. Use per-render enum overrides without modifying libevents or shared definitions. | Focused tests added/updated for PX4 scope, specific/zero/all masks, multiple simultaneous overrides, unknown remainders, and 64-bit flags/custom separators. Standalone formatting/lint and diff whitespace checks passed; no build or unit-test execution in this patch. |
| 7, patch A | Standalone cache helper and seven focused tests added. No runtime wiring or new ROS parameters. | Scoped uncrustify, cpplint, and diff whitespace checks passed. User reported all seven cache tests passing. Sanitizer execution remains pending. |
| 7, patch B | Optional cache reuse/saving integrated into acquisition; eleven focused acquisition tests added. No runtime cache injection or ROS parameter changes. | Scoped source checks recorded below; build and test execution pending. |
| 7, patch C | Runtime cache configuration/injection, early callback-group lookup, delayed FTP gate, and lifecycle tests added. | Source-only checks recorded below; build/test execution pending. |
| 7, patch D; 8–9 | Broader cache integration/vehicle validation and upstream handoff remain. | Not run. |

### Stage 7, patch A: standalone cache helper

Scope: `px4_events/cache.hpp/.cpp`, one focused test target, and this plan. The
helper performs synchronous local I/O and has no ROS logger, timer, FTP client,
vehicle ownership, or active-decoder state. It is linked into its test target
only in this patch, so running MAVROS does not yet read or write persistent cache.

- `load()` returns original bytes only after full identity, resource-limit,
  role-specific checksum, and existing metadata-parser validation. Missing,
  inaccessible, corrupt, unsupported, and mismatched entries are misses.
- `save()` revalidates bytes before touching disk, publishes one complete entry
  atomically, and performs best-effort oldest-saved eviction. It returns an error
  string for the future ROS owner to report without affecting decoding.
- Reuse `metadata_crc32()` for deterministic URI filename hashing; the full URI
  remains in the entry and is compared before accepting a hit. General CRC covers
  uncompressed JSON; event CRC covers original transferred bytes.
- Tests cover plain/XZ round trips for both roles, identity changes/forgery,
  corrupt/truncated entries, rejected saves, size limits, ignored partial files,
  publication failure cleanup, unavailable storage, oldest-first eviction, and
  a reader observing repeated atomic replacement by a separate cache instance.
- No cross-process locking, background worker, runtime cache parameter, startup
  delay change, or automatic cache lookup is introduced in patch A.

Remaining cache patches: B integrates reuse/saving with two-file acquisition;
C wires startup/reconnect and declares the three restart-only parameters;
D covers integration scenarios and documents deployment/persistence behavior.

### Stage 7, patch B: optional acquisition cache integration

Scope: acquisition source/header, focused acquisition tests, cache linkage for
the plugin and affected tests, and this plan. `create()` accepts an optional
shared cache; existing callers pass none, so running MAVROS still does not use
persistent cache. Runtime configuration, cache injection, and bypassing the
30-second download delay remain patch C.

- Check each file's cache entry before service discovery or opening. Route
  validated cached bytes through the existing general/event processing path.
  Cache misses do not spend FTP attempts. A complete hit needs no FTP services.
- Save each downloaded file after its checksum and parsing succeed. A valid
  general file is retained even if subsequent event retrieval fails. Cache hits
  are not rewritten, preserving oldest-saved eviction behavior.
- Persistence errors produce a warning rather than an acquisition failure;
  the valid in-memory decoder remains eligible for activation.
- A whole-attempt retry rechecks cache, allowing reuse of files published by
  another instance during backoff. Existing attempt budgets, EBUSY handling,
  uncertain-close policy, and cancellation/late-response guards remain intact.
- Cache loading/saving uses bounded synchronous local I/O under the existing
  acquisition mutex, as existing validation does. No worker thread is added.
  Completion still runs outside the mutex; a full hit may complete during
  `start()`, which must leave the timer cancelled after terminal completion.
- Eleven new cases cover complete plain/XZ hits without FTP services, cold XZ
  persistence, both partial-hit directions, corrupt entries, firmware/checksum
  changes, retained general metadata after event timeout, cache population during
  retry, cancellation with a late event reply, persistence failure, unresolved
  general close with cached events, and persistence after unresolved final close.

Verification: focused uncrustify, cpplint, and diff whitespace checks passed.
No build, unit-test execution, sanitizer run, or vehicle validation was performed
for patch B by the agent; those remain pending.

### Stage 7, patch C: runtime cache configuration and startup gate

Scope: three read-only sys parameters, optional cache construction/injection,
acquisition's explicit download gate and scheduled cache lookup, focused tests,
and configuration documentation. No change to EVENT decoding/publication, FTP
wire timeouts, retries, or the default 30-second connection-based download delay.

- In valid automatic mode, construct the cache only when enabled. Missing HOME
  or another cache-setup error warns and leaves download-only acquisition usable.
  Zero/negative cache limits log a configuration error but disable persistence
  only; disabled/local modes do not resolve directories or touch cache files.
- Start acquisition after the advertisement, passing whether the sys-owned
  connection delay has elapsed. Cache lookup runs in acquisition's existing
  callback group/timer rather than inline in the advertisement handler. A full
  hit needs no FTP services. General/event hits are logged at DEBUG.
- On a miss, wait in a separate download-gate phase. Sys releases the gate using
  its existing delay timer; only then start the FTP service-availability deadline.
  The gate carries no callback into sys, avoiding an extra lock-order dependency.
  Even very large finite delay settings retain the existing double comparison
  rather than overflowing a steady-clock duration conversion.
- Completion cancels the startup timer for the matching connection, whether
  successful or terminally failed. Disconnect cancels acquisition and invalidates
  automatic definitions before old callbacks can activate them. Reconnect needs
  a new advertisement, then can reuse validated entries without another delay.
- Cache reads and validation remain bounded synchronous local work under the
  acquisition mutex, but no file I/O is initiated by start()/allow_downloads()
  in the advertisement or startup-gate handler. No worker thread is introduced.
- Two focused acquisition tests cover a full hit behind a closed download gate
  and starting service timeouts only after gate release. Nine new plugin tests
  cover warm hits, both partial-hit directions, warm reconnect, disabled cache,
  zero/negative limits, unavailable storage, a small positive limit, and
  disconnect before scheduled cache lookup. Existing read-only/default/raw-mode
  tests are extended. Plugin-test storage is explicitly isolated using a
  temporary cache-directory parameter override; cache-enabled, default size,
  and download-delay assertions retain their production defaults.

Builds, unit-test execution, sanitizer runs, real-vehicle cold/warm cache checks,
and generated plugin-reference updates remain pending for this patch.
Focused uncrustify, cpplint, and diff whitespace checks passed; these are source
checks only and do not establish build or runtime correctness.
