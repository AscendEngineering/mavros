# PX4 event decoding

The system-status plugin can translate PX4 MAVLink EVENT messages using libevents
and either a local firmware-matching definition file or automatically acquired
definitions. Decoding is disabled by default. Automatic mode retrieves the
firmware-advertised metadata through MAVROS FTP services. Persistent caching is
not implemented yet.

## Configuration

Set these parameters on the system-status plugin node, normally `/mavros/sys`,
in the parameter file supplied to MAVROS. These parameters are read-only after
startup; restart MAVROS to change them.

| Parameter | Default | Effect and constraints |
| --- | --- | --- |
| `event_decoding_mode` | `disabled` | `disabled` keeps existing raw logging and does no metadata I/O. `local` loads `event_metadata_file` once at startup. `auto` discovers and retrieves definitions from a connected PX4 FCU. Other values leave decoding inactive and produce a configuration error. |
| `event_metadata_file` | Empty string | Required in `local` mode and must be empty otherwise. An absolute path to readable JSON or `.xz`-compressed JSON inside the MAVROS process's filesystem. Use definitions from the vehicle's exact firmware; local mode cannot verify firmware identity. |
| `event_metadata_download_delay` | `20.0` seconds | In `auto` mode, wait this long from FCU connection before starting FTP acquisition. Must be finite and nonnegative; `0.0` allows immediate retrieval after discovery. Invalid values leave automatic decoding inactive. |

Example parameter-file section (replace the path with your own file):

```yaml
/**/sys:
  ros__parameters:
    event_decoding_mode: local
    event_metadata_file: /data/px4/all_events.json.xz
```

Mount the file into the container when running MAVROS in a container. Compressed
input is limited to 1 MiB; plain or decompressed JSON is limited to 8 MiB. XZ
Embedded validates the stream header, footer, and index before allocating the
declared output size. Its single-call decoder uses that output as the dictionary,
without a separate allocation based on the advertised dictionary size. Successful
decoding must consume all input, match the declared output size, and pass the XZ
integrity checks. CRC32, CRC64, and SHA-256 checks are supported. Multiple blocks
within one stream are supported; concatenated streams, trailing data, and
unsupported filters/checks are rejected. No maximum-sized output buffer is
allocated for smaller metadata. Metadata versions 1 and 2 are
supported. Invalid metadata, duplicate full IDs, and definitions requiring more
than the EVENT message's 40 argument bytes are rejected before activation.

An invalid configuration or file produces one setup error and leaves the
vehicle connection and raw event logging operational. Correct the configuration
or file and restart MAVROS. Successful loading does not prove the definitions
match the connected firmware.

## Automatic discovery and retrieval

With `event_decoding_mode: auto` and an empty `event_metadata_file`, the plugin
requests `COMPONENT_METADATA` from the configured PX4 target using the existing
command service. An explicit `DENIED` or `UNSUPPORTED` response switches to
legacy `COMPONENT_INFORMATION`. Each format permits at most three total sends.
Silence never triggers legacy fallback; other terminal failures stop discovery.
Temporary rejection and ACK timeout consume the current format's attempt budget.

Requests start at least three seconds apart within a format, and only after the
previous command-service response completes. Explicit legacy fallback can start
immediately. An accepted ACK alone is insufficient: discovery needs the actual
advertisement. A valid advertisement from either format can complete discovery.
Only advertisements from the configured system/component are accepted; empty,
unterminated, malformed, or non-FTP URIs stop discovery with a warning.

Service availability and each outstanding ROS service response have a ten-second
wait bound, checked by a 250 ms timer. If a response never arrives, discovery
stops without issuing another unresolved command. Removing a pending ROS request
does not cancel a command already queued or running on the command server.
No blocking waits run in discovery callbacks. The timer stops after success,
failure, or disconnect. A failure warns once for that connection; reconnect
clears the descriptor, resets the budget, and ignores old service callbacks.

Discovery starts promptly after connection, independently of the download delay.
Disabled/local modes and non-PX4 vehicles send no discovery requests. Once the
delay has elapsed and an advertisement is available, acquisition downloads the
general metadata and referenced event definitions. JSON/XZ files are bounded,
checksums are verified, and definitions are parsed before the decoder is
activated. Only MAVLink FTP locations are supported; no HTTP fallback is used.

One acquisition is started per connection; duplicate advertisements do not
restart it. The helper uses one outstanding FTP request, reads up to 239 bytes
at a time without added pacing, and retries an explicit busy response once per
operation. Eligible complete-attempt retries are limited to three attempts with
2/5-second backoffs. Uncertain transport errors stop acquisition without resets
or additional recovery requests. An unresolved general-file close prevents the
event download; complete event bytes can still be validated after an unresolved
final close. Failures retain raw logging.

Events received before activation are logged raw, without buffering or replay.
On disconnect, pending work is cancelled and automatic definitions are cleared.
Late results from an earlier connection cannot activate a decoder. Until caching
is implemented, reconnecting repeats discovery and the configured download delay.
Local-mode definitions remain loaded across disconnects. An EVENT already using
a decoder may finish safely while the plugin clears its active reference.

## Output and verification

Start MAVROS with the configured parameter file and connect a PX4 vehicle that
uses those definitions. A known event produces one readable `FCU: ...` log line
instead of the corresponding raw `FCU: EVENT ... with args ...` line. Unknown
events and events from other autopilots continue to use raw logging. Events are
not buffered or replayed.

An unexpected decoding exception produces a warning with the full event ID and
reason, once per process at that logging call site, then retains raw logging.
Disabled decoding and unknown IDs do not produce this warning. Diagnostic
callback failures are contained so they cannot prevent raw fallback.

Messages use libevents' plain-text `dev` profile and omit the longer description.
Named arguments absent from the visible message are appended in brackets.
An empty message uses its event name. Unknown enum values retain numeric
information. Bitfields retain named set bits and combine undefined set bits in
one `unknown_bits=0x...` remainder; an empty bitfield is displayed as `0`.
Only PX4's `navigation_mode_group_t` argument named `modes` abbreviates an
all-bits-set value as `all`. This describes the supplied applicability mask,
not a claim that every mode is blocked. Other bitfields keep individual flags. Control
characters are flattened to keep each event on one log line.

Ordinary severity mapping is unchanged: levels 0–3 log at ERROR, 4–5 at WARN,
6 at INFO, and 7 at DEBUG. A successfully decoded event with external severity
Disabled (8) logs at DEBUG, retaining its named protocol/summary arguments.
Enable DEBUG logging on `mavros.sys` to inspect those events. Unknown severity
values and undecodable events retain existing raw logging behavior.

The `/mavros/status_event` topic remains raw: message type, sensor-data QoS,
timestamp, severity, shortened 24-bit `px4_id`, argument bytes, and sequence are
unchanged. Translation uses the full MAVLink event ID before shortening; no
decoded ROS topic is added.

For development verification, the focused tests are `mavros-px4-events-test`,
`mavros-px4-event-metadata-test`, `mavros-px4-event-discovery-test`, and
`mavros-status-event-test`. They cover local JSON/XZ loading, namespace IDs,
argument formatting, resource limits, fallback, severity, read-only parameters,
raw publication through the system-status plugin, metadata/checksum parsing,
and discovery retry/fallback/target/reconnect rules. Run builds and tests from
the ROS workspace root, following the repository build/test instructions.
