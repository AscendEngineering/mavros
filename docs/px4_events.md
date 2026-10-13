# PX4 event decoding

The system-status plugin can translate PX4 MAVLink EVENT messages using a local
firmware-matching definition file and libevents. Decoding is disabled by default.
Local files provide decoding. Automatic mode currently discovers the general
metadata advertisement only; FTP retrieval and persistent caching are not
implemented yet, so automatic mode still logs raw events.

## Configuration

Set these parameters on the system-status plugin node, normally `/mavros/sys`,
in the parameter file supplied to MAVROS. Both parameters are read-only after
startup; restart MAVROS to change them.

| Parameter | Default | Effect and constraints |
| --- | --- | --- |
| `event_decoding_mode` | `disabled` | `disabled` keeps existing raw logging and does no metadata I/O. `local` loads `event_metadata_file` once at startup. `auto` requests metadata advertisements from a connected PX4 FCU; retrieval is not implemented yet. Other values leave decoding inactive and produce a configuration error. |
| `event_metadata_file` | Empty string | Required in `local` mode and must be empty otherwise. An absolute path to readable JSON or `.xz`-compressed JSON inside the MAVROS process's filesystem. Use definitions from the vehicle's exact firmware; local mode cannot verify firmware identity. |

Example parameter-file section (replace the path with your own file):

```yaml
/**/sys:
  ros__parameters:
    event_decoding_mode: local
    event_metadata_file: /data/px4/all_events.json.xz
```

Mount the file into the container when running MAVROS in a container. Compressed
input is limited to 1 MiB; plain or decompressed JSON is limited to 8 MiB. XZ
decoder working memory is limited to 64 MiB. Metadata versions 1 and 2 are
supported. Invalid metadata, duplicate full IDs, and definitions requiring more
than the EVENT message's 40 argument bytes are rejected before activation.

An invalid configuration or file produces one setup error and leaves the
vehicle connection and raw event logging operational. Correct the configuration
or file and restart MAVROS. Successful loading does not prove the definitions
match the connected firmware.

## Automatic discovery (retrieval pending)

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

Discovery starts promptly after connection, without the future 20-second file
download delay. Disabled/local modes and non-PX4 vehicles send no discovery
requests. Automatic mode reports at startup that retrieval is not implemented;
even a successful advertisement leaves events raw in this staged implementation.
The advertised checksum is stored but no file checksum is verified in discovery.

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
An empty message uses its event name. Unknown enum values and bitfield bits
retain numeric information; an empty bitfield is displayed as `0`. Control
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
