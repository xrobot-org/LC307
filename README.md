# LC307

XRobot Module for the UPIXELS LC307 UART optical flow sensor.

During construction the module sets the UART to 19200 baud, 8N1, discards
pending input and waits up to `init_timeout_ms` for a valid frame. If none
arrives and `configure_on_boot` is `true`, it sends the LC307 initialization
sequence and the BF3901 image-sensor register table (each packet must be
acknowledged within 200 ms), closes configuration mode and starts streaming.
If `configure_on_boot` is `false`, a missing frame is not treated as an error.
A failed initialization is retried every 100 ms; the constructor returns only
after it succeeds.

The `lc307_thread` thread (`REALTIME` priority) then parses the byte stream:
14-byte frames starting with `0xFE`, checked with an XOR checksum over bytes
2..11. Frames with a bad checksum are counted and dropped; every good frame is
published.

## Published topic

`topic_name` (default `lc307_flow`), type `LC307::Sample`:

| Field | Meaning |
| --- | --- |
| `flow_x_raw`, `flow_y_raw` | Raw flow values from the frame (`int16_t`) |
| `flow_x_at_1m_mps`, `flow_y_at_1m_mps` | The raw flow values converted to `float`; no scaling is applied |
| `integration_time` | Integration time field of the frame |
| `distance_mm` | Distance field of the frame, mm |
| `distance_m` | `distance_mm / 1000`, m |
| `quality` | Quality field of the frame |
| `version` | Version field of the frame |

## Shell command

The module adds the command `lc307` to `bin` in `ramfs`.

- `lc307` or `lc307 status`: prints the init flag, good and bad frame counters,
  the last distance (m) and quality.

## Dependencies

No other Modules; LibXR only.

## Constructor

```cpp
LC307(LibXR::UART& uart, LibXR::RamFS& ramfs,
      const char* topic_name = "lc307_flow",
      size_t task_stack_depth = 2048,
      bool configure_on_boot = true,
      uint32_t init_timeout_ms = 1500,
      uint32_t frame_timeout_ms = 200);
```

Dependencies:

- `uart`: the UART connected to the LC307 (the module sets it to 19200 8N1).
- `ramfs`: RamFS that receives the `lc307` command.

Configuration:

- `topic_name`: name of the published topic, default `lc307_flow`.
- `task_stack_depth`: stack size of the receive thread, default 2048.
- `configure_on_boot`: send the initialization sequence when no frame is
  received during start-up, default `true`.
- `init_timeout_ms`: how long start-up waits for a first frame, ms, default 1500.
- `frame_timeout_ms`: timeout of each wait for UART data in the receive thread,
  ms, default 200 (a timeout only restarts the wait).

## Use

```sh
xrobot module add xrobot-org/LC307
xrobot setup
xrobot instance add xrobot-org/LC307
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/LC307
    id: lc307_0
    args:
      - uart: usart2
      - ramfs: ramfs
      - topic_name: '"lc307_flow"'
      - task_stack_depth: '2048'
      - configure_on_boot: 'true'
      - init_timeout_ms: '1500'
      - frame_timeout_ms: '200'
```

BSP side:

```cpp
XR_REGISTER(usart2, LibXR::UART);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/LC307` in a BSP, prints the current
constructor.
