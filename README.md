# LC307

UPIXELS LC307 光流传感器（UART）驱动模块 / Driver Module for the UPIXELS LC307 optical-flow sensor over UART

## 1. 模块作用 / Purpose

构造时，LC307 把 UART 设置为 19200 波特、8N1，丢弃已收到的数据，并等待至多 `init_timeout_ms` 毫秒以收到一帧有效数据。未收到且 `configure_on_boot` 为 `true` 时，模块发送 LC307 初始化序列和 BF3901 图像传感器寄存器表（每个数据包需在 200 ms 内得到应答），随后关闭配置模式并开始输出数据流。`configure_on_boot` 为 `false` 时，未收到帧视为初始化成功。初始化失败时每 100 ms 重试，构造函数在成功后返回。

随后线程 `lc307_thread`（REALTIME 优先级）解析字节流：帧长 14 字节，以 `0xFE` 开头，对第 2 到第 11 字节做异或校验。校验错误的帧被计数并丢弃，每个有效帧都发布一次。

模块在 RamFS 的 `bin` 目录注册命令 `lc307`：

- `lc307` 或 `lc307 status`：打印初始化标志、有效帧与错误帧计数、最近一次的距离（m）和质量。

Upon construction, LC307 sets the UART to 19200 baud, 8N1, discards pending input and waits up to `init_timeout_ms` ms for a valid frame. When none arrives and `configure_on_boot` is `true`, the Module sends the LC307 initialization sequence and the BF3901 image-sensor register table (each packet must be acknowledged within 200 ms), then closes configuration mode and starts the data stream. When `configure_on_boot` is `false`, a missing frame counts as a successful initialization. A failed initialization is retried every 100 ms, and the constructor returns after it succeeds.

The thread `lc307_thread` (REALTIME priority) then parses the byte stream: frames are 14 bytes long, start with `0xFE`, and carry an XOR checksum over bytes 2 to 11. Frames with a wrong checksum are counted and dropped; every valid frame is published once.

The Module registers the command `lc307` in the RamFS `bin` directory:

- `lc307` or `lc307 status`: print the initialization flag, the counters of valid and bad frames, and the last distance (m) and quality.

## 2. 样本字段 / Sample Fields

Topic 发布的类型为 `LC307::Sample`：

| 字段 | 说明 |
| --- | --- |
| `flow_x_raw`、`flow_y_raw` | 帧中的原始光流值（`int16_t`） |
| `flow_x_at_1m_mps`、`flow_y_at_1m_mps` | 原始光流值的 `float` 形式，数值与原始值相同 |
| `integration_time` | 帧中的积分时间字段 |
| `distance_mm` | 帧中的距离字段，单位 mm |
| `distance_m` | `distance_mm / 1000`，单位 m |
| `quality` | 帧中的质量字段 |
| `version` | 帧中的版本字段 |

The Topic publishes the type `LC307::Sample`:

| Field | Meaning |
| --- | --- |
| `flow_x_raw`, `flow_y_raw` | Raw flow values from the frame (`int16_t`) |
| `flow_x_at_1m_mps`, `flow_y_at_1m_mps` | The raw flow values as `float`, numerically equal to the raw values |
| `integration_time` | Integration-time field of the frame |
| `distance_mm` | Distance field of the frame, in mm |
| `distance_m` | `distance_mm / 1000`, in m |
| `quality` | Quality field of the frame |
| `version` | Version field of the frame |

## 3. 构造接口 / Constructor

```cpp
LC307(LibXR::UART& uart, LibXR::RamFS& ramfs,
      const char* topic_name = "lc307_flow",
      size_t task_stack_depth = 2048,
      bool configure_on_boot = true,
      uint32_t init_timeout_ms = 1500,
      uint32_t frame_timeout_ms = 200);
```

依赖：

- `uart`：连接 LC307 的 `LibXR::UART`，取自 BSP 的硬件注册（`XR_REGISTER`）；模块将其设置为 19200、8N1。
- `ramfs`：接收 `lc307` 命令的 `LibXR::RamFS`，取自 BSP 的硬件注册。

配置参数：

- `topic_name`：发布的 Topic 名称，默认 `"lc307_flow"`。
- `task_stack_depth`：接收线程栈深，默认 2048。
- `configure_on_boot`：启动时没有收到帧则发送初始化序列，默认 `true`。
- `init_timeout_ms`：启动时等待第一帧的时间，单位 ms，默认 1500。
- `frame_timeout_ms`：接收线程每次等待 UART 数据的超时，单位 ms，默认 200；超时后重新开始等待。

Dependencies:

- `uart`: the `LibXR::UART` connected to the LC307, taken from the BSP's Registration (`XR_REGISTER`); the Module sets it to 19200, 8N1.
- `ramfs`: the `LibXR::RamFS` that receives the `lc307` command, taken from the BSP's Registration.

Configuration parameters:

- `topic_name`: name of the published Topic, default `"lc307_flow"`.
- `task_stack_depth`: stack depth of the receive thread, default 2048.
- `configure_on_boot`: send the initialization sequence when no frame is received at start-up, default `true`.
- `init_timeout_ms`: how long start-up waits for the first frame, in ms, default 1500.
- `frame_timeout_ms`: timeout of each wait for UART data in the receive thread, in ms, default 200; after a timeout the wait restarts.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `topic_name`（默认 `lc307_flow`） | 发布 | `LC307::Sample` | 光流与距离样本，字段见第 2 节 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `topic_name` (default `lc307_flow`) | Publish | `LC307::Sample` | Optical-flow and distance sample, fields in section 2 |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/LC307` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/LC307`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/LC307
    id: lc307_0
    args:
      - uart: usart2
      - ramfs: ramfs
      - topic_name: "lc307_flow"
      - task_stack_depth: 2048
      - configure_on_boot: true
      - init_timeout_ms: 1500
      - frame_timeout_ms: 200
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一个通过 UART 连接的 LC307 光流传感器；UART 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: one LC307 optical-flow sensor connected over UART; the UART and RamFS are registered by the BSP with `XR_REGISTER`.
