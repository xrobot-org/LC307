#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: UPIXELS LC307 光流传感器（UART）驱动模块 / Driver Module for the UPIXELS LC307 optical-flow sensor over UART
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>

#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "uart.hpp"

/**
 * @brief LC307 光流传感器驱动模块，解析 UART 数据流并发布光流与距离样本。
 *        Driver Module for the LC307 optical-flow sensor; it parses the UART stream and
 *        publishes optical-flow and distance samples.
 */
class LC307
{
 public:
#pragma pack(push, 1)
  /**
   * @brief 发布的样本，字段紧凑排列。
   *        Published sample with packed fields.
   */
  struct Sample
  {
    int16_t flow_x_raw;         ///< 帧中的 X 方向原始光流值
                                ///< Raw X flow value from the frame
    int16_t flow_y_raw;         ///< 帧中的 Y 方向原始光流值
                                ///< Raw Y flow value from the frame
    float flow_x_at_1m_mps;     ///< flow_x_raw 的 float 形式，数值相同
                                ///< flow_x_raw as float, numerically equal
    float flow_y_at_1m_mps;     ///< flow_y_raw 的 float 形式，数值相同
                                ///< flow_y_raw as float, numerically equal
    uint16_t integration_time;  ///< 帧中的积分时间字段
                                ///< Integration-time field of the frame
    uint16_t distance_mm;       ///< 帧中的距离字段 (mm)
                                ///< Distance field of the frame (mm)
    float distance_m;           ///< 距离 (m)，等于 distance_mm / 1000
                                ///< Distance (m), equal to distance_mm / 1000
    uint8_t quality;            ///< 帧中的质量字段
                                ///< Quality field of the frame
    uint8_t version;            ///< 帧中的版本字段
                                ///< Version field of the frame
  };
#pragma pack(pop)

  /**
   * @brief 构造 LC307：设置 UART、初始化传感器（失败时每 100 ms 重试）并创建接收线程。
   *        Construct LC307: set up the UART, initialize the sensor (retrying every
   *        100 ms on failure) and create the receive thread.
   *
   * @param uart 连接 LC307 的 UART，模块将其设置为 19200、8N1。
   *             UART connected to the LC307; the Module sets it to 19200, 8N1.
   * @param ramfs 接收 `lc307` 命令的 RamFS。
   *              RamFS that receives the `lc307` command.
   * @param topic_name 发布样本的 Topic 名称。
   *                   Name of the Topic that publishes the samples.
   * @param task_stack_depth 接收线程栈深。
   *                         Stack depth of the receive thread.
   * @param configure_on_boot 启动时没有收到帧则发送初始化序列。
   *                          Send the initialization sequence when no frame is received
   *                          at start-up.
   * @param init_timeout_ms 启动时等待第一帧的时间，单位 ms。
   *                        How long start-up waits for the first frame, in ms.
   * @param frame_timeout_ms 接收线程每次等待 UART 数据的超时，单位 ms。
   *                         Timeout of each wait for UART data in the receive thread, in
   *                         ms.
   */
  LC307(
      LibXR::UART& uart,
      LibXR::RamFS& ramfs,
      const char* topic_name = "lc307_flow",
      size_t task_stack_depth = 2048,
      bool configure_on_boot = true,
      uint32_t init_timeout_ms = 1500,
      uint32_t frame_timeout_ms = 200)
      : configure_on_boot_(configure_on_boot),
        init_timeout_ms_(init_timeout_ms),
        frame_timeout_ms_(frame_timeout_ms),
        topic_(LibXR::Topic::CreateTopic<Sample>(topic_name)),
        uart_(std::addressof(uart)),
        cmd_file_(LibXR::RamFS::CreateCommand("lc307", CommandFunc, this))
  {
    ramfs.bin_.Add(cmd_file_);

    while (!Init())
    {
      LibXR::Thread::Sleep(100);
    }

    thread_.Create(this, ThreadFunc, "lc307_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

 private:
#pragma pack(push, 1)
  struct RawFrame
  {
    uint8_t head;
    uint8_t counter;
    int16_t flow_x;
    int16_t flow_y;
    uint16_t timespan;
    uint16_t distance;
    uint8_t quality;
    uint8_t version;
    uint8_t xor_sum;
    uint8_t tail;
  };
#pragma pack(pop)

  static constexpr auto STEP1_INIT =
      std::to_array<uint8_t>({0xAA, 0xAB, 0x96, 0x26, 0xBC, 0x50, 0x5C});

  static constexpr auto BF3901_CONFIG = std::to_array<uint8_t>(
      {0x12, 0x80, 0x11, 0x30, 0x1B, 0x06, 0x6B, 0x43, 0x12, 0x20, 0x3A, 0x00, 0x15, 0x02,
       0x62, 0x81, 0x08, 0xA0, 0x06, 0x68, 0x2B, 0x20, 0x92, 0x25, 0x27, 0x97, 0x17, 0x01,
       0x18, 0x79, 0x19, 0x00, 0x1A, 0xA0, 0x03, 0x00, 0x13, 0x00, 0x01, 0x13, 0x02, 0x20,
       0x87, 0x16, 0x8C, 0x01, 0x8D, 0xCC, 0x13, 0x07, 0x33, 0x10, 0x34, 0x1D, 0x35, 0x46,
       0x36, 0x40, 0x37, 0xA4, 0x38, 0x7C, 0x65, 0x46, 0x66, 0x46, 0x6E, 0x20, 0x9B, 0xA4,
       0x9C, 0x7C, 0xBC, 0x0C, 0xBD, 0xA4, 0xBE, 0x7C, 0x20, 0x09, 0x09, 0x03, 0x72, 0x2F,
       0x73, 0x2F, 0x74, 0xA7, 0x75, 0x12, 0x79, 0x8D, 0x7A, 0x00, 0x7E, 0xFA, 0x70, 0x0F,
       0x7C, 0x84, 0x7D, 0xBA, 0x5B, 0xC2, 0x76, 0x90, 0x7B, 0x55, 0x71, 0x46, 0x77, 0xDD,
       0x13, 0x0F, 0x8A, 0x10, 0x8B, 0x20, 0x8E, 0x21, 0x8F, 0x40, 0x94, 0x41, 0x95, 0x7E,
       0x96, 0x7F, 0x97, 0xF3, 0x13, 0x07, 0x24, 0x58, 0x97, 0x48, 0x25, 0x08, 0x94, 0xB5,
       0x95, 0xC0, 0x80, 0xF4, 0x81, 0xE0, 0x82, 0x1B, 0x83, 0x37, 0x84, 0x39, 0x85, 0x58,
       0x86, 0xFF, 0x89, 0x15, 0x8A, 0xB8, 0x8B, 0x99, 0x39, 0x98, 0x3F, 0x98, 0x90, 0xA0,
       0x91, 0xE0, 0x40, 0x20, 0x41, 0x28, 0x42, 0x26, 0x43, 0x25, 0x44, 0x1F, 0x45, 0x1A,
       0x46, 0x16, 0x47, 0x12, 0x48, 0x0F, 0x49, 0x0D, 0x4B, 0x0B, 0x4C, 0x0A, 0x4E, 0x08,
       0x4F, 0x06, 0x50, 0x06, 0x5A, 0x56, 0x51, 0x1B, 0x52, 0x04, 0x53, 0x4A, 0x54, 0x26,
       0x57, 0x75, 0x58, 0x2B, 0x5A, 0xD6, 0x51, 0x28, 0x52, 0x1E, 0x53, 0x9E, 0x54, 0x70,
       0x57, 0x50, 0x58, 0x07, 0x5C, 0x28, 0xB0, 0xE0, 0xB1, 0xC0, 0xB2, 0xB0, 0xB3, 0x4F,
       0xB4, 0x63, 0xB4, 0xE3, 0xB1, 0xF0, 0xB2, 0xA0, 0x55, 0x00, 0x56, 0x40, 0x96, 0x50,
       0x9A, 0x30, 0x6A, 0x81, 0x23, 0x33, 0xA0, 0xD0, 0xA1, 0x31, 0xA6, 0x04, 0xA2, 0x0F,
       0xA3, 0x2B, 0xA4, 0x0F, 0xA5, 0x2B, 0xA7, 0x9A, 0xA8, 0x1C, 0xA9, 0x11, 0xAA, 0x16,
       0xAB, 0x16, 0xAC, 0x3C, 0xAD, 0xF0, 0xAE, 0x57, 0xC6, 0xAA, 0xD2, 0x78, 0xD0, 0xB4,
       0xD1, 0x00, 0xC8, 0x10, 0xC9, 0x12, 0xD3, 0x09, 0xD4, 0x2A, 0xEE, 0x4C, 0x7E, 0xFA,
       0x74, 0xA7, 0x78, 0x4E, 0x60, 0xE7, 0x61, 0xC8, 0x6D, 0x70, 0x1E, 0x39, 0x98, 0x1A,
       0x9D, 0xF0});

  static int CommandFunc(LC307* self, int argc, char** argv)
  {
    if (argc == 1 || (argc == 2 && std::strcmp(argv[1], "status") == 0))
    {
      LibXR::STDIO::Printf<"init=%d frames=%u bad=%u dist=%.3f quality=%u\r\n">(
          self->init_ok_ ? 1 : 0, self->frame_count_, self->bad_frame_count_,
          self->sample_.distance_m, static_cast<unsigned int>(self->sample_.quality));
      return 0;
    }

    LibXR::STDIO::Printf<"usage: lc307 [status]\r\n">();
    return -1;
  }

  bool Init()
  {
    init_ok_ = false;

    const auto config_ans =
        uart_->SetConfig({19200, LibXR::UART::Parity::NO_PARITY, 8, 1});
    if (config_ans != LibXR::ErrorCode::OK)
    {
      return false;
    }

    DrainRx();
    if (WaitForFrame(init_timeout_ms_))
    {
      init_ok_ = true;
      return true;
    }

    if (!configure_on_boot_)
    {
      init_ok_ = true;
      return true;
    }

    uint8_t feedback[3] = {};
    if (!WriteBytes(STEP1_INIT.data(), STEP1_INIT.size()) ||
        !ReadBytes(feedback, sizeof(feedback), 200) || !AckOk(feedback))
    {
      return false;
    }

    for (size_t i = 0; i < BF3901_CONFIG.size(); i += 2)
    {
      uint8_t packet[5] = {0xBB, 0xDC, BF3901_CONFIG[i], BF3901_CONFIG[i + 1], 0x00};
      packet[4] = static_cast<uint8_t>(packet[1] ^ packet[2] ^ packet[3]);
      if (!WriteBytes(packet, sizeof(packet)) ||
          !ReadBytes(feedback, sizeof(feedback), 200) || !AckOk(feedback))
      {
        return false;
      }
    }

    const uint8_t close_config = 0xDD;
    if (!WriteBytes(&close_config, 1))
    {
      return false;
    }

    DrainRx();
    init_ok_ = true;
    return true;
  }

  static void ThreadFunc(LC307* self)
  {
    uint8_t buffer[32] = {};

    while (true)
    {
      LibXR::ReadOperation ready_op(self->sem_uart_, self->frame_timeout_ms_);
      if (self->uart_->Read({nullptr, 0}, ready_op) != LibXR::ErrorCode::OK)
      {
        continue;
      }

      while (self->uart_->read_port_->Size() > 0)
      {
        const size_t size = std::min(self->uart_->read_port_->Size(), sizeof(buffer));
        LibXR::ReadOperation read_now;
        if (self->uart_->Read({buffer, size}, read_now) != LibXR::ErrorCode::OK)
        {
          break;
        }

        for (size_t i = 0; i < size; ++i)
        {
          self->ParseByte(buffer[i]);
        }
      }
    }
  }

  bool WriteBytes(const uint8_t* data, size_t size)
  {
    LibXR::WriteOperation op(sem_uart_, 500);
    return uart_->Write({data, size}, op) == LibXR::ErrorCode::OK;
  }

  bool ReadBytes(uint8_t* data, size_t size, uint32_t timeout_ms)
  {
    LibXR::ReadOperation op(sem_uart_, timeout_ms);
    return uart_->Read({data, size}, op) == LibXR::ErrorCode::OK;
  }

  void DrainRx()
  {
    uint8_t scratch[32] = {};
    while (uart_->read_port_->Size() > 0)
    {
      const size_t size = std::min(uart_->read_port_->Size(), sizeof(scratch));
      LibXR::ReadOperation op;
      if (uart_->Read({scratch, size}, op) != LibXR::ErrorCode::OK)
      {
        break;
      }
    }
  }

  bool WaitForFrame(uint32_t timeout_ms)
  {
    const auto start = LibXR::Topic::NowTimestamp();
    uint8_t buffer[32] = {};

    while ((LibXR::Topic::NowTimestamp() - start).ToMillisecond() < timeout_ms)
    {
      LibXR::ReadOperation ready_op(sem_uart_, 20);
      if (uart_->Read({nullptr, 0}, ready_op) != LibXR::ErrorCode::OK)
      {
        continue;
      }

      while (uart_->read_port_->Size() > 0)
      {
        const size_t size = std::min(uart_->read_port_->Size(), sizeof(buffer));
        LibXR::ReadOperation read_now;
        if (uart_->Read({buffer, size}, read_now) != LibXR::ErrorCode::OK)
        {
          break;
        }

        for (size_t i = 0; i < size; ++i)
        {
          if (ParseByte(buffer[i]))
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  bool AckOk(const uint8_t feedback[3]) const
  {
    return static_cast<uint8_t>(feedback[0] ^ feedback[1]) == feedback[2] &&
           feedback[1] == 0x00;
  }

  bool ParseByte(uint8_t byte)
  {
    if (rx_index_ == 0)
    {
      if (byte != 0xFE)
      {
        return false;
      }
      frame_buffer_[rx_index_++] = byte;
      return false;
    }

    frame_buffer_[rx_index_++] = byte;
    if (rx_index_ < sizeof(frame_buffer_))
    {
      return false;
    }

    rx_index_ = 0;

    if (frame_buffer_[12] != XorChecksum(&frame_buffer_[2], 10))
    {
      ++bad_frame_count_;
      return false;
    }

    RawFrame frame{};
    std::memcpy(&frame, frame_buffer_.data(), sizeof(frame));

    sample_.flow_x_raw = frame.flow_x;
    sample_.flow_y_raw = frame.flow_y;
    sample_.flow_x_at_1m_mps = static_cast<float>(frame.flow_x);
    sample_.flow_y_at_1m_mps = static_cast<float>(frame.flow_y);
    sample_.integration_time = frame.timespan;
    sample_.distance_mm = frame.distance;
    sample_.distance_m = static_cast<float>(frame.distance) / 1000.0f;
    sample_.quality = frame.quality;
    sample_.version = frame.version;

    ++frame_count_;
    topic_.Publish(sample_);
    return true;
  }

  static uint8_t XorChecksum(const uint8_t* data, size_t size)
  {
    uint8_t value = 0;
    for (size_t i = 0; i < size; ++i)
    {
      value ^= data[i];
    }
    return value;
  }

  bool configure_on_boot_ = true;
  uint32_t init_timeout_ms_ = 1500;
  uint32_t frame_timeout_ms_ = 200;
  bool init_ok_ = false;

  uint32_t frame_count_ = 0;
  uint32_t bad_frame_count_ = 0;
  size_t rx_index_ = 0;

  Sample sample_{};
  std::array<uint8_t, sizeof(RawFrame)> frame_buffer_{};

  LibXR::Topic topic_;
  LibXR::UART* uart_;
  LibXR::Semaphore sem_uart_;
  LibXR::Thread thread_;
  LibXR::RamFS::File cmd_file_;
};
