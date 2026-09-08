#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "edgepick_hardware/command.hpp"
#include "edgepick_hardware/transport.hpp"

namespace edgepick_hardware
{

std::uint8_t parse_i2c_address(const std::string & text);

struct DofbotI2cConfig
{
  std::string device{"/dev/i2c-7"};
  std::uint8_t address{0x15};
  bool enabled{false};
};

struct I2cBlockWrite
{
  std::uint8_t address{0};
  std::uint8_t command{0};
  std::vector<std::uint8_t> data;
};

class I2cBlockBus
{
public:
  virtual ~I2cBlockBus() = default;

  virtual bool write_block(
    std::uint8_t address,
    std::uint8_t command,
    const std::vector<std::uint8_t> & data) = 0;

  virtual std::optional<std::uint16_t> read_word(
    std::uint8_t address,
    std::uint8_t command) = 0;
};

class LinuxI2cBlockBus final : public I2cBlockBus
{
public:
  explicit LinuxI2cBlockBus(std::string device);
  ~LinuxI2cBlockBus() override;

  LinuxI2cBlockBus(const LinuxI2cBlockBus &) = delete;
  LinuxI2cBlockBus & operator=(const LinuxI2cBlockBus &) = delete;
  LinuxI2cBlockBus(LinuxI2cBlockBus && other) noexcept;
  LinuxI2cBlockBus & operator=(LinuxI2cBlockBus && other) noexcept;

  bool write_block(
    std::uint8_t address,
    std::uint8_t command,
    const std::vector<std::uint8_t> & data) override;

  std::optional<std::uint16_t> read_word(
    std::uint8_t address,
    std::uint8_t command) override;

private:
  std::string device_;
  int fd_{-1};
};

class DofbotI2cTransport final : public CommandTransport
{
public:
  explicit DofbotI2cTransport(DofbotI2cConfig config);
  DofbotI2cTransport(std::unique_ptr<I2cBlockBus> bus, DofbotI2cConfig config);

  bool write(const JointCommand & command) override;

  std::optional<std::array<double, kJointCount>> read_servo_angles();

  const DofbotI2cConfig & config() const;

  static std::vector<I2cBlockWrite> encode_command(
    const JointCommand & command,
    std::uint8_t address = 0x15);

private:
  std::unique_ptr<I2cBlockBus> bus_;
  DofbotI2cConfig config_;
};

}  // namespace edgepick_hardware
