#include "edgepick_hardware/dofbot_i2c_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <array>
#include <cstdio>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace edgepick_hardware
{
namespace
{

constexpr std::uint8_t kMotionTimeRegister = 0x1e;
constexpr std::uint8_t kSixServoRegister = 0x1d;
constexpr std::uint8_t kServoReadRegisterBase = 0x30;

std::uint16_t servo_position_for(std::size_t index, double angle_deg)
{
  // Arm_Lib mirrors servos 2-4 before encoding the I2C frame. Keep this
  // identical to the vendor driver; the public command remains the intuitive
  // servo angle supplied to Arm_serial_servo_write6().
  if (index >= 1U && index <= 3U) {
    angle_deg = 180.0 - angle_deg;
  }

  if (index == 4U) {
    return static_cast<std::uint16_t>((3700.0 - 380.0) * angle_deg / 270.0 + 380.0);
  }

  return static_cast<std::uint16_t>((3100.0 - 900.0) * angle_deg / 180.0 + 900.0);
}

std::uint16_t clamped_motion_time_ms(const JointCommand & command)
{
  const auto count = command.motion_time.count();
  const auto clamped = std::clamp<long long>(count, 0, 65535);
  return static_cast<std::uint16_t>(clamped);
}

std::optional<double> servo_angle_from_raw(std::size_t index, std::uint16_t raw)
{
  if (raw == 0U) {
    return std::nullopt;
  }

  int position = 0;
  if (index == 4U) {
    // Match Arm_Lib: convert the raw position to an integer before checking
    // the valid range. This accepts valid endpoint readings such as 0x0c1e.
    position = static_cast<int>(
      270.0 * (static_cast<double>(raw) - 380.0) / (3700.0 - 380.0));
    if (position < 0 || position > 270) {
      return std::nullopt;
    }
  } else {
    position = static_cast<int>(
      180.0 * (static_cast<double>(raw) - 900.0) / (3100.0 - 900.0));
    if (position < 0 || position > 180) {
      return std::nullopt;
    }
  }

  double angle = static_cast<double>(position);
  if (index >= 1U && index <= 3U) {
    angle = 180.0 - angle;
  }
  return angle;
}

void close_fd(int & fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

}  // namespace

std::uint8_t parse_i2c_address(const std::string & text)
{
  std::size_t consumed = 0;
  const auto value = std::stoul(text, &consumed, 0);
  if (consumed != text.size() || value < 0x08U || value > 0x77U) {
    throw std::invalid_argument("i2c_address must be a complete number in 0x08..0x77");
  }
  return static_cast<std::uint8_t>(value);
}

LinuxI2cBlockBus::LinuxI2cBlockBus(std::string device)
: device_(std::move(device))
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("failed to open I2C device '" + device_ + "'");
  }
  // Cooperating EdgePick processes must not interleave the two-write servo frame.
  if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
    close_fd(fd_);
    throw std::runtime_error("I2C device is already owned or cannot be locked: " + device_);
  }
}

LinuxI2cBlockBus::~LinuxI2cBlockBus()
{
  close_fd(fd_);
}

LinuxI2cBlockBus::LinuxI2cBlockBus(LinuxI2cBlockBus && other) noexcept
: device_(std::move(other.device_)), fd_(other.fd_)
{
  other.fd_ = -1;
}

LinuxI2cBlockBus & LinuxI2cBlockBus::operator=(LinuxI2cBlockBus && other) noexcept
{
  if (this != &other) {
    close_fd(fd_);
    device_ = std::move(other.device_);
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

bool LinuxI2cBlockBus::write_block(
  std::uint8_t address,
  std::uint8_t command,
  const std::vector<std::uint8_t> & data)
{
  if (fd_ < 0 || data.empty() || data.size() > 32U) {
    return false;
  }

  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    return false;
  }

  union i2c_smbus_data smbus_data{};
  smbus_data.block[0] = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), smbus_data.block + 1);

  struct i2c_smbus_ioctl_data request {};
  request.read_write = I2C_SMBUS_WRITE;
  request.command = command;
  request.size = I2C_SMBUS_I2C_BLOCK_DATA;
  request.data = &smbus_data;
  return ::ioctl(fd_, I2C_SMBUS, &request) >= 0;
}

std::optional<std::uint16_t> LinuxI2cBlockBus::read_word(
  std::uint8_t address,
  std::uint8_t command)
{
  if (fd_ < 0) {
    return std::nullopt;
  }

  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    return std::nullopt;
  }

  union i2c_smbus_data data{};
  struct i2c_smbus_ioctl_data request {};
  request.read_write = I2C_SMBUS_WRITE;
  request.command = command;
  request.size = I2C_SMBUS_BYTE_DATA;
  request.data = &data;
  data.byte = 0;
  if (::ioctl(fd_, I2C_SMBUS, &request) < 0) {
    return std::nullopt;
  }

  // Arm_Lib waits 3 ms between the read-register trigger and the word read.
  std::this_thread::sleep_for(std::chrono::milliseconds{3});

  request.read_write = I2C_SMBUS_READ;
  request.size = I2C_SMBUS_WORD_DATA;
  if (::ioctl(fd_, I2C_SMBUS, &request) < 0) {
    return std::nullopt;
  }

  // Match smbus.read_word_data() plus Arm_Lib's explicit byte swap.
  const auto raw = static_cast<std::uint16_t>(data.word);
  return static_cast<std::uint16_t>((raw >> 8U) | (raw << 8U));
}

DofbotI2cTransport::DofbotI2cTransport(DofbotI2cConfig config)
: DofbotI2cTransport(
    config.enabled ? std::make_unique<LinuxI2cBlockBus>(config.device) : nullptr,
    config)
{
}

DofbotI2cTransport::DofbotI2cTransport(
  std::unique_ptr<I2cBlockBus> bus,
  DofbotI2cConfig config)
: bus_(std::move(bus)), config_(std::move(config))
{
  if (config_.enabled && !bus_) {
    throw std::invalid_argument("DofbotI2cTransport requires an I2C bus");
  }
}

bool DofbotI2cTransport::write(const JointCommand & command)
{
  if (!config_.enabled) {
    return false;
  }

  const auto writes = encode_command(command, config_.address);
  if (writes.empty()) {
    return false;
  }
  for (const auto & write : writes) {
    if (!bus_->write_block(write.address, write.command, write.data)) {
      return false;
    }
  }
  return true;
}

std::optional<std::array<double, kJointCount>> DofbotI2cTransport::read_servo_angles()
{
  if (!config_.enabled || !bus_) {
    return std::nullopt;
  }

  std::array<double, kJointCount> angles{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const auto raw = bus_->read_word(
      config_.address, static_cast<std::uint8_t>(kServoReadRegisterBase + index + 1U));
    if (!raw.has_value()) {
      std::fprintf(
        stderr, "edgepick_hardware: failed to read servo %zu at register 0x%02x\n",
        index + 1U, static_cast<unsigned int>(kServoReadRegisterBase + index + 1U));
      return std::nullopt;
    }
    const auto angle = servo_angle_from_raw(index, *raw);
    if (!angle.has_value()) {
      std::fprintf(
        stderr, "edgepick_hardware: invalid raw position 0x%04x for servo %zu\n",
        static_cast<unsigned int>(*raw), index + 1U);
      return std::nullopt;
    }
    angles[index] = *angle;
  }
  return angles;
}

const DofbotI2cConfig & DofbotI2cTransport::config() const
{
  return config_;
}

std::vector<I2cBlockWrite> DofbotI2cTransport::encode_command(
  const JointCommand & command,
  std::uint8_t address)
{
  const JointLimits limits;
  if (address < 0x08 || address > 0x77 || command.motion_time.count() <= 0 ||
    command.motion_time.count() > 65535)
  {
    return {};
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double angle = command.angles_deg[index];
    if (!std::isfinite(angle) || angle < limits.min_deg[index] ||
      angle > limits.max_deg[index])
    {
      return {};
    }
  }
  const std::uint16_t motion_time = clamped_motion_time_ms(command);
  I2cBlockWrite time_write;
  time_write.address = address;
  time_write.command = kMotionTimeRegister;
  time_write.data = {
    static_cast<std::uint8_t>((motion_time >> 8) & 0xff),
    static_cast<std::uint8_t>(motion_time & 0xff),
  };

  I2cBlockWrite servo_write;
  servo_write.address = address;
  servo_write.command = kSixServoRegister;
  servo_write.data.reserve(kJointCount * 2U);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const std::uint16_t position = servo_position_for(index, command.angles_deg[index]);
    servo_write.data.push_back(static_cast<std::uint8_t>((position >> 8) & 0xff));
    servo_write.data.push_back(static_cast<std::uint8_t>(position & 0xff));
  }

  return {time_write, servo_write};
}

}  // namespace edgepick_hardware
