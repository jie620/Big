#include "edgepick_hardware/dofbot_i2c_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <array>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace edgepick_hardware
{
namespace
{

constexpr std::uint8_t kMotionTimeRegister = 0x1e;
constexpr std::uint8_t kSixServoRegister = 0x1d;

std::uint16_t servo_position_for(std::size_t index, double angle_deg)
{
  if (index == 4U) {
    return static_cast<std::uint16_t>((3700.0 - 380.0) * angle_deg / 270.0 + 380.0);
  }

  if (index == 1U || index == 2U || index == 3U) {
    angle_deg = 180.0 - angle_deg;
  }
  return static_cast<std::uint16_t>((3100.0 - 900.0) * angle_deg / 180.0 + 900.0);
}

std::uint16_t clamped_motion_time_ms(const JointCommand & command)
{
  const auto count = command.motion_time.count();
  const auto clamped = std::clamp<long long>(count, 0, 65535);
  return static_cast<std::uint16_t>(clamped);
}

void close_fd(int & fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

}  // namespace

LinuxI2cBlockBus::LinuxI2cBlockBus(std::string device)
: device_(std::move(device))
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("failed to open I2C device '" + device_ + "'");
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

DofbotI2cTransport::DofbotI2cTransport(DofbotI2cConfig config)
: DofbotI2cTransport(
    config.enabled ? std::make_unique<LinuxI2cBlockBus>(config.device) : nullptr,
    std::move(config))
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
  for (const auto & write : writes) {
    if (!bus_->write_block(write.address, write.command, write.data)) {
      return false;
    }
  }
  return true;
}

const DofbotI2cConfig & DofbotI2cTransport::config() const
{
  return config_;
}

std::vector<I2cBlockWrite> DofbotI2cTransport::encode_command(
  const JointCommand & command,
  std::uint8_t address)
{
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
