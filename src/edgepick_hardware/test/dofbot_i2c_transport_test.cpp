#include <memory>
#include <limits>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "edgepick_hardware/dofbot_i2c_transport.hpp"

namespace edgepick_hardware
{
namespace
{

class RecordingI2cBus final : public I2cBlockBus
{
public:
  bool write_block(
    std::uint8_t address,
    std::uint8_t command,
    const std::vector<std::uint8_t> & data) override
  {
    writes.push_back(I2cBlockWrite{address, command, data});
    return accept_writes;
  }

  std::optional<std::uint16_t> read_word(
    std::uint8_t,
    std::uint8_t) override
  {
    return read_value;
  }

  bool accept_writes{true};
  std::optional<std::uint16_t> read_value{0x0001};
  std::vector<I2cBlockWrite> writes;
};

JointCommand sample_command()
{
  return JointCommand{{90.0, 120.0, 30.0, 0.0, 135.0, 45.0}, std::chrono::milliseconds{2000}};
}

TEST(DofbotI2cTransportTest, ParsesOnlyCompleteSevenBitAddresses)
{
  EXPECT_EQ(parse_i2c_address("0x15"), 21U);
  EXPECT_EQ(parse_i2c_address("21"), 21U);
  for (const auto text : {"0x15junk", "0x115", "0x00", "-1", "", "0x80"}) {
    EXPECT_THROW(parse_i2c_address(text), std::exception);
  }
}

TEST(DofbotI2cTransportTest, AllowsOnlyOneOwnerPerDevice)
{
  char path[] = "/tmp/edgepick_bus_lock_XXXXXX";
  const int fd = ::mkstemp(path);
  ASSERT_GE(fd, 0);
  ::close(fd);
  {
    LinuxI2cBlockBus first(path);
    EXPECT_THROW({LinuxI2cBlockBus second(path);}, std::runtime_error);
  }
  EXPECT_NO_THROW({LinuxI2cBlockBus reopened(path);});
  ::unlink(path);
}

TEST(DofbotI2cTransportTest, EncodesVendorSixServoFrame)
{
  const auto writes = DofbotI2cTransport::encode_command(sample_command());

  ASSERT_EQ(writes.size(), 2U);
  EXPECT_EQ(writes[0].address, 0x15);
  EXPECT_EQ(writes[0].command, 0x1e);
  EXPECT_EQ(writes[0].data, (std::vector<std::uint8_t>{0x07, 0xd0}));

  EXPECT_EQ(writes[1].address, 0x15);
  EXPECT_EQ(writes[1].command, 0x1d);
  EXPECT_EQ(
    writes[1].data,
    (std::vector<std::uint8_t>{
      0x07, 0xd0,
      0x06, 0x61,
      0x0a, 0xad,
      0x0c, 0x1c,
      0x07, 0xf8,
      0x05, 0xaa,
    }));
}

TEST(DofbotI2cTransportTest, RefusesWritesUnlessExplicitlyEnabled)
{
  auto bus = std::make_unique<RecordingI2cBus>();
  auto * bus_ptr = bus.get();
  DofbotI2cConfig config;
  config.enabled = false;
  DofbotI2cTransport transport(std::move(bus), config);

  EXPECT_FALSE(transport.write(sample_command()));
  EXPECT_TRUE(bus_ptr->writes.empty());
}

TEST(DofbotI2cTransportTest, DisabledTransportCanExistWithoutAnOpenBus)
{
  DofbotI2cConfig config;
  config.enabled = false;
  DofbotI2cTransport transport(std::move(config));

  EXPECT_FALSE(transport.config().enabled);
  EXPECT_EQ(transport.config().device, "/dev/i2c-7");
}

TEST(DofbotI2cTransportTest, WritesTimeThenServoFrameWhenEnabled)
{
  auto bus = std::make_unique<RecordingI2cBus>();
  auto * bus_ptr = bus.get();
  DofbotI2cConfig config;
  config.enabled = true;
  DofbotI2cTransport transport(std::move(bus), config);

  EXPECT_TRUE(transport.write(sample_command()));
  ASSERT_EQ(bus_ptr->writes.size(), 2U);
  EXPECT_EQ(bus_ptr->writes[0].command, 0x1e);
  EXPECT_EQ(bus_ptr->writes[1].command, 0x1d);
}

TEST(DofbotI2cTransportTest, ReportsFailureWhenAnyBusWriteFails)
{
  auto bus = std::make_unique<RecordingI2cBus>();
  auto * bus_ptr = bus.get();
  bus_ptr->accept_writes = false;
  DofbotI2cConfig config;
  config.enabled = true;
  DofbotI2cTransport transport(std::move(bus), config);

  EXPECT_FALSE(transport.write(sample_command()));
  ASSERT_EQ(bus_ptr->writes.size(), 1U);
  EXPECT_EQ(bus_ptr->writes[0].command, 0x1e);
}

TEST(DofbotI2cTransportTest, RejectsInvalidCommandsBeforeAnyBusWrite)
{
  auto bus = std::make_unique<RecordingI2cBus>();
  auto * recorded = bus.get();
  DofbotI2cConfig config;
  config.enabled = true;
  DofbotI2cTransport transport(std::move(bus), config);
  for (double invalid : {-1.0, 181.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto command = sample_command();
    command.angles_deg[0] = invalid;
    EXPECT_FALSE(transport.write(command));
  }
  for (int duration : {-1, 0, 65536}) {
    auto command = sample_command();
    command.motion_time = std::chrono::milliseconds{duration};
    EXPECT_FALSE(transport.write(command));
  }
  EXPECT_TRUE(DofbotI2cTransport::encode_command(sample_command(), 0x80).empty());
  EXPECT_TRUE(recorded->writes.empty());
}

}  // namespace
}  // namespace edgepick_hardware
