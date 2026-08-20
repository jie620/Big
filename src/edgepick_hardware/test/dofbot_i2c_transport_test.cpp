#include <memory>
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

  bool accept_writes{true};
  std::vector<I2cBlockWrite> writes;
};

JointCommand sample_command()
{
  return JointCommand{{90.0, 120.0, 30.0, 0.0, 135.0, 45.0}, std::chrono::milliseconds{2000}};
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

}  // namespace
}  // namespace edgepick_hardware
