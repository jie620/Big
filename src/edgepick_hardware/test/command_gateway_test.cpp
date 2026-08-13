#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "edgepick_hardware/command_gateway.hpp"
#include "edgepick_hardware/mock_transport.hpp"

namespace edgepick_hardware
{
namespace
{

using Clock = std::chrono::steady_clock;

JointCommand valid_command()
{
  return JointCommand{{90.0, 90.0, 90.0, 90.0, 90.0, 30.0}, std::chrono::milliseconds{1000}};
}

TEST(CommandGatewayTest, AcceptsAValidCommandAndRecordsIt)
{
  MockTransport transport;
  CommandGateway gateway(transport);

  EXPECT_EQ(gateway.submit(valid_command(), Clock::time_point{}), CommandStatus::kAccepted);
  ASSERT_EQ(transport.writes().size(), 1U);
  EXPECT_TRUE(transport.writes().front().accepted);
  EXPECT_EQ(gateway.statistics().attempted, 1U);
  EXPECT_EQ(gateway.statistics().accepted, 1U);
}

TEST(CommandGatewayTest, RejectsInvalidAnglesWithoutWriting)
{
  MockTransport transport;
  CommandGateway gateway(transport);
  JointCommand command = valid_command();
  command.angles_deg[4] = 270.1;

  EXPECT_EQ(gateway.submit(command, Clock::time_point{}), CommandStatus::kInvalidJointAngle);
  EXPECT_TRUE(transport.writes().empty());
  EXPECT_EQ(gateway.statistics().rejected, 1U);
}

TEST(CommandGatewayTest, RejectsNonFiniteAnglesWithoutWriting)
{
  MockTransport transport;
  CommandGateway gateway(transport);
  JointCommand command = valid_command();
  command.angles_deg[0] = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(gateway.submit(command, Clock::time_point{}), CommandStatus::kInvalidJointAngle);
  EXPECT_TRUE(transport.writes().empty());
}

TEST(CommandGatewayTest, RejectsUnsafeMotionDurationWithoutWriting)
{
  MockTransport transport;
  CommandGateway gateway(transport);
  JointCommand command = valid_command();
  command.motion_time = std::chrono::milliseconds{99};

  EXPECT_EQ(gateway.submit(command, Clock::time_point{}), CommandStatus::kInvalidDuration);
  EXPECT_TRUE(transport.writes().empty());
}

TEST(CommandGatewayTest, SuppressesDuplicateCommands)
{
  MockTransport transport;
  CommandGateway gateway(transport);
  const auto start = Clock::time_point{};

  EXPECT_EQ(gateway.submit(valid_command(), start), CommandStatus::kAccepted);
  EXPECT_EQ(
    gateway.submit(valid_command(), start + std::chrono::milliseconds{100}),
    CommandStatus::kDuplicateSuppressed);
  EXPECT_EQ(transport.writes().size(), 1U);
}

TEST(CommandGatewayTest, RateLimitsChangedCommands)
{
  MockTransport transport;
  CommandGateway gateway(transport);
  const auto start = Clock::time_point{};
  JointCommand changed = valid_command();
  changed.angles_deg[0] = 100.0;

  EXPECT_EQ(gateway.submit(valid_command(), start), CommandStatus::kAccepted);
  EXPECT_EQ(
    gateway.submit(changed, start + std::chrono::milliseconds{19}),
    CommandStatus::kRateLimited);
  EXPECT_EQ(
    gateway.submit(changed, start + std::chrono::milliseconds{20}),
    CommandStatus::kAccepted);
  EXPECT_EQ(transport.writes().size(), 2U);
}

TEST(CommandGatewayTest, RecordsTransportFailureForFaultInjection)
{
  MockTransport transport;
  transport.set_accept_writes(false);
  CommandGateway gateway(transport);

  EXPECT_EQ(gateway.submit(valid_command(), Clock::time_point{}), CommandStatus::kTransportError);
  ASSERT_EQ(transport.writes().size(), 1U);
  EXPECT_FALSE(transport.writes().front().accepted);
  EXPECT_EQ(gateway.statistics().transport_failures, 1U);
  EXPECT_EQ(gateway.statistics().accepted, 0U);
}

}  // namespace
}  // namespace edgepick_hardware
