#include <gtest/gtest.h>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace edgepick_hardware
{
namespace
{

TEST(PluginLoadingTest, LoadsMockSystemInterfaceThroughPluginlib)
{
  // ros2_control loads hardware through pluginlib, not through normal C++
  // linkage. This catches packaging mistakes such as accidentally building the
  // hardware package as a static library.
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  const auto system =
    loader.createSharedInstance("edgepick_hardware/MockSystemInterface");

  ASSERT_NE(system, nullptr);
}

}  // namespace
}  // namespace edgepick_hardware
