#pragma once

#include "edgepick_hardware/command.hpp"

namespace edgepick_hardware
{

class CommandTransport
{
public:
  virtual ~CommandTransport() = default;

  virtual bool write(const JointCommand & command) = 0;
};

}  // namespace edgepick_hardware
