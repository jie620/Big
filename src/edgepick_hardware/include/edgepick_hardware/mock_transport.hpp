#pragma once

#include <vector>

#include "edgepick_hardware/transport.hpp"

namespace edgepick_hardware
{

struct MockWrite
{
  JointCommand command;
  bool accepted{false};
};

class MockTransport final : public CommandTransport
{
public:
  bool write(const JointCommand & command) override;
  void set_accept_writes(bool accept_writes);
  const std::vector<MockWrite> & writes() const;

private:
  bool accept_writes_{true};
  std::vector<MockWrite> writes_;
};

}  // namespace edgepick_hardware
