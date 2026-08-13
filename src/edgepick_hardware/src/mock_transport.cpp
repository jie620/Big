#include "edgepick_hardware/mock_transport.hpp"

namespace edgepick_hardware
{

bool MockTransport::write(const JointCommand & command)
{
  writes_.push_back(MockWrite{command, accept_writes_});
  return accept_writes_;
}

void MockTransport::set_accept_writes(bool accept_writes)
{
  accept_writes_ = accept_writes;
}

const std::vector<MockWrite> & MockTransport::writes() const
{
  return writes_;
}

}  // namespace edgepick_hardware
