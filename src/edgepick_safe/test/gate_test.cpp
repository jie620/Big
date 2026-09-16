#include <gtest/gtest.h>
#include "edgepick_safe/gate.hpp"
using edgepick_safe::Gate;
Gate ready() { Gate g; g.armed=true; g.estop=false; g.estop_at=g.joint_at=g.target_at=g.cloud_at=g.scene_at=g.policy_at=10; return g; }
TEST(Gate, StartupClosed) { Gate g; EXPECT_EQ(g.reason(10), "emergency_stop"); }
TEST(Gate, ValidProposal) { auto g=ready(); EXPECT_TRUE(g.proposal(10.1,10,1,1,{}).empty()); }
TEST(Gate, InvalidAction) {
  auto g=ready(); std::array<double,6> q{};
  q[0]=NAN; EXPECT_EQ(g.proposal(10.1,10,1,1,q),"joint_limit");
  q[0]=2; EXPECT_EQ(g.proposal(10.1,10,1,1,q),"joint_limit");
  q[0]=0.4; EXPECT_EQ(g.proposal(10.1,10,1,1,q),"action_jump");
}
TEST(Gate, ReplayedAndExpired) { auto g=ready(); g.sequence=4;
  EXPECT_EQ(g.proposal(10.1,10,1,4,{}),"replayed_proposal");
  EXPECT_EQ(g.proposal(10.1,11,1,5,{}),"expired_proposal");
  EXPECT_EQ(g.proposal(10.1,9,1,5,{}),"expired_proposal");
}
TEST(Gate, EachWatchdog) {
  for(auto field:{&Gate::estop_at,&Gate::joint_at,&Gate::target_at,&Gate::cloud_at,&Gate::scene_at,&Gate::policy_at}) {
    auto g=ready(); g.*field=0; EXPECT_FALSE(g.reason(10.1).empty());
  }
}
TEST(Gate, FaultDoesNotAutoResume) { auto g=ready(); g.stop(); g.estop=false; EXPECT_EQ(g.reason(10.1),"latched_fault"); }
TEST(Gate, ClockRollback) { EXPECT_FALSE(edgepick_safe::fresh(9,10,2)); }
