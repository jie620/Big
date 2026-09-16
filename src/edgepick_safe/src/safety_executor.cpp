#include "edgepick_safe/gate.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <map>
#include <tuple>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <edgepick_interfaces/msg/vla_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using edgepick_safe::joints;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
class SafetyExecutor : public rclcpp::Node {
 public:
  SafetyExecutor(): Node("safety_gate"), buffer_(get_clock()), listener_(buffer_) {
    real_ = declare_parameter("real_hardware", false);
    calibrated_ = declare_parameter("calibration_confirmed", false);
    gate_.observation_timeout=declare_parameter("observation_timeout",1.0);
    gate_.scene_timeout=declare_parameter("scene_timeout",2.0);
    gate_.policy_timeout=declare_parameter("policy_timeout",10.0);
    gate_.max_step=declare_parameter("max_joint_step",0.35);
    execution_timeout_=declare_parameter("execution_timeout",15.0);
    retries_=declare_parameter("planning_retries",2);
    destination_=declare_parameter<std::vector<double>>("destination",{0.16,0.16,0.04});
    radius_=declare_parameter("place_radius",0.06);
    payload_radius_=declare_parameter("payload_radius",0.035);
    if(destination_.size()!=3 || !std::all_of(destination_.begin(),destination_.end(),[](double x){return std::isfinite(x);}) ||
       radius_<=0 || payload_radius_<=0 || execution_timeout_<=0 || retries_<0 || retries_>5 ||
       gate_.max_step<=0 || gate_.observation_timeout<=0 || gate_.scene_timeout<=0 || gate_.policy_timeout<=0)
      throw std::runtime_error("invalid safety configuration");
    events_=create_publisher<std_msgs::msg::String>("/edgepick/safe/event",10);
    ready_=create_publisher<std_msgs::msg::Bool>("/edgepick/safe/ready",1);
    states_=create_publisher<std_msgs::msg::String>("/edgepick/safe/state",rclcpp::QoS(1).transient_local());
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){
      std::lock_guard<std::mutex> l(lock_);
      std::array<double,6> q;
      for(size_t i=0;i<6;++i) {
        auto it=std::find(m->name.begin(),m->name.end(),joints[i]);
        if(it==m->name.end() || std::count(m->name.begin(),m->name.end(),joints[i])!=1) return;
        auto index=static_cast<size_t>(it-m->name.begin());
        if(index>=m->position.size() || !std::isfinite(m->position[index])) return;
        q[i]=m->position[index];
      }
      double stamp=rclcpp::Time(m->header.stamp).seconds();
      if(!edgepick_safe::fresh(now().seconds(),stamp,gate_.observation_timeout)) return;
      gate_.measured=q; gate_.joint_at=stamp;
    }));
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::PointStamped>("/edgepick/safe/target",10,[this](geometry_msgs::msg::PointStamped::ConstSharedPtr m){
      if(m->header.frame_id!="base_link" || !std::isfinite(m->point.x) || !std::isfinite(m->point.y) || !std::isfinite(m->point.z)) return;
      std::lock_guard<std::mutex> l(lock_); target_=*m; gate_.target_at=rclcpp::Time(m->header.stamp).seconds();
    }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Header>("/edgepick/safe/scene_applied",10,[this](std_msgs::msg::Header::ConstSharedPtr m){
      if(m->frame_id!="base_link") return;
      std::lock_guard<std::mutex> l(lock_); gate_.cloud_at=gate_.scene_at=rclcpp::Time(m->stamp).seconds();
    }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Header>("/edgepick/safe/destination_clear",10,[this](std_msgs::msg::Header::ConstSharedPtr m){
      if(m->frame_id!="base_link") return;
      std::lock_guard<std::mutex> l(lock_); destination_at_=rclcpp::Time(m->stamp).seconds();
    }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Header>("/edgepick/safe/policy_alive",10,[this](std_msgs::msg::Header::ConstSharedPtr m){
      std::lock_guard<std::mutex> l(lock_); gate_.policy_at=rclcpp::Time(m->stamp).seconds();
    }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>("/edgepick/safe/estop",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::ConstSharedPtr m){
      std::lock_guard<std::mutex> l(lock_); gate_.estop=m->data; gate_.estop_at=now().seconds(); if(m->data) stop_locked("emergency_stop");
    }));
    subscriptions_.push_back(create_subscription<edgepick_interfaces::msg::VLAAction>("/edgepick/vla/proposal",1,[this](edgepick_interfaces::msg::VLAAction::ConstSharedPtr m){ accept(*m); }));
    arm_=create_service<std_srvs::srv::Trigger>("/edgepick/safe/arm",[this](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr out){
      std::lock_guard<std::mutex> l(lock_);
      if(busy_ || !initialized_ || (real_ && !calibrated_) || gate_.held) {out->message="busy, uncalibrated, not initialized, or payload recovery required"; return;}
      auto candidate=gate_; candidate.fault=false; candidate.armed=true;
      out->message=candidate.reason(now().seconds()); out->success=out->message.empty();
      if(out->success){gate_=candidate; cancelled_=false; phase_="approach"; event("armed");}
    });
    scene_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scene_service_=create_service<moveit_msgs::srv::ApplyPlanningScene>("/edgepick/safe/apply_observation",
      [this](const moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr request,
             moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr response){
        if(!initialized_) return;
        auto objects=request->scene.world.collision_objects;
        // Remove sensor returns on the robot using its actual collision geometry.
        // Padding is limited to one sensor voxel; nearby uncertain returns remain obstacles.
        {
          planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
          auto state=scene->getCurrentState(); state.update();
          for(auto & entry:robot_bodies_){
            std::get<2>(entry)->setPose(state.getCollisionBodyTransform(std::get<0>(entry),std::get<1>(entry)));
          }
          objects.erase(std::remove_if(objects.begin(),objects.end(),[&](const auto & object){
            if(object.id.rfind("observed_",0)!=0 || object.operation!=object.ADD || object.primitive_poses.size()!=1) return false;
            const auto & p=object.primitive_poses.front().position;
            return std::any_of(robot_bodies_.begin(),robot_bodies_.end(),[&](const auto & entry){return std::get<2>(entry)->containsPoint(Eigen::Vector3d(p.x,p.y,p.z));});
          }),objects.end());
        }
        // Replacing the complete observation removes old self returns as well.
        auto present=scene_interface_.getKnownObjectNames();
        for(const auto & name:present){
          if(name.rfind("observed_",0)==0 && std::none_of(objects.begin(),objects.end(),[&](const auto & o){return o.id==name;})){
            moveit_msgs::msg::CollisionObject removed;removed.id=name;removed.operation=removed.REMOVE;objects.push_back(removed);
          }
        }
        // Commit to the watchdog scene before forwarding to MoveIt. Its monitored
        // scene topic can lag while an execution capability is busy.
        {
          planning_scene_monitor::LockedPlanningSceneRW scene(monitor_);
          for(const auto & object:objects) {
            if(!scene->processCollisionObjectMsg(object)) return;
          }
        }
        response->success=scene_interface_.applyCollisionObjects(objects);
      },rmw_qos_profile_services_default,scene_group_);
    timer_=create_wall_timer(50ms,[this]{monitor();});
  }
  ~SafetyExecutor(){cancelled_=true; if(worker_.joinable()) worker_.join();}
  void initialize(){
    arm_group_=std::make_shared<MoveGroup>(shared_from_this(),"arm_group");
    grip_group_=std::make_shared<MoveGroup>(shared_from_this(),"grip_group");
    for(auto group:{arm_group_,grip_group_}) {
      group->setPlanningTime(2.0); group->setNumPlanningAttempts(2);
      group->setMaxVelocityScalingFactor(0.15); group->setMaxAccelerationScalingFactor(0.15);
      group->setGoalJointTolerance(0.03);
    }
    monitor_=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(),"robot_description");
    if(!monitor_->getPlanningScene()) throw std::runtime_error("missing robot model");
    monitor_->startSceneMonitor("/monitored_planning_scene"); monitor_->startStateMonitor("/joint_states");
    monitor_->requestPlanningSceneState("/get_planning_scene");
    // Mesh-to-body conversion computes convex hulls; do it once at startup,
    // never inside the live perception/cancellation path.
    for(const auto * link:monitor_->getRobotModel()->getLinkModelsWithCollisionGeometry()){
      for(size_t i=0;i<link->getShapes().size();++i){
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(link->getShapes()[i].get()));
        body->setPadding(0.015);
        robot_bodies_.emplace_back(link,i,std::move(body));
      }
    }
    initialized_=true;
  }
 private:
  void event(const std::string & text){if(!rclcpp::ok()) return;std_msgs::msg::String m; m.data=text; events_->publish(m); RCLCPP_INFO(get_logger(),"%s",text.c_str());}
  void stop_locked(const std::string & why){
    if(!gate_.fault) event(why);
    gate_.stop(); cancelled_=true; phase_="fault";
    if(arm_group_) arm_group_->stop();
    if(grip_group_) grip_group_->stop();
  }
  void monitor(){
    std::lock_guard<std::mutex> l(lock_);
    if(gate_.armed){auto why=gate_.reason(now().seconds()); if(!why.empty()) stop_locked(why);}
    std_msgs::msg::Bool r; r.data=initialized_ && gate_.reason(now().seconds()).empty() && !busy_ && phase_!="succeeded"; ready_->publish(r);
    std_msgs::msg::String s; s.data=phase_; states_->publish(s);
  }
  void accept(const edgepick_interfaces::msg::VLAAction & m){
    std::lock_guard<std::mutex> l(lock_);
    if(!initialized_ || busy_ || phase_=="succeeded") return;
    if(m.joint_names.size()!=6 || m.positions.size()!=6 || m.velocities.size()!=0) {event("invalid_action_shape");return;}
    for(size_t i=0;i<6;++i) if(m.joint_names[i]!=joints[i]) {event("joint_order_mismatch");return;}
    std::array<double,6> q; std::copy(m.positions.begin(),m.positions.end(),q.begin());
    double stamp=rclcpp::Time(m.header.stamp).seconds();
    auto why=gate_.proposal(now().seconds(),stamp,m.valid_for_ms/1000.0,m.sequence_id,q);
    if(!why.empty()){event(why);return;}
    gate_.sequence=m.sequence_id; cancelled_=false; busy_=true;
    if(worker_.joinable()) worker_.join();
    worker_=std::thread([this,q,stamp,ttl=m.valid_for_ms/1000.0]{
      try { run(q,stamp+ttl); } catch(const std::exception & e){ std::lock_guard<std::mutex> l(lock_); stop_locked(std::string("exception:")+e.what()); }
      busy_=false;
    });
  }
  bool healthy(double deadline){
    std::lock_guard<std::mutex> l(lock_);
    return rclcpp::ok() && !cancelled_ && gate_.reason(now().seconds()).empty() && now().seconds()<=deadline;
  }
  bool path_valid(const MoveGroup::Plan & plan){
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    moveit::core::RobotState state(scene->getCurrentState());
    moveit::core::robotStateMsgToRobotState(plan.start_state_,state);
    auto previous=state;
    // Check interpolated states, including gripper and attached payload.
    for(const auto & p:plan.trajectory_.joint_trajectory.points){
      state.setVariablePositions(plan.trajectory_.joint_trajectory.joint_names,p.positions); state.update();
      const int steps=std::max(1,static_cast<int>(std::ceil(previous.distance(state)/0.02)));
      for(int i=0;i<=steps;++i){
        auto sample=previous; previous.interpolate(state,double(i)/steps,sample); sample.update();
        if(!sample.satisfiesBounds() || scene->isStateColliding(sample,"",false)) return false;
      }
      previous=state;
    }
    return !plan.trajectory_.joint_trajectory.points.empty();
  }
  bool move(const std::shared_ptr<MoveGroup> & group,const std::vector<double> & q,double deadline){
    MoveGroup::Plan plan;
    bool planned=false;
    for(int attempt=0;attempt<=retries_ && healthy(deadline);++attempt){
      group->setStartStateToCurrentState();
      std::map<std::string,double> target;
      const size_t offset=group==grip_group_?5:0;
      for(size_t i=0;i<q.size();++i) target[joints[offset+i]]=q[i];
      if(!group->setJointValueTarget(target)) return false;
      if(group->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS && path_valid(plan)){planned=true;break;}
      event("planning_retry");
    }
    if(!planned || !healthy(deadline)) return false;
    auto result=std::async(std::launch::async,[group,plan]{return group->execute(plan);});
    const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(execution_timeout_);
    bool safe=true;
    while(result.wait_for(50ms)!=std::future_status::ready){
      // The accepted goal may run beyond the proposal TTL; watchdogs remain live.
      if(!healthy(INFINITY) || std::chrono::steady_clock::now()>until || !path_valid(plan)) {
        safe=false; group->stop();
        std::lock_guard<std::mutex> l(lock_); stop_locked("execution_cancelled_or_collision");
      }
    }
    if(result.get()!=moveit::core::MoveItErrorCode::SUCCESS || !safe || !healthy(INFINITY)) return false;
    std::lock_guard<std::mutex> l(lock_);
    const size_t offset=group==grip_group_?5:0;
    for(size_t i=0;i<q.size();++i) if(std::abs(gate_.measured[offset+i]-q[i])>0.05) return false;
    return true;
  }
  bool recheck_destination(double since){
    for(int i=0;i<40 && healthy(INFINITY);++i){
      {
        std::lock_guard<std::mutex> l(lock_);
        if(gate_.scene_at>since && gate_.target_at>since && destination_at_>since){
          auto state=arm_group_->getCurrentState(0.2);
          if(!state) return false;
          auto point=state->getGlobalLinkTransform("Gripping_point_Link").translation();
          const double dx=point.x()-destination_[0],dy=point.y()-destination_[1],dz=point.z()-destination_[2];
          if(std::sqrt(dx*dx+dy*dy+dz*dz)>radius_) return false;
          // Scene producer refuses a clear receipt unless the destination is observed.
          auto objects=scene_interface_.getObjects({"destination_blocked"});
          return objects.empty();
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }
  bool payload(bool attach){
    moveit_msgs::msg::AttachedCollisionObject object;
    object.link_name="Gripping_point_Link"; object.object.header.frame_id=object.link_name;
    object.object.id="edgepick_payload";
    object.object.operation=attach?moveit_msgs::msg::CollisionObject::ADD:moveit_msgs::msg::CollisionObject::REMOVE;
    if(attach){
      shape_msgs::msg::SolidPrimitive sphere; sphere.type=sphere.SPHERE; sphere.dimensions={payload_radius_};
      geometry_msgs::msg::Pose pose; pose.orientation.w=1; object.object.primitives={sphere}; object.object.primitive_poses={pose};
      object.touch_links={"Gripping_point_Link","Arm5_Link","rlink1","rlink2","rlink3","llink1","llink2","llink3"};
    }
    return scene_interface_.applyAttachedCollisionObject(object);
  }
  void run(const std::array<double,6> & q,double deadline){
    bool held;
    {std::lock_guard<std::mutex> l(lock_); held=gate_.held; phase_=held?"transport":"approach";}
    if(!move(arm_group_,std::vector<double>(q.begin(),q.begin()+5),deadline)){ fail("planning_or_execution_failed");return; }
    const bool opening=held && q[5]>-0.2;
    if(opening){
      {std::lock_guard<std::mutex> l(lock_);phase_="destination_recheck";}
      if(!recheck_destination(now().seconds())) {fail("destination_not_clear_or_unobserved");return;}
    }
    if(!move(grip_group_,{q[5]},deadline)){fail("gripper_failed");return;}
    if(!held && q[5]<=-0.5){
      // Verify visual target proximity before attaching a conservative payload.
      auto state=arm_group_->getCurrentState(0.2); if(!state){fail("grasp_feedback_missing");return;}
      auto p=state->getGlobalLinkTransform("Gripping_point_Link").translation();
      bool near;
      {std::lock_guard<std::mutex> l(lock_); auto t=target_.point; near=(p-Eigen::Vector3d(t.x,t.y,t.z)).norm()<0.07;}
      if(!near || !payload(true)){fail("grasp_not_verified");return;}
      {std::lock_guard<std::mutex> l(lock_);gate_.held=true;phase_="transport";}
      event("grasp_proximity_verified");
    }
    if(opening){
      if(!payload(false)){fail("detach_failed");return;}
      {std::lock_guard<std::mutex> l(lock_);gate_.held=false;phase_="verify_place";}
      double released=now().seconds(); bool verified=false;
      for(int i=0;i<40 && healthy(INFINITY);++i){
        {std::lock_guard<std::mutex> l(lock_); auto p=target_.point;
          if(gate_.target_at>released && std::hypot(p.x-destination_[0],p.y-destination_[1])<radius_ && std::abs(p.z-destination_[2])<radius_) {verified=true;break;}}
        std::this_thread::sleep_for(50ms);
      }
      if(!verified){fail("place_not_verified");return;}
      {std::lock_guard<std::mutex> l(lock_);phase_="succeeded";gate_.armed=false;}
      event("place_verified");
    } else event("proposal_completed");
  }
  void fail(const std::string & why){std::lock_guard<std::mutex> l(lock_);stop_locked(why);}
  std::vector<std::tuple<const moveit::core::LinkModel*,size_t,std::unique_ptr<bodies::Body>>> robot_bodies_;
  edgepick_safe::Gate gate_; std::mutex lock_; std::thread worker_;
  std::atomic<bool> busy_{false},cancelled_{false},initialized_{false};
  double destination_at_=0;
  bool real_,calibrated_; int retries_; double radius_,payload_radius_,execution_timeout_;
  std::vector<double> destination_; std::string phase_="disarmed";
  geometry_msgs::msg::PointStamped target_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_;
  std::shared_ptr<MoveGroup> arm_group_,grip_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  moveit::planning_interface::PlanningSceneInterface scene_interface_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_,states_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr scene_service_;
  rclcpp::CallbackGroup::SharedPtr scene_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv); auto node=std::make_shared<SafetyExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor; executor.add_node(node);
  std::thread spin([&]{executor.spin();});
  try{node->initialize();}catch(const std::exception & e){RCLCPP_FATAL(node->get_logger(),"%s",e.what());rclcpp::shutdown();}
  spin.join(); executor.remove_node(node); node.reset(); if(rclcpp::ok())rclcpp::shutdown(); return 0;
}
