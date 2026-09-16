"""Synthetic inputs only; use an isolated ROS domain with mock hardware."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header, Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from edgepick_interfaces.msg import VLAAction
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from .contract import JOINTS, ordered_state
from .perception import box


class MockInputs(Node):
    def __init__(self):
        super().__init__("safe_mock_inputs")
        self.scenario=self.declare_parameter("scenario","normal").value
        if self.scenario not in ("normal","estop","target_loss","depth_timeout","policy_timeout","nan","jump","replay","collision","destination_blocked"):
            raise ValueError("unknown mock scenario")
        self.auto=self.declare_parameter("auto_arm",False).value
        self.armed_at=0;self.last_arm=0;self.start=time.monotonic();self.armed=False;self.ready=False;self.q=None;self.seq=0;self.sent=False;self.future=None
        self.target=self.create_publisher(PointStamped,"/edgepick/safe/target",10)
        self.scene=self.create_publisher(Header,"/edgepick/safe/scene_applied",10)
        self.clear=self.create_publisher(Header,"/edgepick/safe/destination_clear",10)
        self.alive=self.create_publisher(Header,"/edgepick/safe/policy_alive",10)
        self.estop=self.create_publisher(Bool,"/edgepick/safe/estop",QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.action=self.create_publisher(VLAAction,"/edgepick/vla/proposal",1)
        self.create_subscription(Bool,"/edgepick/safe/ready",self.on_ready,1)
        self.create_subscription(JointState,"/joint_states",self.on_joints,10)
        self.arm=self.create_client(Trigger,"/edgepick/safe/arm")
        self.apply=self.create_client(ApplyPlanningScene,"/apply_planning_scene")
        self.create_timer(0.1,self.tick)
    def on_ready(self,m):
        if not m.data:self.sent=False
        self.ready=m.data
    def on_joints(self,m):
        try:self.q=ordered_state(m.name,m.position)
        except (ValueError,KeyError):pass
    def tick(self):
        elapsed=time.monotonic()-self.start
        fault=self.armed and time.monotonic()-self.armed_at>2
        header=Header(stamp=self.get_clock().now().to_msg(),frame_id="base_link")
        self.estop.publish(Bool(data=fault and self.scenario=="estop"))
        if not(fault and self.scenario=="policy_timeout"):self.alive.publish(header)
        if not(fault and self.scenario=="target_loss"):
            point=PointStamped(header=header);point.point.y=0.2;point.point.z=0.04;self.target.publish(point)
        if not(fault and self.scenario=="depth_timeout") and self.apply.service_is_ready() and (self.future is None or self.future.done()):
            scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True
            if fault and self.scenario=="collision":scene.world.collision_objects=[box("mock_obstacle",[0,0,0.25],[0.3,0.3,0.3])]
            if self.scenario=="destination_blocked":scene.world.collision_objects=[box("destination_blocked",[0.16,0.16,0.04],[0.12]*3)]
            self.future=self.apply.call_async(ApplyPlanningScene.Request(scene=scene))
            self.future.add_done_callback(lambda future,h=header:self.on_scene(future,h))
        if self.auto and not self.armed and elapsed>4 and time.monotonic()-self.last_arm>2 and self.arm.service_is_ready():
            self.last_arm=time.monotonic()
            self.auto=False
            future=self.arm.call_async(Trigger.Request())
            future.add_done_callback(self.on_arm)
        if self.ready and not self.sent and self.q is not None:
            q=list(self.q);q[0]+=0.04 if q[0]<0.02 else -0.04
            if self.scenario=="nan":q[0]=float("nan")
            if self.scenario=="jump":q[0]=1.0
            self.seq+=1
            m=VLAAction(header=header,joint_names=JOINTS,positions=q,sequence_id=1 if self.scenario=="replay" else self.seq,valid_for_ms=8000,task="mock joint motion")
            self.action.publish(m);self.sent=True
    def on_scene(self,future,header):
        if future.result().success:
            self.scene.publish(header)
            if self.scenario!="destination_blocked":self.clear.publish(header)
    def on_arm(self,future):
        response=future.result();self.armed=response.success
        if self.armed:self.armed_at=time.monotonic()
        self.get_logger().info(f"arm={response.success}: {response.message}")
        if not self.armed:self.auto=True


def main(args=None):
    rclpy.init(args=args);node=MockInputs()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
