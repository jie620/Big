"""MuJoCo controller backend. Policy can only reach it through MoveIt actions."""
import math
import os
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from .contract import JOINTS, valid_action


class MujocoBackend(Node):
    def __init__(self):
        super().__init__("safe_mujoco_backend")
        import mujoco
        self.mj=mujoco
        path=Path(self.declare_parameter("model_xml","").value)
        if not path.is_file():raise ValueError("model_xml must point to the packaged MuJoCo scene")
        self.model=mujoco.MjModel.from_xml_path(str(path));self.data=mujoco.MjData(self.model)
        self.closed=False;self.last_render=0.;self.lock=threading.RLock();self.stopping=False;self.arm_busy=False;self.grip_busy=False
        self.address=[self.model.jnt_qposadr[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in JOINTS]
        self.actuators=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_ACTUATOR,n+"_act") for n in JOINTS]
        # Vendor conversion preserves actuator order; assert it before using control slots.
        if self.model.nu!=6:raise ValueError("MuJoCo scene must contain six actuators")
        for i,n in enumerate(JOINTS):
            joint=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n)
            if self.model.actuator_trnid[i,0]!=joint:raise ValueError("actuator ordering mismatch")
        self.model.opt.timestep=0.002
        initial=valid_action(self.declare_parameter("initial_positions",[0.,0.4,-1.3,-1.35,0.,0.]).value)
        self.data.ctrl[:]=initial;self.data.qpos[self.address]=initial;mujoco.mj_forward(self.model,self.data)
        self.pub=self.create_publisher(JointState,"/joint_states",10)
        self.event=self.create_publisher(String,"/edgepick/sim/event",10)
        self.group=ReentrantCallbackGroup()
        self.arm=ActionServer(self,FollowJointTrajectory,"/arm_group_controller/follow_joint_trajectory",self.execute_arm,goal_callback=self.arm_goal,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.grip=ActionServer(self,GripperCommand,"/grip_group_controller/gripper_cmd",self.execute_grip,goal_callback=self.grip_goal,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_subscription(Bool,"/edgepick/safe/estop",self.estop,10,callback_group=self.group)
        self.render=bool(self.declare_parameter("render",False).value)
        self.renderer=None
        self.bridge=CvBridge();self.tf=TransformBroadcaster(self)
        self.rgb=self.create_publisher(Image,"/camera/color/image_raw",10)
        self.depth=self.create_publisher(Image,"/camera/aligned_depth_to_color/image_raw",10)
        self.info=self.create_publisher(CameraInfo,"/camera/color/camera_info",10)
        self.attached=False;self.offset=None
        self.orange=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,"orange")
        self.site=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SITE,"grasp_point")
        self.thread=threading.Thread(target=self.physics,daemon=True);self.thread.start()
        self.create_timer(0.05,self.publish_state)
    def estop(self,m):
        if m.data:
            with self.lock:self.data.ctrl[:]=self.data.qpos[self.address];self.stopping=True
        else:self.stopping=False
    def arm_goal(self,request):
        trajectory=request.trajectory
        if trajectory.joint_names!=JOINTS[:5] or not trajectory.points:return GoalResponse.REJECT
        last=0
        try:
            for p in trajectory.points:
                duration=p.time_from_start.sec+p.time_from_start.nanosec*1e-9
                if len(p.positions)!=5 or duration<last or duration<0 or duration>60:return GoalResponse.REJECT
                valid_action(list(p.positions)+[0]);last=duration
        except (ValueError,TypeError):return GoalResponse.REJECT
        with self.lock:
            if self.arm_busy or self.stopping:return GoalResponse.REJECT
            self.arm_busy=True
        return GoalResponse.ACCEPT
    def grip_goal(self,request):
        with self.lock:
            if self.grip_busy or self.stopping or not math.isfinite(request.command.position) or not -1.6<=request.command.position<=0:return GoalResponse.REJECT
            self.grip_busy=True
        return GoalResponse.ACCEPT
    def execute_arm(self,goal):
        result=FollowJointTrajectory.Result()
        points=goal.request.trajectory.points
        with self.lock:start=self.data.qpos[self.address[:5]].copy()
        times=[0.0]+[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points]
        positions=[start]+[np.asarray(p.positions) for p in points]
        started=time.monotonic()
        try:
            while rclpy.ok():
                elapsed=time.monotonic()-started
                if goal.is_cancel_requested or self.stopping:
                    with self.lock:self.data.ctrl[:5]=self.data.qpos[self.address[:5]]
                    if goal.is_cancel_requested:goal.canceled()
                    else:goal.abort()
                    result.error_code=result.PATH_TOLERANCE_VIOLATED;return result
                index=min(len(times)-1,max(1,int(np.searchsorted(times,elapsed,side="right"))))
                span=times[index]-times[index-1]
                fraction=1.0 if span<=0 else min(1.0,(elapsed-times[index-1])/span)
                with self.lock:
                    self.data.ctrl[:5]=positions[index-1]+fraction*(positions[index]-positions[index-1])
                    error=np.max(np.abs(self.data.qpos[self.address[:5]]-positions[-1]))
                if elapsed>=times[-1] and error<0.04:goal.succeed();result.error_code=result.SUCCESSFUL;return result
                if elapsed>times[-1]+3:goal.abort();result.error_code=result.GOAL_TOLERANCE_VIOLATED;return result
                time.sleep(0.01)
            goal.abort();return result
        finally:
            with self.lock:self.arm_busy=False
    def execute_grip(self,goal):
        result=GripperCommand.Result();started=time.monotonic()
        try:
            with self.lock:self.data.ctrl[5]=goal.request.command.position
            while rclpy.ok() and time.monotonic()-started<4:
                with self.lock:result.position=float(self.data.qpos[self.address[5]])
                if goal.is_cancel_requested or self.stopping:
                    with self.lock:self.data.ctrl[5]=result.position
                    if goal.is_cancel_requested:goal.canceled()
                    else:goal.abort()
                    return result
                if abs(result.position-goal.request.command.position)<0.04:
                    result.reached_goal=True;goal.succeed();return result
                time.sleep(0.01)
            goal.abort();return result
        finally:
            with self.lock:self.grip_busy=False
    def physics(self):
        while rclpy.ok() and not self.closed:
            started=time.monotonic()
            with self.lock:
                for _ in range(5):
                    self.mj.mj_step(self.model,self.data)
                    grip=self.data.qpos[self.address[5]]
                    before=self.attached
                    if self.attached and grip> -0.2:self.attached=False
                    elif not self.attached and grip<=-0.5 and np.linalg.norm(self.data.xpos[self.orange]-self.data.site_xpos[self.site])<0.05:
                        self.attached=True;self.offset=self.data.xpos[self.orange]-self.data.site_xpos[self.site]
                    if self.attached:
                        j=self.model.body_jntadr[self.orange];address=self.model.jnt_qposadr[j]
                        self.data.qpos[address:address+3]=self.data.site_xpos[self.site]+self.offset
                        self.data.qvel[self.model.jnt_dofadr[j]:self.model.jnt_dofadr[j]+6]=0
                        self.mj.mj_forward(self.model,self.data)
                    if before!=self.attached:self.event.publish(String(data="attached" if self.attached else "released"))
            if self.render and time.monotonic()-self.last_render>=0.1:
                self.publish_image();self.last_render=time.monotonic()
            time.sleep(max(0,0.01-(time.monotonic()-started)))
    def publish_state(self):
        m=JointState(header=Header(stamp=self.get_clock().now().to_msg()),name=JOINTS)
        with self.lock:m.position=self.data.qpos[self.address].tolist()
        self.pub.publish(m)
    def publish_image(self):
        # GL context is created and used only on the physics/render thread.
        if self.renderer is None:self.renderer=self.mj.Renderer(self.model,height=240,width=320)
        camera=self.mj.mj_name2id(self.model,self.mj.mjtObj.mjOBJ_CAMERA,"yolo")
        header=Header(stamp=self.get_clock().now().to_msg(),frame_id="sim_camera_optical")
        with self.lock:
            self.renderer.update_scene(self.data,camera="yolo");rgb=self.renderer.render().copy()
            self.renderer.enable_depth_rendering();depth=self.renderer.render().copy();self.renderer.disable_depth_rendering()
            rotation=self.data.cam_xmat[camera].reshape(3,3)@np.diag([1,-1,-1])
            quaternion=np.zeros(4);self.mj.mju_mat2Quat(quaternion,rotation.ravel())
            transform=TransformStamped(header=Header(stamp=header.stamp,frame_id="base_link"),child_frame_id=header.frame_id)
            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=self.data.cam_xpos[camera].tolist()
            transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z=quaternion.tolist()
        self.tf.sendTransform(transform)
        for array,encoding,pub in ((rgb,"rgb8",self.rgb),(depth,"32FC1",self.depth)):
            msg=self.bridge.cv2_to_imgmsg(array,encoding);msg.header=header;pub.publish(msg)
        focal=120/math.tan(math.radians(self.model.cam_fovy[camera])/2)
        self.info.publish(CameraInfo(header=header,width=320,height=240,k=[focal,0.,159.5,0.,focal,119.5,0.,0.,1.],distortion_model="plumb_bob",d=[0.]*5))


def main(args=None):
    os.environ.setdefault("MUJOCO_GL","egl")
    rclpy.init(args=args);node=MujocoBackend();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.closed=True;node.thread.join(timeout=5)
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
