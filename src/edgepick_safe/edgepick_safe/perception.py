"""Registered RGB-D -> target, conservative obstacle voxels and fresh place review."""
import math
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import Header
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from .contract import fresh


def select_synchronized_pair(rgb_messages, depth_messages, tolerance):
    """Return the newest RGB/depth pair within ``tolerance`` seconds.

    The camera topics are independent sensor streams.  Selecting by newest
    message alone can pair an RGB frame with a stale depth frame, so the
    bounded buffers are searched for the newest valid pair instead.  The
    function is intentionally ROS-message agnostic to keep the policy easy to
    test and to avoid retaining an unbounded queue in the node.
    """
    if tolerance < 0:
        raise ValueError("synchronization tolerance must be non-negative")

    def stamp(message):
        header = message.header
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    best = None
    for rgb in rgb_messages:
        rgb_stamp = stamp(rgb)
        for depth in depth_messages:
            depth_stamp = stamp(depth)
            delta = abs(rgb_stamp - depth_stamp)
            if delta > tolerance:
                continue
            # Prefer the pair whose older member is newest.  If two pairs
            # have the same age, prefer the one with the smaller skew.
            key = (min(rgb_stamp, depth_stamp), -delta)
            if best is None or key > best[0]:
                best = (key, rgb, depth, min(rgb_stamp, depth_stamp))
    return None if best is None else best[1:]


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    v = np.array([q.x,q.y,q.z,q.w], dtype=float)
    if not np.isfinite(v).all() or abs(np.linalg.norm(v)-1) > 0.01:
        raise ValueError("Invalid TF quaternion")
    x,y,z,w = v / np.linalg.norm(v)
    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    translation = np.array([t.x,t.y,t.z])
    if not np.isfinite(translation).all(): raise ValueError("Invalid TF translation")
    return rotation, translation


def box(identifier, xyz, dimensions):
    obj = CollisionObject()
    obj.id = identifier
    obj.header.frame_id = "base_link"
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float,dimensions)))
    pose = Pose()
    pose.position.x,pose.position.y,pose.position.z = map(float,xyz)
    pose.orientation.w = 1.0
    obj.primitives, obj.primitive_poses = [primitive], [pose]
    return obj


def destination_visible_clear(depth, k, rotation, translation, center, radius, surface_z, margin):
    # All footprint grid samples must see the support surface; missing depth is unknown.
    samples = np.array([[center[0]+dx*radius,center[1]+dy*radius,surface_z]
                        for dx in np.linspace(-1,1,9) for dy in np.linspace(-1,1,9)])
    camera = (samples-translation) @ rotation
    if np.any(camera[:,2]<=0): return False
    uv = np.stack([camera[:,0]*k[0]/camera[:,2]+k[2],camera[:,1]*k[4]/camera[:,2]+k[5]],axis=1)
    for (u,v), expected in zip(uv,camera[:,2]):
        u,v = int(round(u)),int(round(v))
        if not (2<=u<depth.shape[1]-2 and 2<=v<depth.shape[0]-2): return False
        patch = depth[v-2:v+3,u-2:u+3]
        if not np.isfinite(patch).all() or np.any(patch<=0): return False
        if abs(float(np.median(patch))-expected)>margin: return False
    return True


class Perception(Node):
    def __init__(self):
        super().__init__("safe_perception")
        from ultralytics import YOLO
        self.model = YOLO(self.declare_parameter("yolo_model", "").value, task="detect")
        detector_device = str(self.declare_parameter("detector_device", "auto").value)
        if detector_device == "auto":
            try:
                import torch
                detector_device = "cuda:0" if torch.cuda.is_available() else "cpu"
            except ImportError:
                detector_device = "cpu"
        self.detector_device = detector_device
        self.detector_half = bool(self.declare_parameter("detector_half", True).value)
        if not detector_device.startswith("cuda"):
            self.detector_half = False
        self.label = self.declare_parameter("target_label", "cube").value
        self.confidence = float(self.declare_parameter("confidence",0.5).value)
        self.registered = self.declare_parameter("registered_depth_confirmed",False).value
        self.destination = np.array(self.declare_parameter("destination",[0.12,0.30,0.04]).value)
        self.radius = float(self.declare_parameter("place_radius",0.06).value)
        self.zone_side = float(self.declare_parameter("place_zone_side",0.05).value)
        self.surface = float(self.declare_parameter("surface_z",0.0).value)
        self.voxel = float(self.declare_parameter("voxel_size",0.025).value)
        self.max_voxels = int(self.declare_parameter("max_voxels",4000).value)
        self.age = float(self.declare_parameter("observation_timeout",1.0).value)
        self.sync_tolerance = float(self.declare_parameter("rgbd_sync_tolerance",0.1).value)
        self.buffer_size = int(self.declare_parameter("sensor_buffer_size",4).value)
        if (self.voxel<=0 or self.radius<=0 or self.zone_side<=0 or self.max_voxels<1 or
                self.sync_tolerance<0 or self.buffer_size<1):
            raise ValueError("Invalid scene parameters")
        # Sensor callbacks only enqueue bounded references.  Expensive YOLO,
        # depth conversion, and point-cloud construction run in one worker so
        # a slow inference cannot block the ROS callback thread.
        self.rgb_buffer = deque(maxlen=self.buffer_size)
        self.depth_buffer = deque(maxlen=self.buffer_size)
        self.info = None
        self.sensor_lock = threading.Lock()
        self.bridge = CvBridge()
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.pending=None; self.old_ids=set(); self.last_stamp=0.0
        self.work_future = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="safe-perception")
        for kind, default, callback in [("image","/camera/color/image_raw",self.on_rgb),("depth","/camera/aligned_depth_to_color/image_raw",self.on_depth)]:
            self.create_subscription(Image,self.declare_parameter(kind+"_topic",default).value,callback,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,self.declare_parameter("camera_info_topic","/camera/color/camera_info").value,self.on_info,qos_profile_sensor_data)
        self.target_pub=self.create_publisher(PointStamped,"/edgepick/safe/target",10)
        self.scene_pub=self.create_publisher(Header,"/edgepick/safe/scene_applied",10)
        self.clear_pub=self.create_publisher(Header,"/edgepick/safe/destination_clear",10)
        self.apply=self.create_client(ApplyPlanningScene,"/edgepick/safe/apply_observation")
        self.rate_hz = float(self.declare_parameter("perception_rate_hz", 5.0).value)
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0:
            raise ValueError("perception_rate_hz must be positive")
        self.create_timer(1.0 / self.rate_hz,self.tick)
        self.last_error=0.0

    def on_rgb(self,m):
        with self.sensor_lock:
            self.rgb_buffer.append(m)

    def on_depth(self,m):
        with self.sensor_lock:
            self.depth_buffer.append(m)

    def on_info(self,m):
        with self.sensor_lock:
            self.info=m

    @staticmethod
    def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

    def tick(self):
        if not self.registered or self.info is None: return
        if self.pending is not None and not self.pending.done(): return
        try:
            if self.work_future is not None:
                if not self.work_future.done(): return
                result = self.work_future.result()
                self.work_future = None
                if result is None: return
                stamp, target, objects, clear = result
                now = self.get_clock().now().nanoseconds / 1e9
                # Inference may finish after a newer camera frame arrives. A
                # stale scene must never be sent to the planner.
                with self.sensor_lock:
                    newest = select_synchronized_pair(
                        tuple(self.rgb_buffer), tuple(self.depth_buffer), self.sync_tolerance
                    )
                if (not fresh(now, stamp, self.age) or not self.apply.service_is_ready() or
                        (newest is not None and newest[2] > stamp)):
                    return
                ids={o.id for o in objects}
                for old in self.old_ids-ids:
                    objects.append(CollisionObject(id=old,operation=CollisionObject.REMOVE))
                scene=PlanningScene(is_diff=True); scene.robot_state.is_diff=True; scene.world.collision_objects=objects
                request=ApplyPlanningScene.Request(scene=scene)
                header=Header(stamp=rclpy.time.Time(seconds=stamp).to_msg(),frame_id="base_link")
                msg=PointStamped(header=header); msg.point.x,msg.point.y,msg.point.z=map(float,target)
                self.target_pub.publish(msg)
                self.pending=self.apply.call_async(request)
                self.pending.add_done_callback(lambda future: self.applied(future,header,clear,ids))
                return

            if not self.apply.service_is_ready(): return
            with self.sensor_lock:
                info = self.info
                pair = select_synchronized_pair(tuple(self.rgb_buffer), tuple(self.depth_buffer), self.sync_tolerance)
            if info is None or pair is None: return
            rgb, dep, stamp = pair
            if stamp<=self.last_stamp or not fresh(self.get_clock().now().nanoseconds/1e9,stamp,self.age): return
            if rgb.header.frame_id!=dep.header.frame_id or info.header.frame_id!=rgb.header.frame_id: return
            if (rgb.width,rgb.height)!=(dep.width,dep.height) or (info.width,info.height)!=(rgb.width,rgb.height): return
            self.last_stamp=stamp
            self.work_future=self.worker.submit(self.process_observation,rgb,dep,info,stamp)
        except Exception as exc:
            self.work_future = None
            if time.monotonic()-self.last_error>2:
                self.get_logger().warning(f"Perception closed: {exc}");self.last_error=time.monotonic()

    def process_observation(self, rgb, dep, info, stamp):
        """Run the expensive perception path on the single worker thread."""
        if info.k[0]<=0 or info.k[4]<=0 or not np.isfinite(info.k).all(): raise ValueError("Invalid intrinsics")
        if dep.encoding not in ("16UC1","32FC1"): raise ValueError("Unsupported depth encoding")
        depth=self.bridge.imgmsg_to_cv2(dep).astype(np.float32)
        if dep.encoding=="16UC1": depth*=0.001
        if np.count_nonzero(np.isfinite(depth)&(depth>0.05)&(depth<1.5))<depth.size*0.25: raise ValueError("Insufficient depth coverage")
        tf=self.buffer.lookup_transform("base_link",dep.header.frame_id,rclpy.time.Time.from_msg(dep.header.stamp),timeout=Duration(seconds=0.02))
        rotation,translation=transform_matrix(tf.transform)
        frame=self.bridge.imgmsg_to_cv2(rgb,"bgr8")
        result=self.model.predict(
            frame,
            conf=self.confidence,
            device=self.detector_device,
            half=self.detector_half,
            verbose=False,
        )[0]
        targets=[b for b in result.boxes if str(result.names[int(b.cls.item())])==self.label]
        if len(targets)!=1: raise ValueError("Target missing or ambiguous")
        x1,y1,x2,y2=targets[0].xyxy[0].cpu().numpy()
        u,v=int((x1+x2)/2),int((y1+y2)/2)
        if not (2<=u<depth.shape[1]-2 and 2<=v<depth.shape[0]-2): raise ValueError("Target outside image")
        patch=depth[v-2:v+3,u-2:u+3]; patch=patch[np.isfinite(patch)&(patch>0.05)&(patch<1.5)]
        if patch.size<13: raise ValueError("Target depth missing")
        z=float(np.median(patch)); target_camera=np.array([(u-info.k[2])*z/info.k[0],(v-info.k[5])*z/info.k[4],z])
        target=target_camera@rotation.T+translation
        points_v,points_u=np.mgrid[0:depth.shape[0]:3,0:depth.shape[1]:3]
        z=depth[::3,::3].ravel(); pu,pv=points_u.ravel(),points_v.ravel()
        valid=np.isfinite(z)&(z>0.05)&(z<1.5); z,pu,pv=z[valid],pu[valid],pv[valid]
        camera=np.stack([(pu-info.k[2])*z/info.k[0],(pv-info.k[5])*z/info.k[4],z],axis=1)
        points=camera@rotation.T+translation
        workspace=(np.abs(points[:,0])<0.45)&(np.abs(points[:,1])<0.45)&(points[:,2]>self.surface+0.015)&(points[:,2]<0.65)
        # Only the selected object's small contact volume is excluded. No robot-wide blind zone.
        workspace &= np.linalg.norm(points-target,axis=1)>0.04
        voxels=np.unique(np.floor(points[workspace]/self.voxel).astype(int),axis=0)
        if len(voxels)>self.max_voxels: raise ValueError("Scene too dense; refusing truncation")
        objects=[box("observed_"+"_".join(map(str,v)),(v+0.5)*self.voxel,[self.voxel*1.2]*3) for v in voxels]
        objects.append(box("support_surface",[0,0,self.surface-0.025],[1.0,1.0,0.05]))
        clear=destination_visible_clear(depth,info.k,rotation,translation,self.destination,self.zone_side/2,self.surface,0.015)
        if not clear: objects.append(box("destination_blocked",self.destination,[self.zone_side,self.zone_side,0.08]))
        return stamp, target, objects, clear

    def destroy_node(self):
        self.worker.shutdown(wait=True)
        return super().destroy_node()

    def applied(self,future,header,clear,ids):
        try:
            if future.result().success:
                self.old_ids=ids
                self.scene_pub.publish(header)
                if clear: self.clear_pub.publish(header)
        except Exception as exc:
            self.get_logger().error(f"Scene update failed: {exc}")


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=Perception();rclpy.spin(node)
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
