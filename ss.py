import os
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
import subprocess
import signal
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import asyncio
import websockets
import threading
import json
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
#import tf2_sensor_msgs.tf2_sensor_msgs
import math
import numpy as np
from builtin_interfaces.msg import Time as BuiltinTime
from rosidl_runtime_py import message_to_ordereddict
import base64
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowPath
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.task import Future
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from geometry_msgs.msg import Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import PointCloud2, PointField

def quaternion_to_matrix(quaternion):
    """Convert a quaternion into a 4x4 transformation matrix."""
    x, y, z, w = quaternion
    tx = 2.0 * x
    ty = 2.0 * y
    tz = 2.0 * z
    twx = tx * w
    twy = ty * w
    twz = tz * w
    txx = tx * x
    txy = ty * x
    txz = tz * x
    tyy = ty * y
    tyz = tz * y
    tzz = tz * z

    return np.array([
        [1.0 - (tyy + tzz), txy - twz, txz + twy, 0.0],
        [txy + twz, 1.0 - (txx + tzz), tyz - twx, 0.0],
        [txz - twy, tyz + twx, 1.0 - (txx + tyy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

def pointcloud2_to_xyz_array(cloud_msg):
    """Convert a sensor_msgs/PointCloud2 message to a numpy array."""
    fmt = "fff"  # Assume XYZ-only PointCloud2
    dtype = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)])
    dtype = dtype.newbyteorder('>' if cloud_msg.is_bigendian else '<')
    xyz_array = np.frombuffer(cloud_msg.data, dtype=dtype)
    return np.vstack([xyz_array["x"], xyz_array["y"], xyz_array["z"]]).T

def xyz_array_to_pointcloud2(xyz_array, frame_id, stamp):
    """Convert a numpy array to a sensor_msgs/PointCloud2 message."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    header = PointCloud2().header
    header.frame_id = frame_id
    header.stamp = stamp
    data = xyz_array.astype(np.float32).tobytes()
    return PointCloud2(
        header=header,
        height=1,
        width=len(xyz_array),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * len(xyz_array),
        data=data,
    )

def transform_pointcloud(cloud_msg, transform_stamped):
    """Transform a PointCloud2 message using a given TransformStamped."""
    # Convert transform to a 4x4 matrix
    trans = transform_stamped.transform.translation
    rot = transform_stamped.transform.rotation
    transform_matrix = quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
    transform_matrix[:3, 3] = [trans.x, trans.y, trans.z]

    # Convert point cloud to XYZ numpy array
    xyz_array = pointcloud2_to_xyz_array(cloud_msg)

    # Apply transformation
    transformed_xyz = np.dot(xyz_array, transform_matrix[:3, :3].T) + transform_matrix[:3, 3]

    # Convert back to PointCloud2
    return xyz_array_to_pointcloud2(transformed_xyz, transform_stamped.header.frame_id, cloud_msg.header.stamp)



class WebSocketROS2Bridge(Node):
    def __init__(self, clients):
        super().__init__('websocket_ros2_bridge')
        self.clients1 = clients
        self.websocket_uri = "ws://0.0.0.0:8888"
        self.map_save_path = os.path.join(os.getcwd(), "amr_configs/maps")
        #self.websocket_server = None
        # Start the WebSocket server in a separate thread
        self.websocket_thread = threading.Thread(target=self.start_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        

       

        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            map_qos)
        self.subscription_map  # Prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.subscription_scan

        self.navpose_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pathfollow_action_client = ActionClient(self, FollowPath, '/follow_path')
        self.nav_through_poses_action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.pose_array_publisher = self.create_publisher(PoseArray, 'pose_array', 10)
        self.smooth_path_publisher = self.create_publisher(Path, 'smoothed_path', 10)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.launch_services = {}
        self.launch_threads = {}
        self.launch_tasks = {}

        # Global UI State
        self.current_ui_state = {
            "selected_map": None,
            "selected_waypoint_name": None,
            "selected_path_name": None
        }

        # Message Caching for new clients
        self.last_map_json = None
        self.last_pose_json = None

        


        #self.publisher = self.create_publisher(PointCloud2, '/scan_pointcloud', 10)
        #self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose_in_map', 10)
    
    def send_goal_pose(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.navpose_action_client.wait_for_server()

        self._send_goal_future = self.navpose_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback and broadcast to WebSocket clients"""
        feedback = feedback_msg.feedback
        
        # Prepare feedback data
        feedback_data = {
            "type": "nav_feedback",
            "data": {
                "distance_remaining": feedback.distance_remaining if hasattr(feedback, 'distance_remaining') else None,
                "navigation_time": {
                    "sec": feedback.navigation_time.sec if hasattr(feedback, 'navigation_time') else 0
                } if hasattr(feedback, 'navigation_time') else None,
                "current_pose": {
                    "pose": {
                        "position": {
                            "x": feedback.current_pose.pose.position.x if hasattr(feedback, 'current_pose') else 0,
                            "y": feedback.current_pose.pose.position.y if hasattr(feedback, 'current_pose') else 0,
                        }
                    }
                } if hasattr(feedback, 'current_pose') else None
            }
        }
        
        # Broadcast to all WebSocket clients
        # asyncio.run(self.send_data_to_clients(json.dumps(feedback_data)))
        self.broadcast_message(json.dumps(feedback_data))

    def send_goal_path(self, goal_path: FollowPath):


        self.pathfollow_action_client.wait_for_server()

        self._send_goal_path_future = self.pathfollow_action_client.send_goal_async(goal_path)
        self._send_goal_path_future.add_done_callback(self.goal_response_callback)

    def send_goal_through_poses(self, poses):
        self.nav_through_poses_action_client.wait_for_server()

        self._send_goal_path_future = self.nav_through_poses_action_client.send_goal_async(poses)
        self._send_goal_path_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            # Broadcast rejection
            # Broadcast rejection
            self.broadcast_message(json.dumps({
                "type": "nav_result",
                "success": False,
                "error": "Goal rejected by navigation server"
            }))
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        status = future.result().status
        
        # Broadcast result to WebSocket clients
        # Broadcast result to WebSocket clients
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            self.broadcast_message(json.dumps({
                "type": "nav_result",
                "success": True
            }))
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
            self.broadcast_message(json.dumps({
                "type": "nav_result",
                "success": False,
                "error": f"Navigation failed (status: {status})"
            }))
        

    def map_callback(self, msg):
        # Prepare map data to be sent through WebSocket
        map_data_json = self.ros2_msg_to_json('map',msg)
        self.last_map_json = map_data_json
        # asyncio.run(self.send_data_to_clients(map_data_json))
        self.broadcast_message(map_data_json)

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud2
        cloud_msg = self.laserscan_to_pointcloud2(scan_msg)
        current_time = rclpy.time.Time()

        # Convert rclpy.time.Time to builtin_interfaces.msg.Time
        time_msg = BuiltinTime(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])

        cloud_msg.header.stamp = time_msg #scan_msg.header.stamp  # Ensure correct timestamp
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",  # target frame
                    cloud_msg.header.frame_id,  # source frame (laser frame)
                    rclpy.time.Time.from_msg(scan_msg.header.stamp),  # time at which the transform is needed
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                # Fallback to latest available transform if exact time fails
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    cloud_msg.header.frame_id,
                    rclpy.time.Time()
                )
            
            # Robot pose in map frame
            transform_p = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            # Laser scan in map frame as pointcloud2
            #transformed_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)
            transformed_cloud = transform_pointcloud(cloud_msg, transform)

            if transformed_cloud:
                # Publish Transformed PointCloud2
                #self.publisher.publish(transformed_cloud)
        
                string = ''.join(chr(i) for i in transformed_cloud.data)
                # Convert each integer to its byte representation and concatenate
                byte_array = bytes(transformed_cloud.data)

                # Encode the byte array to Base64
                base64_string = base64.b64encode(byte_array).decode('utf-8')
      
                transformed_cloud_json = self.ros2_msg_to_json('scan_pointcloud',transformed_cloud)
                dictionary_tc = json.loads(transformed_cloud_json)
                dictionary_tc['data']['data'] = base64_string
                #print(dictionary_tc['data']['data']) 
    
                #print(dictionary_tc['data']['data']) 
    
                # asyncio.run(self.send_data_to_clients(json.dumps(dictionary_tc)))
                self.broadcast_message(json.dumps(dictionary_tc))

            
                        # Create a PoseStamped message to publish
            pose_in_map = PoseStamped()
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            pose_in_map.header.frame_id = 'map'
            # Construct the Pose message from the transform
            pose_in_map.pose.position.x = transform_p.transform.translation.x
            pose_in_map.pose.position.y = transform_p.transform.translation.y
            pose_in_map.pose.position.z = transform_p.transform.translation.z
            
            pose_in_map.pose.orientation.x = transform_p.transform.rotation.x
            pose_in_map.pose.orientation.y = transform_p.transform.rotation.y
            pose_in_map.pose.orientation.z = transform_p.transform.rotation.z
            pose_in_map.pose.orientation.w = transform_p.transform.rotation.w
  
            # Publish the pose
            #self.pose_publisher.publish(pose_in_map)
            pose_in_map_json = self.ros2_msg_to_json('robot_pose_in_map',pose_in_map)
            self.last_pose_json = pose_in_map_json
            # asyncio.run(self.send_data_to_clients(pose_in_map_json))
            self.broadcast_message(pose_in_map_json)
       
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform pointcloud: {e}")
            return None


    def laserscan_to_pointcloud2(self, scan_msg):
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:  # Ignore invalid ranges
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # Assuming a 2D scan, z is 0
                points.append([x, y, z])
            angle += scan_msg.angle_increment

        header = scan_msg.header
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        return cloud_msg
    #-----------  Handle incomming message ------------   
    def convert_json_pose_to_poasestamp(self, p):
        pose_s = PoseStamped()
        pose_s.header.stamp = self.get_clock().now().to_msg()
        pose_s.header.frame_id = 'map'
        # Construct the Pose message from the transform
        pose_s.pose.position.x = float(p['position']['x'])
        pose_s.pose.position.y = float(p['position']['y'])
        pose_s.pose.position.z = float(p['position']['z'])
        
        pose_s.pose.orientation.x = float(p['orientation']['x'])
        pose_s.pose.orientation.y = float(p['orientation']['y'])
        pose_s.pose.orientation.z = float(p['orientation']['z'])
        pose_s.pose.orientation.w = float(p['orientation']['w'])
        
        return pose_s

    def publish_initial_pose(self, p_data):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = float(p_data['position']['x'])
        msg.pose.pose.position.y = float(p_data['position']['y'])
        msg.pose.pose.position.z = float(p_data['position']['z'])
        
        msg.pose.pose.orientation.x = float(p_data['orientation']['x'])
        msg.pose.pose.orientation.y = float(p_data['orientation']['y'])
        msg.pose.pose.orientation.z = float(p_data['orientation']['z'])
        msg.pose.pose.orientation.w = float(p_data['orientation']['w'])
        
        # Set covariance (copied from typical AMCL initial pose)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]
                               
        self.initial_pose_publisher.publish(msg)
        # self.get_logger().info('Published initial pose')
    
    def create_path(self):
        path = Path()
        path.header.frame_id = 'map'
        
        # Create a few poses as an example path
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = i * 0.2
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Neutral orientation
            path.poses.append(pose)
    
        return path

    def convert_json_pose_array_to_path(self, ps):
        path = Path()
        path.header.frame_id = 'map'
        pss = []
        for i in range(len(ps)):
            pose_s = PoseStamped()
            pose_s.header.stamp.sec = 0
            pose_s.header.stamp.nanosec = 0
            pose_s.header.frame_id = 'map'
            # Construct the Pose message from the transform
            pose_s.pose.position.x = float(ps[i]['position']['x'])
            pose_s.pose.position.y = float(ps[i]['position']['y'])
            pose_s.pose.position.z = 0.0
            
            pose_s.pose.orientation.x = float(ps[i]['orientation']['x'])
            pose_s.pose.orientation.y = float(ps[i]['orientation']['y'])
            pose_s.pose.orientation.z = float(ps[i]['orientation']['z'])
            pose_s.pose.orientation.w = float(ps[i]['orientation']['w'])
            pss.append(pose_s)
        path.poses = pss
        return path
    
    def convert_json_pose_array_to_pose_array(self, ps):
        pss = []
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp.sec = 0
        pose_array_msg.header.stamp.nanosec = 0
        pose_array_msg.header.frame_id = 'map'
        for i in range(len(ps)):
            pose_s = Pose()
         
            # Construct the Pose message from the transform
            pose_s.position.x = float(ps[i]['position']['x'])
            pose_s.position.y = float(ps[i]['position']['y'])
            pose_s.position.z = 0.0
            
            pose_s.orientation.x = float(ps[i]['orientation']['x'])
            pose_s.orientation.y = float(ps[i]['orientation']['y'])
            pose_s.orientation.z = float(ps[i]['orientation']['z'])
            pose_s.orientation.w = float(ps[i]['orientation']['w'])
            pss.append(pose_s)
        pose_array_msg.poses = pss
        return pose_array_msg
    
    def convert_json_pose_array_to_poses(self, ps):
        pss = []
        for i in range(len(ps)):
            pose_s = PoseStamped()
            pose_s.header.stamp.sec = 0
            pose_s.header.stamp.nanosec = 0
            pose_s.header.frame_id = 'map'
            
            # Construct the Pose message from the transform
            pose_s.pose.position.x = float(ps[i]['position']['x'])
            pose_s.pose.position.y = float(ps[i]['position']['y'])
            pose_s.pose.position.z = 0.0
            
            pose_s.pose.orientation.x = float(ps[i]['orientation']['x'])
            pose_s.pose.orientation.y = float(ps[i]['orientation']['y'])
            pose_s.pose.orientation.z = float(ps[i]['orientation']['z'])
            pose_s.pose.orientation.w = float(ps[i]['orientation']['w'])
            pss.append(pose_s)
        return pss
    

    
    def convert_json_pose_array_to_poses(self, ps):
        pss = []    
        for i in range(len(ps)):
            pose_s = PoseStamped()
            pose_s.header.stamp.sec = 0
            pose_s.header.stamp.nanosec = 0
            pose_s.header.frame_id = 'map'
            # Construct the Pose message from the transform
            pose_s.pose.position.x = float(ps[i]['position']['x'])
            pose_s.pose.position.y = float(ps[i]['position']['y'])
            pose_s.pose.position.z = 0.0
            
            pose_s.pose.orientation.x = float(ps[i]['orientation']['x'])
            pose_s.pose.orientation.y = float(ps[i]['orientation']['y'])
            pose_s.pose.orientation.z = float(ps[i]['orientation']['z'])
            pose_s.pose.orientation.w = float(ps[i]['orientation']['w'])
            pss.append(pose_s) 
        return pss
        
  
    async def websocket_handler(self, websocket, path):
        self.clients1.add(websocket)
        
        # Send current map list to the new client
        maps = self.get_saved_maps()
        await websocket.send(json.dumps({"type": "map_list", "data": maps}))

        # Send current process statuses
        for launch_name in self.launch_services:
            await websocket.send(json.dumps({
                "type": "process_status",
                "name": launch_name,
                "status": "running"
            }))

        # Send current UI state
        await websocket.send(json.dumps({
            "type": "ui_state",
            "data": self.current_ui_state
        }))

        # Send cached map and pose if available
        if self.last_map_json:
            await websocket.send(self.last_map_json)
        if self.last_pose_json:
            await websocket.send(self.last_pose_json)

        try:
            async for message in websocket:
                # Process incoming messages here
                #print(f"Received message: {message}")
                json_dada = json.loads(message)
                
                if json_dada['type'] == "ui_state":
                    # Update global state
                    for key in json_dada['data']:
                        if key in self.current_ui_state:
                            self.current_ui_state[key] = json_dada['data'][key]
                    # Broadcast to everyone else
                    await self.send_data_to_clients(json.dumps({
                        "type": "ui_state",
                        "data": self.current_ui_state
                    }))
                
                elif(json_dada['type'] == "action"):
                    if(json_dada['name'] == "navtopose"):
                        print(json_dada['data'])
                        pp = self.convert_json_pose_to_poasestamp(json_dada['data'])
                        print(pp)
                        self.send_goal_pose(pp)
                    if(json_dada['name'] == "set_pose"):
                        self.publish_initial_pose(json_dada['data'])

                    if(json_dada['name'] == "pathfollow"):
                        print("Get follow path")
                        try:
                            # --- NavigateThroughPoses ---
                            poses_p = self.convert_json_pose_array_to_poses(json_dada['data'])
                            
                            # Publish for visualization (Optional, might be heavy if big)
                            parray = self.convert_json_pose_array_to_pose_array(json_dada['data'])
                            self.pose_array_publisher.publish(parray)
                            
                            goal_poses = NavigateThroughPoses.Goal()
                            goal_poses.poses = poses_p
                            
                            # print("Sending goal poses...") # Reduce spam
                            self.send_goal_through_poses(goal_poses)
                        except Exception as e:
                            print(f"Error in pathfollow: {e}")
                            import traceback
                            traceback.print_exc()

                    if(json_dada['name'] == "gotowaypoint_by_name"):
                        data = json_dada.get('data', {})
                        map_name = data.get('map_name')
                        wp_name = data.get('waypoint_name')
                        
                        if not map_name or not wp_name:
                            self.broadcast_message(json.dumps({
                                "type": "nav_result",
                                "success": False,
                                "error": "Missing map_name or waypoint_name"
                            }))
                        else:
                            map_base = os.path.splitext(map_name)[0]
                            map_data = self.load_map_data(map_base)
                            waypoints = map_data.get("waypoints", [])
                            
                            target_wp = next((wp for wp in waypoints if wp.get('name') == wp_name), None)
                            
                            if target_wp:
                                pp = self.convert_json_pose_to_poasestamp(target_wp)
                                self.send_goal_pose(pp)
                                self.broadcast_message(json.dumps({
                                    "type": "nav_status",
                                    "text": f"Navigating to waypoint: {wp_name}",
                                    "color": "blue"
                                }))
                            else:
                                self.broadcast_message(json.dumps({
                                    "type": "nav_result",
                                    "success": False,
                                    "error": f"Waypoint '{wp_name}' not found in map '{map_name}'"
                                }))
                
                    
                elif(json_dada['type'] == "topic"):
                    if(json_dada['name'] == "dummytopic"):
                        pass
                elif(json_dada['type'] == "service"):
                    if(json_dada['name'] == "dummyservice"):
                        pass
                elif(json_dada['type'] == "process"):
                    if(json_dada['name'] == "upstart"):
                        launch_file_1 = os.path.join(os.getcwd(), 'minimal.py')
                        self.start_launch("minimal", launch_file_1)
                    if(json_dada['name'] == "stop_upstart"):
                        self.stop_launch('minimal')
                    if(json_dada['name'] == "start_slam"):
                        launch_file_2 = os.path.join(os.getcwd(), 'slam_async_nav.py')
                        self.start_launch("slam_nav", launch_file_2)
                    if(json_dada['name'] == "stop_slam"):
                        self.stop_launch('slam_nav')
                    if(json_dada['name'] == "save_map"):
                        map_name = json_dada.get('data', 'my_map')
                        map_path = os.path.join(self.map_save_path, map_name)
                        # Use --free 0.15 to ensure unknown space (approx 0.196) is not classified as free (0.25 default)
                        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path, '--free', '0.15']
                        try:
                            subprocess.Popen(cmd)
                            print(f"Saving map to {map_path}")
                            # Give it a moment to save, then update clients
                            asyncio.create_task(self.delayed_map_list_update())
                        except Exception as e:
                            print(f"Failed to save map: {e}")
                    if(json_dada['name'] == "start_nav"):
                        # Start navigation stack with a specific map
                        map_name = json_dada.get('data')
                        use_keepout = json_dada.get('use_keepout', False)
                        if map_name:
                            if not map_name.endswith('.yaml'): map_name += '.yaml'
                            map_path = os.path.join(self.map_save_path, map_name)
                            launch_file_nav = os.path.join(os.getcwd(), 'start_nav.py')
                            
                            args = [f"map:={map_path}"]
                            if use_keepout:
                                args.append("use_keepout:=true")
                                # Use a specific params file for keepout if desired, 
                                # or keep using the default and expect the user to have configured it.
                                # For now, we'll just enable the keepout servers.
                            
                            self.start_launch("nav_stack", launch_file_nav, args=args)
                    if(json_dada['name'] == "stop_nav"):
                        self.stop_launch('nav_stack')

                    if(json_dada['name'] == "save_mask"):
                        # Save the keepout mask as a PGM image and YAML metadata
                        map_name = json_dada.get('map_name')
                        mask_data_b64 = json_dada.get('mask_data') # Base64 encoded PGM or raw bytes
                        
                        if map_name and mask_data_b64:
                            map_base = os.path.splitext(map_name)[0]
                            mask_pgm_path = os.path.join(self.map_save_path, f"{map_base}_mask.pgm")
                            mask_yaml_path = os.path.join(self.map_save_path, f"{map_base}_mask.yaml")
                            
                            try:
                                # Decode base64 data
                                # Decode base64 data
                                mask_bytes = base64.b64decode(mask_data_b64.split(',')[-1])
                                
                                # Use PIL to convert PNG (with transparency/red) to Grayscale PGM (Black/White)
                                from PIL import Image
                                import io
                                
                                img = Image.open(io.BytesIO(mask_bytes)).convert("RGBA")
                                # Create a new grayscale image (L)
                                # ROS2 Map Server: 0 is occupied (black), 255 is free (white)
                                # Our UI draws red/opaque for keepout, transparent for free.
                                
                                width, height = img.size
                                grayscale_img = Image.new("L", (width, height), 255) # Start all white (free)
                                
                                pixels = img.load()
                                gs_pixels = grayscale_img.load()
                                
                                for y in range(height):
                                    for x in range(width):
                                        r, g, b, a = pixels[x, y]
                                        # If pixel is not fully transparent, treat it as keepout (black)
                                        if a > 0:
                                            gs_pixels[x, y] = 0
                                
                                grayscale_img.save(mask_pgm_path)
                                
                                # Create YAML metadata for the mask
                                # We assume the mask has the same resolution/origin as the main map
                                # Users might need to adjust this if they use a different scale.
                                main_yaml_path = os.path.join(self.map_save_path, f"{map_base}.yaml")
                                import yaml # Assuming pyyaml is available or we can write manually
                                
                                yaml_content = {
                                    'image': f"{map_base}_mask.pgm",
                                    'resolution': 0.05,
                                    'origin': [0.0, 0.0, 0.0],
                                    'negate': 0,
                                    'occupied_thresh': 0.65,
                                    'free_thresh': 0.196,
                                    'mode': 'trinary'
                                }
                                
                                if os.path.exists(main_yaml_path):
                                    try:
                                        with open(main_yaml_path, 'r') as f:
                                            main_yaml = yaml.safe_load(f)
                                            # Copy alignment-critical fields
                                            for field in ['resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh', 'mode']:
                                                if field in main_yaml:
                                                    yaml_content[field] = main_yaml[field]
                                    except Exception as e:
                                        print(f"Error reading main yaml for mask: {e}")

                                with open(mask_yaml_path, 'w') as f:
                                    # Use flow style for origin if it's a list, matching common ROS2 style
                                    yaml.dump(yaml_content, f, default_flow_style=False)
                                
                                self.broadcast_message(json.dumps({"type": "mask_saved", "success": True, "map_name": map_name}))
                                print(f"Saved mask for {map_name}")
                            except Exception as e:
                                print(f"Failed to save mask: {e}")
                                self.broadcast_message(json.dumps({"type": "mask_saved", "success": False, "error": str(e)}))
                    if(json_dada['name'] == "delete_map"):
                        map_name = json_dada.get('data', '')
                        if map_name:
                            # Standardize map name (remove .yaml if present)
                            map_base = os.path.splitext(map_name)[0]
                            map_yaml = os.path.join(self.map_save_path, f"{map_base}.yaml")
                            map_pgm = os.path.join(self.map_save_path, f"{map_base}.pgm")
                            map_json = os.path.join(self.map_save_path, f"{map_base}.json")
                            try:
                                # Delete all associated files
                                for file_path in [map_yaml, map_pgm, map_json]:
                                    if os.path.exists(file_path):
                                        os.remove(file_path)
                                        print(f"Deleted {file_path}")
                                
                                # Update map list
                                asyncio.create_task(self.delayed_map_list_update())
                            except Exception as e:
                                print(f"Error deleting map files for {map_name}: {e}")

                    if(json_dada['name'] == "save_waypoint"):
                        data = json_dada.get('data')
                        if data:
                            map_name = data.get('map_name')
                            waypoint = data.get('waypoint')
                            if map_name and waypoint:
                                map_base = os.path.splitext(map_name)[0]
                                self.save_map_data(map_base, "waypoints", waypoint)
                                # Broadcast updated list
                                asyncio.create_task(self.broadcast_waypoint_list(websocket, self.load_map_data(map_base).get("waypoints", [])))

                    if(json_dada['name'] == "save_path"):
                        data = json_dada.get('data')
                        if data:
                            map_name = data.get('map_name')
                            path_data = data.get('path')
                            if map_name and path_data:
                                map_base = os.path.splitext(map_name)[0]
                                self.save_map_data(map_base, "paths", path_data)
                                # Broadcast updated lists
                                asyncio.create_task(self.broadcast_path_list(websocket, self.load_map_data(map_base).get("paths", [])))

                    if(json_dada['name'] == "load_waypoints"):
                        map_name = json_dada.get('data')
                        if map_name:
                            map_base = os.path.splitext(map_name)[0]
                            data = self.load_map_data(map_base)
                            asyncio.create_task(self.broadcast_waypoint_list(websocket, data.get("waypoints", [])))
                            asyncio.create_task(self.broadcast_path_list(websocket, data.get("paths", [])))

                    if(json_dada['name'] == "delete_waypoint"):
                        data = json_dada.get('data')
                        if data:
                            map_name = data.get('map_name')
                            wp_name = data.get('waypoint_name')
                            if map_name and wp_name:
                                map_base = os.path.splitext(map_name)[0]
                                self.delete_map_item(map_base, "waypoints", wp_name)
                                asyncio.create_task(self.broadcast_waypoint_list(websocket, self.load_map_data(map_base).get("waypoints", [])))

                    if(json_dada['name'] == "delete_path"):
                        data = json_dada.get('data')
                        if data:
                            map_name = data.get('map_name')
                            path_name = data.get('path_name')
                            if map_name and path_name:
                                map_base = os.path.splitext(map_name)[0]
                                self.delete_map_item(map_base, "paths", path_name)
                                asyncio.create_task(self.broadcast_path_list(websocket, self.load_map_data(map_base).get("paths", [])))

                
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients1.remove(websocket)

    async def delayed_map_list_update(self):
        await asyncio.sleep(2)
        await self.broadcast_map_list()

    def get_saved_maps(self):
        maps = []
        try:
            if os.path.exists(self.map_save_path):
                for file in os.listdir(self.map_save_path):
                    if file.endswith(".yaml"):
                        maps.append(os.path.splitext(file)[0])
        except Exception as e:
            print(f"Error listing maps: {e}")
        return sorted(maps)

    async def broadcast_map_list(self):
        maps = self.get_saved_maps()
        payload = json.dumps({"type": "map_list", "data": maps})
        await self.send_data_to_clients(payload)

    async def broadcast_waypoint_list(self, websocket, wps):
        payload = json.dumps({"type": "waypoint_list", "data": wps})
        await websocket.send(payload)

    async def broadcast_path_list(self, websocket, paths):
        payload = json.dumps({"type": "path_list", "data": paths})
        await websocket.send(payload)

    # --- Map Data Helpers ---
    def load_map_data(self, map_base):
        json_path = os.path.join(self.map_save_path, f"{map_base}.json")
        default_data = {"waypoints": [], "paths": []}
        if not os.path.exists(json_path):
            return default_data
        
        try:
            with open(json_path, 'r') as f:
                content = json.load(f)
                # Migration: if list, assume it's waypoints
                if isinstance(content, list):
                    return {"waypoints": content, "paths": []}
                # Else assume it's the new dict structure
                return content
        except Exception as e:
            print(f"Error loading map data: {e}")
            return default_data

    def save_map_data(self, map_base, type_key, item):
        json_path = os.path.join(self.map_save_path, f"{map_base}.json")
        data = self.load_map_data(map_base)
        
        # Ensure key exists
        if type_key not in data:
            data[type_key] = []
        
        data[type_key].append(item)
        
        try:
            with open(json_path, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"Saved {type_key} to {json_path}")
        except Exception as e:
            print(f"Error saving map data: {e}")

    def delete_map_item(self, map_base, type_key, item_name):
        json_path = os.path.join(self.map_save_path, f"{map_base}.json")
        data = self.load_map_data(map_base)
        
        if type_key in data:
            data[type_key] = [i for i in data[type_key] if i.get('name') != item_name and i.get('path_name') != item_name]
        
        try:
            with open(json_path, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"Deleted {item_name} from {type_key} in {json_path}")
        except Exception as e:
            print(f"Error deleting map item: {e}")

    def start_websocket_server(self):
        print("Start loop")
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        start_server = websockets.serve(self.websocket_handler, "0.0.0.0", 8888)
        self.loop.run_until_complete(start_server)
        print("Loop is running")
        self.loop.run_forever()

    

    async def send_data_to_clients(self, data):
        try:
            #print("Sending data to clients")  # Ensure this line is printed
            if not self.clients1:
                #print("No clients connected")
                return

            for websocket in self.clients1:
                #print(f"Sending data to client: {websocket}")
                await websocket.send(data)
                #print(f"Data sent to client: {websocket}")
        except Exception as e:
            print(f"Error in send_data_to_clients: {e}")

    def broadcast_message(self, data):
        """Helper to schedule message broadcast on the WebSocket event loop from any thread."""
        if hasattr(self, 'loop') and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.send_data_to_clients(data), self.loop)
        else:
            # Fallback if loop isn't ready or we are on the main thread (less likely here)
            # print("Warning: WebSocket loop not ready for broadcast")
            pass


    def ros2_msg_to_json(self, topic_name, msg):
        # Convert the message to an ordered dictionary
        msg_dict = message_to_ordereddict(msg)

        # Create a dictionary with the topic name and message data
        msg_with_topic = {
            "topic": topic_name,
            "data": msg_dict
        }

        # Convert the dictionary to a JSON string
        return json.dumps(msg_with_topic)

    async def broadcast_process_status(self, name, status):
        payload = json.dumps({
            "type": "process_status",
            "name": name,
            "status": status
        })
        await self.send_data_to_clients(payload)

    def start_launch(self, launch_name, launch_file_path, args=None):
        """Start a launch file using subprocess."""
        if launch_name in self.launch_services:
            self.get_logger().warn(f"Launch '{launch_name}' is already running.")
            return

        if not os.path.isfile(launch_file_path):
            self.get_logger().error(f"Launch file not found: {launch_file_path}")
            return

        try:
            # Use subprocess to start the launch file externally
            # This avoids the 'main thread' requirement of LaunchService
            # start_new_session=True creates a new process group (Python 3.2+)
            cmd = ['ros2', 'launch', launch_file_path]
            if args:
                cmd.extend(args)
                
            process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            
            self.launch_services[launch_name] = process
            self.get_logger().info(f"Started launch file '{launch_name}': {launch_file_path}")
            
            # Broadcast status to all clients
            payload = json.dumps({"type": "process_status", "name": launch_name, "status": "running"})
            self.broadcast_message(payload)
            
        except Exception as e:
            self.get_logger().error(f"Failed to start launch file '{launch_name}': {e}")

    def stop_launch(self, launch_name):
        """Stop a running launch file."""
        if launch_name not in self.launch_services:
            self.get_logger().warn(f"Launch '{launch_name}' is not running.")
            return

        try:
            process = self.launch_services[launch_name]
            # Send SIGINT to the entire process group mimicing Ctrl-C
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            
            # Optionally wait for it to cleanly exit, but don't block too long
            try:
                # We can't block here for too long if we want to be responsive
                # process.wait(timeout=2)
                pass 
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            
            del self.launch_services[launch_name]
            self.get_logger().info(f"Stopped launch file '{launch_name}'.")
            
            # Broadcast status to all clients
            payload = json.dumps({"type": "process_status", "name": launch_name, "status": "stopped"})
            self.broadcast_message(payload)
        except Exception as e:
            self.get_logger().error(f"Failed to stop launch file '{launch_name}': {e}")

    def stop_all_launches(self):
        """Stop all running launch files."""
        for launch_name in list(self.launch_services.keys()):
            self.stop_launch(launch_name)


def main(args=None):
    rclpy.init(args=args)
    clients1 = set()
    node = WebSocketROS2Bridge(clients1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

""" def start_ros2_node(clients):
    rclpy.init()
    node = WebSocketROS2Bridge(clients)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    clients1 = set()  # Use a set to store connected WebSocket clients
    # Start the ROS 2 node in another thread or process
    ros2_thread = threading.Thread(target=start_ros2_node, args=(clients1,))
    ros2_thread.start() """
