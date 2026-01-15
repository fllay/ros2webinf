import rclpy
import os
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
        self.map_save_path = "/home/pi/amr_configs/maps"
        #self.websocket_server = None
        # Start the WebSocket server in a separate thread
        self.websocket_thread = threading.Thread(target=self.start_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        

       

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
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

        


        #self.publisher = self.create_publisher(PointCloud2, '/scan_pointcloud', 10)
        #self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose_in_map', 10)
    
    def send_goal_pose(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.navpose_action_client.wait_for_server()

        self._send_goal_future = self.navpose_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        

    def map_callback(self, msg):
        # Prepare map data to be sent through WebSocket
        map_data_json = self.ros2_msg_to_json('map',msg)
        asyncio.run(self.send_data_to_clients(map_data_json))

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud2
        cloud_msg = self.laserscan_to_pointcloud2(scan_msg)
        current_time = rclpy.time.Time()

        # Convert rclpy.time.Time to builtin_interfaces.msg.Time
        time_msg = BuiltinTime(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])

        cloud_msg.header.stamp = time_msg #scan_msg.header.stamp  # Ensure correct timestamp
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",  # target frame
                cloud_msg.header.frame_id,  # source frame (laser frame)
                rclpy.time.Time.from_msg(scan_msg.header.stamp),  # time at which the transform is needed
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
    
                asyncio.run(self.send_data_to_clients(json.dumps(dictionary_tc)))

            
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
            asyncio.run(self.send_data_to_clients(pose_in_map_json))
       
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
        
  
    async def websocket_handler(self, websocket, path):
        self.clients1.add(websocket)
        
        # Send current map list to the new client
        maps = self.get_saved_maps()
        await websocket.send(json.dumps({"type": "map_list", "data": maps}))

        try:
            async for message in websocket:
                # Process incoming messages here
                #print(f"Received message: {message}")
                json_dada = json.loads(message)
                if(json_dada['type'] == "action"):
                    if(json_dada['name'] == "navtopose"):
                        print(json_dada['data'])
                        pp = self.convert_json_pose_to_poasestamp(json_dada['data'])
                        print(pp)
                        self.send_goal_pose(pp)
                    if(json_dada['name'] == "set_pose"):
                        self.publish_initial_pose(json_dada['data'])
                    if(json_dada['name'] == "pathfollow"):
                        print("Get follow path")

                        #print(json_dada['data'])
                        g_path = self.convert_json_pose_array_to_path(json_dada['data'])
                        poses = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in g_path.poses])

                        # x = poses[:, 0]
                        # y = poses[:, 1]

                        # # Remove duplicate points and sort by x value
                        # unique_indices = np.unique(x, return_index=True)
                        # x_unique = x[unique_indices[1]]
                        # y_unique = y[unique_indices[1]]
                        
                        # # Sort by x values
                        # sorted_indices = np.argsort(x_unique)
                        # x_sorted = x_unique[sorted_indices]
                        # y_sorted = y_unique[sorted_indices]

                        # # Create a UnivariateSpline interpolation function
                        # spline = UnivariateSpline(x_sorted, y_sorted, s=1)  # s is the smoothing factor

                        # # Generate new x values
                        # x_new = np.linspace(min(x_sorted), max(x_sorted), 100)
                        # y_new = spline(x_new)

                        # # Create a new Path message for the smoothed path
                        # smoothed_path = Path()
                        # smoothed_path.header = g_path.header
                        # for xi, yi in zip(x_new, y_new):
                        #     pose = PoseStamped()
                        #     pose.header = g_path.header
                        #     pose.pose.position.x = xi
                        #     pose.pose.position.y = yi
                        #     pose.pose.orientation.w = 1.0
                        #     smoothed_path.poses.append(pose)

                        
                                # Create a goal message
                        # goal_msg = FollowPath.Goal()
                        # goal_msg.path = self.create_path()
                        # goal_msg.controller_id = ''  # Use default controller
                        # goal_msg.goal_checker_id = ''  # Use default goal checker
                        # print(goal_msg)
                        #self.smooth_path_publisher.publish(smoothed_path)

                        #goal_msg1 = FollowPath.Goal()
                        #goal_msg1.path = g_path
                        #goal_msg1.controller_id = ''  # Use default controller
                        #goal_msg1.goal_checker_id = ''  # Use default goal checker
                        # print(goal_msg1)
                        #self.send_goal_path(goal_msg1)
                        poses_p = self.convert_json_pose_array_to_poses(json_dada['data'])
                        parray = self.convert_json_pose_array_to_pose_array(json_dada['data'])
                        print(parray)
                        self.pose_array_publisher.publish(parray)
                        
                        goal_poses = NavigateThroughPoses.Goal()
                        goal_poses.poses = poses_p
                        print(goal_poses)
                        self.send_goal_through_poses(goal_poses)
                
                    
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
                        map_name = json_dada.get('data')
                        if map_name:
                            # Construct full path. Assume map name might not have .yaml extension if passed from UI list
                            if not map_name.endswith('.yaml'):
                                map_name += '.yaml'
                            
                            map_path = os.path.join(self.map_save_path, map_name)
                            launch_file_nav = os.path.join(os.getcwd(), 'start_nav.py')
                            
                            # key=value arguments for ros2 launch
                            map_arg = f"map:={map_path}"
                            self.start_launch("nav_stack", launch_file_nav, args=[map_arg])
                    if(json_dada['name'] == "stop_nav"):
                        self.stop_launch('nav_stack')

                
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
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            
            del self.launch_services[launch_name]
            self.get_logger().info(f"Stopped launch file '{launch_name}'.")
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
