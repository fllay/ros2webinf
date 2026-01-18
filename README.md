# ROS2 Web Interface

This project provides a web-based interface for monitoring and controlling a ROS2 robot. It uses a WebSocket bridge to communicate between the ROS2 backend and a Three.js-based frontend.

## Features

### Visualization
- **Real-time Map Visualization**: Displays the global map (OccupancyGrid) from ROS2 with proper rendering of free space (white), occupied space (black), and unknown areas (gray).
- **Robot State Tracking**: Visualizes the robot's current pose and laser scan data (Point Cloud).
- **Navigation Status Display**: Real-time feedback showing distance to goal, current position, and navigation success/failure.

### Path Management System
- **Interactive Path Drawing**: Left-click to add points, right-click and select "End" to finish.
- **Path Persistence**: Save drawn paths with custom names; paths are stored in the map's JSON file.
- **Path List**: View and select saved paths for execution.
- **Advanced Visualization**: Paths are rendered as thin directional arrows with orientation indicators at the arrival point.
- **Path Following**: Execute saved paths using the `NavigateThroughPoses` action.

### Map Management
- **Save Map**: Save the current SLAM-generated map to disk.
- **Map List**: View all saved maps with clickable selection.
- **Start/Stop Navigation**: Launch Nav2 stack with a selected map.
- **Delete Map**: Remove unwanted maps from storage.
- **Map Storage**: Maps are saved to `/home/pi/amr_configs/maps` (configurable).

### Waypoint System
- **Add Waypoints**: Click and drag on the map to define waypoint position and heading.
- **Named Waypoints**: Prompt to name each waypoint upon creation.
- **Visual Markers**: Waypoints are displayed as short dark-green arrows with labels directly on the 3D map.
- **Waypoint Storage**: Saved as JSON arrays in `<map_name>.json` within the map directory.
- **Map Association**: Waypoints are automatically filtered and loaded based on the currently selected map.
- **Navigate to Waypoint**: High-precision navigation to any saved waypoint with one click.
- **Deletion**: Easily remove specific waypoints with a confirmation prompt.

### Process Management
- **Upstart/Stop**: Start and stop the minimal launch file.
- **SLAM Nav**: Start and stop SLAM with async navigation.
- **Nav Stack**: Start and stop the Nav2 navigation stack with a selected map.
- **Button Interlocks**: Smart button states prevent conflicting operations.
- **Stability**: Robust WebSocket communication with thread-safe data broadcasting from ROS2 callbacks to connected clients.

### Interactive UI
- **Pan/Zoom**: Intuitive map controls with mouse and keyboard.
- **Camera Rotation**: Rotate the view with dedicated buttons (+90°, -90°, Reset).
- **Materialize CSS**: Clean, modern styling.
- **Status Indicators**: Color-coded buttons and real-time status updates.
- **Dynamic Layout**: Responsive panel layout that maximizes visualization space.

## Architecture

The project consists of three main components:

1.  **`ss.py` (ROS2 WebSocket Bridge)**:
    -   Acts as a ROS2 Node (`websocket_ros2_bridge`).
    -   Subscribes to `/map`, `/scan` (converting to PointCloud2), and TF transforms.
    -   Hosts a WebSocket server on port `8888`.
    -   Broadcasts map, pose, sensor data, and navigation feedback to connected web clients.
    -   Receives navigation commands and process control from the frontend.
    -   Manages ROS2 launch files via subprocess.
    -   Publishes initial pose estimates to `/initialpose`.

2.  **`web.py` (Web Server)**:
    -   A simple Flask application.
    -   Serves the `index.html` and static assets.
    -   Runs on port `8000` by default.

3.  **Frontend (`templates/index.html`)**:
    -   Uses Three.js for 2D/3D visualization.
    -   Connects to the WebSocket bridge to push/pull data.
    -   Provides interactive controls for navigation and map management.

## Prerequisites

-   **ROS2** (e.g., Humble, Iron, Jazzy)
-   **Nav2** (Navigation2 stack)
-   **SLAM Toolbox** (for mapping)
-   **Python 3.8+**
-   **Python Packages**:
    -   `rclpy` (ROS2 Python client library)
    -   `flask`
    -   `websockets`
    -   `numpy`
    -   `scipy`
    -   `nav2_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`

## Installation

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    cd ros2webinf
    ```

2.  **Install Python dependencies:**
    ```bash
    pip install flask websockets numpy scipy
    ```
    *Note: ROS2 dependencies (`rclpy`, `nav2_msgs`, etc.) should be installed via your ROS2 installation.*

3.  **Create required directories on your robot:**
    ```bash
    mkdir -p /home/pi/amr_configs/maps
    ```

4.  **Copy launch files to your robot:**
    - `minimal.py` → `/home/pi/minimal.py`
    - `slam_async_nav.py` → `/home/pi/slam_async_nav.py`
    - `start_nav.py` → Current working directory where `ss.py` runs
    - `nav2_parameter.yaml` → `/home/pi/amr_configs/nav2_parameter.yaml`

## Configuration

Before running, adjust the following settings:

1.  **WebSocket Connection**:
    -   The frontend automatically uses `window.location.hostname` to connect to the WebSocket server.
    -   Ensure `ss.py` is running on the same machine or update the connection logic in `index.html` if needed.

2.  **Map Save Path** (in `ss.py`):
    -   Default: `/home/pi/amr_configs/maps`
    -   Modify `self.map_save_path` in `ss.py` line ~119 if needed.

3.  **Launch File Paths** (in `ss.py`):
    -   `minimal.py`: Line ~505
    -   `slam_async_nav.py`: Line ~510
    -   `start_nav.py`: Line ~543

## Usage

### Starting the System

1.  **Start the WebSocket Bridge:**
    ```bash
    source /opt/ros/<distro>/setup.bash
    cd /path/to/ros2webinf
    python3 ss.py
    ```

2.  **Start the Web Server:**
    ```bash
    python3 web.py
    ```

3.  **Access the Interface:**
    -   Open a web browser and navigate to `http://<robot-ip>:8000`

### Workflow Example

#### Mapping (SLAM)
1. Click **"SLAM Nav"** to start SLAM and navigation.
2. Drive the robot around (use teleop or send navigation goals).
3. Click **"Save Map"** and enter a name (e.g., "office_map").
4. Click **"Stop SLAM"** when mapping is complete.

#### Navigation with Saved Map
1. Select a map from the **"Saved Maps"** list.
2. Click **"Start Nav"** to launch Nav2 with the selected map.
3. Click **"Set Pose"** and draw an arrow on the map to set the robot's initial position.
4. Click **"Nav to Pose"** and draw an arrow to send a navigation goal.
5. Monitor progress in the **"Navigation Status"** field.
6. Click **"Stop Nav"** when done.

## Controls

### Mouse Controls
-   **Pan**: `Ctrl` + `Left Click & Drag`
-   **Zoom**: `Mouse Wheel` scroll
-   **Nav to Pose**: Click and drag to set goal position and orientation
-   **Set Pose**: Click and drag to set initial pose estimate
-   **Draw Path**: Click points on the map to create a path

### Right Panel Buttons
-   **Upstart / Stop**: Start/stop minimal launch file
-   **SLAM Nav / Stop SLAM**: Start/stop SLAM with navigation
-   **Save Map**: Save current map to disk
-   **Delete Map**: Delete selected map and its waypoints
-   **Start Nav / Stop Nav**: Start/stop Nav2 with selected map
-   **Go To WP**: Navigate to the selected waypoint

### Left Panel Menu
-   **Nav to pose**: Enable navigation goal mode
-   **Set Pose**: Enable initial pose estimation mode
-   **Add Waypoint**: Enable waypoint creation mode (click and drag on map)
-   **Draw Path**: Enable path drawing mode
-   **Path follower**: Execute drawn path
-   **Disable All**: Cancel current interaction mode

### Camera Controls
-   **↺ +90°**: Rotate view counterclockwise
-   **Reset**: Reset camera rotation
-   **↻ -90°**: Rotate view clockwise

## API Documentation

### WebSocket API (Client <--> Server)

The server listens on **`ws://0.0.0.0:8888`**.

#### Incoming Messages (Client → Server)

| Type | Name | Data | Action |
|:-----|:-----|:-----|:-------|
| **`action`** | `navtopose` | `{ position, orientation }` | Send NavigateToPose goal |
| **`action`** | `pathfollow` | `[ { position, orientation }, ... ]` | Send goal through poses |
| **`process`** | `upstart` | - | Start minimal.py launch |
| **`process`** | `save_path` | `{ map_name, path: {name, poses} }` | Save path to map's JSON |
| **`process`** | `delete_path` | `{ map_name, path_name }` | Remove path from JSON |
| **`process`** | `save_map` | `"map_name"` | Save SLAM map |
| **`process`** | `start_nav` | `"map_name"` | Start Nav2 with selected map |
| **`ui_state`** | `{ ... }` | - | Synchronize UI selections |
| **`process`** | `save_waypoint` | `{ map_name, waypoint }` | Save a new waypoint |
| **`process`** | `delete_waypoint` | `{ map_name, waypoint_name }` | Remove a waypoint |

#### Outgoing Messages (Server → Client)

| Type | Content | Description |
|:-----|:--------|:------------|
| **`map`** | OccupancyGrid (JSON) | Global map data |
| **`robot_pose_in_map`** | PoseStamped (JSON) | Robot's current pose |
| **`scan_pointcloud`** | PointCloud2 (Base64) | Laser scan as point cloud |
| **`map_list`** | `["map1", "map2", ...]` | List of saved map names |
| **`waypoint_list`** | `[ { name, position, orientation }, ... ]` | List of waypoints for the map |
| **`process_status`** | `{ name: "minimal", status: "running" }` | Process state updates |
| **`nav_feedback`** | `{ distance_remaining, current_pose, ... }` | Real-time navigation feedback |
| **`nav_result`** | `{ success: true/false, error: "..." }` | Navigation completion status |

### ROS 2 Interface

**Node Name:** `websocket_ros2_bridge`

**Subscribed Topics:**
-   `/map` (`nav_msgs/OccupancyGrid`) - Global map
-   `/scan` (`sensor_msgs/LaserScan`) - Laser scan data
-   TF transforms (`map` → `base_link`)

**Published Topics:**
-   `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) - Initial pose estimate
-   `/pose_array` (`geometry_msgs/PoseArray`) - Path visualization
-   `/scan_pointcloud` (`sensor_msgs/PointCloud2`) - Converted laser scan

**Action Clients:**
-   `/navigate_to_pose` (`nav2_msgs/NavigateToPose`) - Single goal navigation
-   `/navigate_through_poses` (`nav2_msgs/NavigateThroughPoses`) - Multi-waypoint navigation
-   `/follow_path` (`nav2_msgs/FollowPath`) - Path following

**Launch Management:**
-   Uses `subprocess` to manage ROS2 launch files
-   Sends `SIGINT` for graceful shutdown

## Map File Format

Saved maps consist of two files:
-   `<map_name>.yaml` - Metadata (resolution, origin, thresholds)
-   `<map_name>.pgm` - Grayscale image (free=white, occupied=black, unknown=gray)

**Map Threshold Configuration:**
Maps are saved with `--free 0.15` to ensure unknown areas (gray, ~0.196) are correctly classified as unknown rather than free space.

## Troubleshooting

**Maps show only black and white (no gray):**
-   Re-save the map using the updated "Save Map" button
-   The new map will have correct thresholds (`free_thresh: 0.15`)

**Buttons stuck in disabled state:**
-   Refresh the web page
-   The 5-second timer will update button states automatically

**Navigation status not updating:**
-   Ensure `ss.py` is restarted with the latest code
-   Check WebSocket connection in browser console

**Map list not updating after save:**
-   Wait 2-3 seconds for the delayed update
-   Refresh the page if needed

## License

[Specify your license here]

## Contributing

[Contribution guidelines]
