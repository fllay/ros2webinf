# ROS2 Web Interface

This project provides a web-based interface for monitoring and controlling a ROS2 robot. It uses a WebSocket bridge to communicate between the ROS2 backend and a Three.js-based frontend.

## Features

- **Real-time Map Visualization**: Displays the global map (OccupancyGrid) from ROS2.
- **Robot State Tracking**: Visualizes the robot's current pose and laser scan data (Point Cloud).
- **Navigation Controls**:
  - **Nav to Pose**: Send navigation goals by clicking and dragging on the map.
  - **Path Drawing**: Interactively draw a path for the robot to follow.
  - **Path Following**: Execute the drawn path using ROS2 navigation actions.
- **Process Management**: capability to start/stop specific ROS2 launch files (currently configured for a specific `minimal.py` launch).
- **Interactive UI**:
  - Pan/Zoom map controls.
  - Camera rotation.
  - Materialize CSS styling.

## Architecture

The project consists of two main Python scripts:

1.  **`ss.py` (ROS2 WebSocket Bridge)**:
    -   Acting as a ROS2 Node (`websocket_ros2_bridge`).
    -   Subscribes to `/map`, `/scan` (converting to PointCloud2), and TF transforms.
    -   Hosts a WebSocket server on port `8888`.
    -   Broadcasts map, pose, and sensor data to connected web clients.
    -   Receives navigation commands from the frontend and sends them to ROS2 Action Servers (`/navigate_to_pose`, `/follow_path`, `navigate_through_poses`).

2.  **`web.py` (Web Server)**:
    -   A simple Flask application.
    -   Serves the `index.html` and static assets.
    -   Runs on port `8000` by default.

3.  **Frontend (`templates/index.html`)**:
    -   Uses Three.js for 2D/3D visualization.
    -   Connects to the WebSocket bridge to push/pull data.

## Prerequisites

-   **ROS2** (e.g., Humble, Foxy, Iron)
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
    pip install -r requirements.txt
    ```
    *Note: You also need the ROS2 python dependencies installed via your ROS2 installation.*

## Configuration

Before running, you may need to adjust the following settings:

1.  **WebSocket Connection IP**:
    -   Open `templates/index.html`.
    -   Locate line 205: `const ws = new WebSocket("ws://192.168.68.36:8888");`.
    -   Change `192.168.68.36` to the IP address of the machine running `ss.py` (or `localhost` if running locally).

2.  **Launch Scripts**:
    -   The script `ss.py` currently has a hardcoded path for a launch file: `/home/pi/minimal.py`.
    -   If you intend to use the "Upstart" feature, modify `ss.py` (line 469) to point to your desired launch file.

## Usage

1.  **Start the WebSocket Bridge:**
    Make sure your ROS2 environment is sourced.
    ```bash
    source /opt/ros/<distro>/setup.bash
    python3 ss.py
    ```

2.  **Start the Web Server:**
    Open a new terminal.
    ```bash
    python3 web.py
    ```

3.  **Access the Interface:**
    -   Open a web browser and navigate to `http://localhost:8000` (or the IP of the machine running `web.py`).

## Controls

-   **Pan**: `Ctrl` + `Left Click & Drag` (or just drag if supported).
-   **Zoom**: `Mouse Wheel` scroll.
-   **Menu**: Right-click to access context options (End, Enable Zoom, Pan, Disable).
-   **Left Sidebar**:
    -   `Nav to pose`: Click to enable. On the map, click and drag to set position and orientation.
    -   `Draw Path`: Click to enable. Click points on the map to draw a path.
    -   `Path follower`: Sends the drawn path to the robot.
    -   `Disable All`: Cancel current tool.
