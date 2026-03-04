import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import multiprocessing
from multiprocessing import Manager, Lock
from flask import Flask, send_from_directory, render_template, jsonify
import asyncio
import websockets
import functools
import subprocess

NAV2_NODES = [
    '/bt_navigator',
    '/planner_server',
    '/controller_server',
]




# Flask application
app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/nav2_status')
def nav2_status():
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=5
        )
        running_nodes = result.stdout.strip().split('\n') if result.stdout.strip() else []
        active_nav2 = [n for n in NAV2_NODES if n in running_nodes]
        nav2_running = len(active_nav2) == len(NAV2_NODES)
        return jsonify({
            'nav2_running': nav2_running,
            'active_nodes': active_nav2,
            'expected_nodes': NAV2_NODES
        })
    except subprocess.TimeoutExpired:
        return jsonify({'nav2_running': False, 'error': 'timeout checking nodes'}), 503
    except Exception as e:
        return jsonify({'nav2_running': False, 'error': str(e)}), 500

@app.route('/api/nav2_map')
def nav2_map():
    try:
        result = subprocess.run(
            ['ros2', 'param', 'get', '/map_server', 'yaml_filename'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode != 0 or not result.stdout.strip():
            return jsonify({'map_name': None, 'error': 'map_server not running or param unavailable'}), 503

        # Output looks like: "String value is: /path/to/map.yaml"
        output = result.stdout.strip()
        map_path = output.split(':', 1)[-1].strip()
        map_name = map_path.split('/')[-1]  # Get just the filename
        return jsonify({'map_name': map_name, 'map_path': map_path})
    except subprocess.TimeoutExpired:
        return jsonify({'map_name': None, 'error': 'timeout'}), 503
    except Exception as e:
        return jsonify({'map_name': None, 'error': str(e)}), 500

app.run(host='0.0.0.0', port=8000)



