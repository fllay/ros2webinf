import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import multiprocessing
from multiprocessing import Manager, Lock
from flask import Flask, send_from_directory, render_template
import asyncio
import websockets
import functools




# Flask application to serve the frontend web interface
app = Flask(__name__)


@app.route('/')
def index():
    """Serve the main UI page."""
    return render_template('index.html')

# Start the web server on all interfaces at port 8000
app.run(host='0.0.0.0', port=8000)



