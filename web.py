import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import multiprocessing
from multiprocessing import Manager, Lock
from flask import Flask, send_from_directory, render_template
import asyncio
import websockets
import functools




# Flask application
app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')

app.run(host='0.0.0.0', port=8000)



