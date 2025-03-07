import socket
import json
import queue
import numpy as np
import requests
import time
import threading
import re
import yaml
from pymycobot import ElephantRobot

# -------------------------------
# Load Configuration
# -------------------------------
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

GRIPPER_CLOSE_THRESHOLD = config.get("GRIPPER_CLOSE_THRESHOLD", 48)  # mm
FIXED_SPEED = config.get("FIXED_SPEED", 5000)

robot_ip = config.get("robot_ip", "10.60.77.187")
robot_port = config.get("robot_port", 5001)

service_ip = config.get("service_ip", "10.60.2.237")
service_port = config.get("service_port", 12588)

# -------------------------------
# Robot Client Initialization
# -------------------------------
elephant_client = ElephantRobot(robot_ip, robot_port)
elephant_client.start_client()
gripper_state_deque = queue.Queue(maxsize=1)
elephant_client.set_gripper_mode(0)
print('Testing Gripper')
elephant_client.set_gripper_value(0, 100)
time.sleep(2)
elephant_client.set_gripper_value(100, 100)
time.sleep(1)

# -------------------------------
# Constants and Helper Functions
# -------------------------------

GRIPPER_OPEN = 100  # Maximum open value for gripper
GRIPPER_CLOSE = 0   # Fully closed value for gripper
GRIPPER_SPEED = 100  # Speed of gripper movement

# Frequency Control for UDP streaming (in seconds)
command_interval = 0.05

def is_within_bounds(coords, radius=630):
    """Validate that the coordinates (x, y, z) are within a spherical bound."""
    x, y, z = coords[:3]
    return (x**2 + y**2 + z**2) <= radius**2

def correct_out_of_bounds(coords, radius=630):
    """If coordinates are out of bounds, scale them to fit within the bounds."""
    x, y, z = coords[:3]
    distance = np.sqrt(x**2 + y**2 + z**2)
    if distance > radius:
        scale_factor = radius / distance
        x, y, z = x * scale_factor, y * scale_factor, z * scale_factor
        print(f"Coordinates {coords[:3]} are out of bounds. Corrected to: [{x}, {y}, {z}]")
        return [x, y, z] + coords[3:]
    return coords

def write_coords_checked(coords, speed):
    """Write coordinates to the robot after checking bounds."""
    if not is_within_bounds(coords):
        coords = correct_out_of_bounds(coords)
    print(f"Writing coordinates to robot: {coords} at speed: {speed}")
    elephant_client.write_coords(coords, speed)

def control_gripper(finger_distance):
    """Control the gripper based on the provided distance."""
    if finger_distance <= GRIPPER_CLOSE_THRESHOLD:
        elephant_client.set_gripper_value(GRIPPER_CLOSE, GRIPPER_SPEED)
        gripper_state_deque.put(0)
        print(f"Gripper closed (distance: {finger_distance} mm).")
    else:
        elephant_client.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
        gripper_state_deque.put(1)
        print(f"Gripper opened (distance: {finger_distance} mm).")

def parse_incoming_data(data):
    """
    Parses the new data format:
      [[pos1, pos2, pos3, pos4, pos5, pos6], gripper]
    and returns a tuple:
      ( [pos1, pos2, pos3, pos4, pos5, pos6], gripper )
    """
    if isinstance(data, list) and len(data) == 2:
        position = data[0]
        gripper = data[1]
        if isinstance(position, list) and len(position) == 6 and all(isinstance(x, (int, float)) for x in position):
            if isinstance(gripper, (int, float)):
                return position, gripper
            elif isinstance(gripper, list) and len(gripper) == 1 and isinstance(gripper[0], (int, float)):
                return position, gripper[0]
    print("Invalid data format:", data)
    return None, None

# -------------------------------
# TCP Client Implementation
# -------------------------------

def process_message(obj):
    """Process the JSON message by parsing and controlling the robot."""
    print("Received JSON object:", obj)
    position, gripper = parse_incoming_data(obj)
    if position is None:
        return

    print("Parsed data: Position:", position, "Gripper:", gripper)
    if not (isinstance(position, list) and len(position) == 6):
        print("Invalid position format")
        return

    write_coords_checked(position, speed=FIXED_SPEED)
    
    if gripper is not None:
        if isinstance(gripper, (int, float)):
            control_gripper(gripper)
        elif isinstance(gripper, list) and len(gripper) == 1 and isinstance(gripper[0], (int, float)):
            control_gripper(gripper[0])

def tcp_client_receiver():
    """
    Connects to the sender's TCP server and continuously receives data.
    Expected data format (JSON) with newline delimiter:
      [[pos1, pos2, pos3, pos4, pos5, pos6], gripper]\n
    This implementation uses a buffer to handle complete messages and spawns a thread
    to process each message so that the receiving loop is not blocked.
    """
    SERVER_IP = service_ip   # Using service_ip from config.yaml
    SERVER_PORT = service_port  # Using service_port from config.yaml

    buffer = ""
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print(f"Connected to server {SERVER_IP}:{SERVER_PORT}")
        
        client_socket.settimeout(2.0)
        
        while True:
            try:
                data = client_socket.recv(1024)
            except socket.timeout:
                data = b""
            
            if data:
                buffer += data.decode('utf-8')
            
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError as e:
                    print("JSON decode error:", e)
                    continue
                
                threading.Thread(target=process_message, args=(obj,)).start()
                
    except ConnectionRefusedError:
        print("Unable to connect to server. Please check if the server is running.")
    except ConnectionResetError:
        print("Connection reset by server.")
    finally:
        client_socket.close()

def stream_joint_angles():
    HTTP_ENDPOINT = "http://10.60.2.23:61740/receive_message"
    gripper_state = 1
    while True:
        try:
            angles = elephant_client.get_angles()
            gripper_state_now = gripper_state_deque.get()
            if gripper_state_now is not None:
                gripper_state = gripper_state_now
            if angles:
                message = json.dumps({"joint_angles": angles, "gripper": gripper_state})
                response = requests.post(HTTP_ENDPOINT, json=message)
                if response.status_code == 200:
                    print(f"Sent joint angles: {angles}")
            else:
                print("Failed to fetch joint angles.")
            time.sleep(command_interval)
        except Exception as e:
            print(f"Error streaming joint angles: {e}")
            time.sleep(command_interval)

# -------------------------------
# Start Threads and Main Loop
# -------------------------------

# Start the joint angles streaming thread
stream_thread = threading.Thread(target=stream_joint_angles, daemon=True)
stream_thread.start()

# Robot initialization
elephant_client.write_angles([0, -145.751, 121.44, -66.206, -89.872, 3.814], 2500)
elephant_client.set_gripper_mode(0)

# Start the TCP client receiver thread
tcp_client_thread = threading.Thread(target=tcp_client_receiver, daemon=True)
tcp_client_thread.start()

# Keep the main thread active.
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    elephant_client.stop_client()