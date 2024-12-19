from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import numpy as np
import requests
import time
import socket
import threading
from pymycobot import ElephantRobot

# Initialize the robot client
elephant_client = ElephantRobot("10.60.77.187", 5001)
elephant_client.start_client()

# Gripper Thresholds
GRIPPER_CLOSE_THRESHOLD = 48  # mm
GRIPPER_OPEN = 100            # Maximum open value for gripper
GRIPPER_CLOSE = 0             # Fully closed value for gripper
GRIPPER_SPEED = 100           # Speed of gripper movement

def is_within_bounds(coords, radius=630):
    """
    Validate that the coordinates (x, y, z) are within a spherical bound.
    """
    x, y, z = coords[:3]
    distance_squared = x**2 + y**2 + z**2
    return distance_squared <= radius**2

def correct_out_of_bounds(coords, radius=630):
    """
    If coordinates are out of bounds, scale them to fit within the bounds.
    """
    x, y, z = coords[:3]
    distance = np.sqrt(x**2 + y**2 + z**2)
    if distance > radius:
        scale_factor = radius / distance
        x, y, z = x * scale_factor, y * scale_factor, z * scale_factor
        print(f"Coordinates {coords[:3]} are out of bounds. Corrected to: [{x}, {y}, {z}]")
        return [x, y, z] + coords[3:]
    else:
        return coords

def write_coords_checked(coords, speed):
    """
    Write coordinates to the robot after checking if they are within bounds.
    """
    if not is_within_bounds(coords):
        coords = correct_out_of_bounds(coords)
    print(f"Writing coordinates to robot: {coords}")
    elephant_client.write_coords(coords, speed)

def control_gripper(finger_distance):
    """
    Controls the gripper based on the distance between index and thumb fingers.
    """
    if finger_distance <= GRIPPER_CLOSE_THRESHOLD:
        elephant_client.set_gripper_value(GRIPPER_CLOSE, GRIPPER_SPEED)
        print(f"Gripper closed (distance: {finger_distance} mm).")
    else:
        elephant_client.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
        print(f"Gripper opened (distance: {finger_distance} mm).")

class RobotRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        try:
            # Read the content length from headers
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            # Decode and parse JSON data
            data = json.loads(post_data.decode('utf-8'))
            
            # Ensure "message" key exists and validate its contents
            if "message" not in data:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'Invalid JSON: Missing "message" key')
                return
            
            coords = data["message"]
            gripper = data.get('gripper', None)

            if not isinstance(coords, list) or len(coords) != 6:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'Invalid coordinates in "message"')
                return
            
            # Write coordinates to robot arm
            write_coords_checked(coords, speed=5000)
            
            # Handle gripper if data is provided
            if gripper is not None:
                if isinstance(gripper, (int, float)):
                    control_gripper(gripper)
                elif isinstance(gripper, list) and len(gripper) == 1 and isinstance(gripper[0], (int, float)):
                    control_gripper(gripper[0])
            
            # Respond with success
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'Success')
            
        except Exception as e:
            # Handle exceptions and respond with an error
            print(f"Error handling request: {e}")
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b'Internal server error')

    def log_message(self, format, *args):
        pass  # Suppresses all default logging

# Function to stream joint angles over UDP
def stream_joint_angles():
    HTTP_ENDPOINT = "http://10.60.77.198:61740/receive_message"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        try:
            angles = elephant_client.get_angles()
            # Fetch joint angles
            if angles:
                # Create a JSON message with joint angles
                message = json.dumps({"joint_angles": angles})
                response = requests.post(HTTP_ENDPOINT, json=message)
                if response.status_code == 200:
                    print(f"Sent joint angles: {angles}")
                # else:
                    # print(f"Failed to send joint angles. HTTP {response.status_code}: {response.text}")
            else:
                print("Failed to fetch joint angles.")
            # time.sleep(0.01)  # Stream at 100 Hz
        except Exception as e:
            print(f"Error streaming joint angles: {e}")
            # time.sleep(0.001)  # Wait before retrying in case of an error

# Start the joint angle streaming in a separate thread
stream_thread = threading.Thread(target=stream_joint_angles, daemon=True)
stream_thread.start()

# Robot initialization
elephant_client.set_gripper_mode(0)

# Start HTTP server
server_address = ('10.60.77.187', 65432)
httpd = HTTPServer(server_address, RobotRequestHandler)
print(f"HTTP server running on {server_address[0]}:{server_address[1]}...")
try:
    httpd.serve_forever()
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    elephant_client.stop_client()



    httpd.server_close()