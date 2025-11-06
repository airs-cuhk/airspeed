#!/usr/bin/env python3
"""
VR Data Server - HTTPS server to receive VR data and publish to ROS2

This server receives JSON data from a VR device via HTTPS and publishes it
to a ROS2 topic for real-time processing by the teleoperation node.

Dependencies:
- Python 3.7+
- aiohttp (for the web server)
- rclpy (for ROS2 communication)
- std_msgs (for String message type)
"""

import json
import os
import ssl
import sys
import subprocess
import time
from aiohttp import web
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


def get_resource_path(str_relative_path):
    """Get path to resource, works for dev and for PyInstaller"""
    if hasattr(sys, '_MEIPASS'):
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        return os.path.join(sys._MEIPASS, str_relative_path)
    # Development environment
    return os.path.join(os.path.abspath(os.path.dirname(__file__)), str_relative_path)


STATIC_PATH = get_resource_path("../else/static")
global_config = None

# Global message list for client communication
list_client_msg = []

# ROS2节点全局实例
ros_node = None


class VRDataPublisherNode(Node):
    """ROS2节点：发布原始VR数据"""
    
    def __init__(self):
        super().__init__('vr_data_publisher')
        # 创建发布器：发布原始VR数据（JSON字符串）
        self.publisher = self.create_publisher(String, '/vr_raw_data', 10)
        self.get_logger().info('VR Data Publisher Node initialized')
        self.get_logger().info('Publishing raw VR data to topic: /vr_raw_data')
    
    def publish_vr_data(self, vr_data_dict):
        """发布VR数据到ROS2话题
        
        Args:
            vr_data_dict: VR数据字典
        """
        try:
            # 将字典转换为JSON字符串
            json_str = json.dumps(vr_data_dict)
            
            # 创建ROS2消息
            msg = String()
            msg.data = json_str
            
            # 发布消息
            self.publisher.publish(msg)
            self.get_logger().debug(f'Published VR data to ROS2 topic')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing VR data: {e}')


def get_config():
    global global_config
    if global_config is not None:
        return global_config
    str_use_config_file = get_resource_path("../config/config.json")
    with open(str_use_config_file, "r") as file_use_config:
        str_use_data = file_use_config.read()
    global_config = json.loads(str_use_data)
    return global_config


def get_config_port():
    return get_config()["port"]


def __exist_pem(str_pem_dir, str_ip_address: str):
    __IP_Address = "IP_address.txt"
    __CERT_PEM = "cert.pem"
    __KEY_PEM = "key.pem"
    b_all_exist = os.path.exists(str_pem_dir + "/" + __IP_Address) \
                  and os.path.exists(str_pem_dir + "/" + __CERT_PEM) \
                  and os.path.exists(str_pem_dir + "/" + __KEY_PEM) \
                  and os.path.exists(str_pem_dir)
    if not b_all_exist:
        return False
    with open(str_pem_dir + "/" + __IP_Address, "r") as file:
        str_ip = file.read()
    if not str_ip == str_ip_address:
        return False
    return True


def __create_pem(str_pem_dir: str, str_ip_address: str):
    __IP_Address = "IP_address.txt"
    __CERT_PEM = "cert.pem"
    __KEY_PEM = "key.pem"
    # mkdir
    str_IP_Address = str_pem_dir + "/" + __IP_Address
    str_cert_pem = str_pem_dir + "/" + __CERT_PEM
    str_key_pem = str_pem_dir + "/" + __KEY_PEM

    os.makedirs(str_pem_dir, exist_ok=True)
    with open(str_IP_Address, 'w') as file:
        file.write(str_ip_address)
   
    # For simplicity in this dry version, we'll create a basic certificate
    # In production, you would use mkcert like in the original implementation
    try:
        # Create a simple self-signed certificate for testing
        import OpenSSL
        from OpenSSL import crypto
        
        # Create a key pair
        k = crypto.PKey()
        k.generate_key(crypto.TYPE_RSA, 2048)

        # Create a self-signed cert
        cert = crypto.X509()
        cert.get_subject().C = "US"
        cert.get_subject().ST = "State"
        cert.get_subject().L = "City"
        cert.get_subject().O = "Organization"
        cert.get_subject().OU = "Organizational Unit"
        cert.get_subject().CN = str_ip_address
        cert.set_serial_number(1000)
        cert.gmtime_adj_notBefore(0)
        cert.gmtime_adj_notAfter(365*24*60*60)  # 1 year
        cert.set_issuer(cert.get_subject())
        cert.set_pubkey(k)
        cert.sign(k, 'sha256')

        # Write the cert and key to disk
        with open(str_cert_pem, "wt") as f:
            f.write(crypto.dump_certificate(crypto.FILETYPE_PEM, cert).decode('utf-8'))
        with open(str_key_pem, "wt") as f:
            f.write(crypto.dump_privatekey(crypto.FILETYPE_PEM, k).decode('utf-8'))
    except Exception as e:
        # Fallback to basic certificate creation
        print(f"Error creating certificate with OpenSSL: {e}")


def init_pem(str_pem_dir: str, str_ip_address: str):
    if __exist_pem(str_pem_dir, str_ip_address):
        return
    __create_pem(str_pem_dir, str_ip_address)


def create_ssl_context():
    """Create SSL context for HTTPS server"""
    # Create a simple self-signed certificate for testing
    # In production, use proper certificates
    
    # Check if we have existing certificates
    pem_dir = get_resource_path("../else/pem")
    cert_file = os.path.join(pem_dir, "cert.pem")
    key_file = os.path.join(pem_dir, "key.pem")
    
    if os.path.exists(cert_file) and os.path.exists(key_file):
        # Use existing certificates
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert_file, key_file)
        return context
    else:
        # Create a basic SSL context without certificate verification
        # WARNING: This is insecure and should only be used for testing
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        return context


async def on_request_index(request):
    """Serve the main VR HTML page"""
    str_html = "VRDriverPlaneCamera.html"
    str_full_path = os.path.join(STATIC_PATH, str_html)
    
    if not os.path.exists(str_full_path):
        return web.Response(status=404, text="VR HTML page not found")
    
    # Serve the actual VR HTML file
    with open(str_full_path, "r") as f:
        content = f.read()
    return web.Response(content_type="text/html", text=content)


async def on_request_file(request):
    """Serve static files"""
    str_path = request.match_info.get('path', '')
    str_full_path = os.path.join(STATIC_PATH, str_path)
    
    if not os.path.exists(str_full_path):
        return web.Response(status=404, text='File not found')
    
    if os.path.isdir(str_full_path):
        return web.Response(status=403, text='Directory access is not allowed')
    
    # Determine content type based on file extension
    str_content_type = "application/octet-stream"
    if str_path.endswith(".js"):
        str_content_type = "application/javascript"
    elif str_path.endswith(".css"):
        str_content_type = "text/css"
    elif str_path.endswith(".html"):
        str_content_type = "text/html"
    elif str_path.endswith(".json"):
        str_content_type = "application/json"
    elif str_path.endswith(".png"):
        str_content_type = "image/png"
    elif str_path.endswith(".jpg") or str_path.endswith(".jpeg"):
        str_content_type = "image/jpeg"
    elif str_path.endswith(".ico"):
        str_content_type = "image/x-icon"
    
    with open(str_full_path, 'rb') as f:
        content = f.read()
    return web.Response(body=content, content_type=str_content_type)


async def on_post_pose_data(request):
    """Handle incoming VR pose data and publish to ROS2"""
    global ros_node
    
    try:
        # Parse JSON data from request
        dict_param = await request.json()
        
        # 发布到ROS2话题（替代保存到JSON文件）
        if ros_node is not None:
            ros_node.publish_vr_data(dict_param)
        else:
            print("Warning: ROS2 node not initialized, cannot publish VR data")
        
        # Display VR data (optional, for debugging)
        print("==================================================")
        print("Received VR Data:")
        print("==================================================")
        print(json.dumps(dict_param, indent=2))
        print("==================================================")
        
        # Return success response
        return web.Response(text="Data received successfully")
        
    except Exception as e:
        print(f"Error processing VR data: {e}")
        import traceback
        traceback.print_exc()
        return web.Response(status=400, text=f"Error processing data: {e}")


async def on_post_log(request):
    """Handle client log messages"""
    try:
        dict_param = await request.json()
        log_message = dict_param.get("log", "")
        print(f"Client Log: {log_message}")
        return web.Response()
    except Exception as e:
        print(f"Error processing log data: {e}")
        return web.Response(status=400, text=f"Error processing log data: {e}")


async def on_post_config(request):
    """Return server configuration"""
    try:
        config = get_config()
        return web.Response(
            content_type="application/json",
            text=json.dumps(config)
        )
    except Exception as e:
        print(f"Error serving config: {e}")
        return web.Response(status=500, text=f"Error serving config: {e}")


async def on_post_adb_connect(request):
    """Manually trigger ADB connection to VR device"""
    try:
        if connect_vr_device_via_adb():
            return web.Response(text="Successfully connected to VR device via ADB")
        else:
            return web.Response(status=500, text="Failed to connect to VR device via ADB")
    except Exception as e:
        print(f"Error connecting via ADB: {e}")
        return web.Response(status=500, text=f"Error connecting via ADB: {e}")


async def on_post_request_msg(request):
    """Handle message queue communication"""
    global list_client_msg
    try:
        # Copy messages and clear the list
        list_copy_msg = [msg for msg in list_client_msg]
        list_client_msg = []
        
        str_data = json.dumps({"msg": list_copy_msg})
        return web.Response(content_type="application/json", text=str_data)
    except Exception as e:
        print(f"Error handling message request: {e}")
        return web.Response(status=500, text=f"Error handling message request: {e}")


async def on_get_vr_data(request):
    """Get VR data endpoint (deprecated - data now via ROS2 topic)"""
    return web.Response(
        status=410,
        text="This endpoint is deprecated. VR data is now published via ROS2 topic '/vr_raw_data'"
    )


def send_msg(dict_msg):
    """Send a message to clients"""
    global list_client_msg
    if len(list_client_msg) > 20:
        list_client_msg.pop(0)
    list_client_msg.append(dict_msg)


def check_adb_connection():
    """Check if ADB is connected to a device"""
    try:
        result = subprocess.run(["adb", "devices"], capture_output=True, text=True, timeout=5)
        lines = result.stdout.strip().split('\n')
        # Skip the first line (header) and empty lines
        devices = [line.split('\t')[0] for line in lines[1:] if line and '\t' in line]
        return len(devices) > 0, devices
    except Exception as e:
        print(f"Error checking ADB connection: {e}")
        return False, []


def connect_vr_device_via_adb():
    """Connect to VR device using ADB and reverse ports"""
    try:
        # Check if any devices are connected
        connected, devices = check_adb_connection()
        if not connected:
            print("No ADB devices connected")
            return False
            
        print(f"Found ADB devices: {devices}")
        
        # Reverse the port (5000) from device to host
        result = subprocess.run(["adb", "reverse", "tcp:5000", "tcp:5000"], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("Successfully reversed port 5000 from VR device")
            return True
        else:
            print(f"Failed to reverse port: {result.stderr}")
            return False
    except Exception as e:
        print(f"Error connecting to VR device via ADB: {e}")
        return False


def kill_process_on_port(port):
    """Kill any process running on the specified port"""
    try:
        # Try to find and kill the process on Windows
        result = subprocess.run(["netstat", "-ano"], capture_output=True, text=True, encoding='utf-8', errors='ignore')
        if result.stdout is None:
            return False
        lines = result.stdout.strip().split('\n')
        
        for line in lines:
            if f":{port}" in line and "LISTENING" in line:
                parts = line.split()
                if len(parts) >= 5:
                    pid = parts[-1]
                    print(f"Killing process {pid} running on port {port}")
                    subprocess.run(["taskkill", "/F", "/PID", pid], capture_output=True)
                    time.sleep(1)  # Give it a moment to terminate
                    return True
        return False
    except Exception as e:
        print(f"Error killing process on port {port}: {e}")
        return False


def ros_spin_thread(node):
    """在后台线程中运行ROS2 spin"""
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in ROS2 spin thread: {e}")


def start_server(port=5000, host="0.0.0.0"):
    """Start the HTTPS server with ROS2 integration"""
    global ros_node
    
    # 初始化ROS2
    print("Initializing ROS2...")
    try:
        rclpy.init()
        ros_node = VRDataPublisherNode()
        
        # 在后台线程中运行ROS2 spin
        ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
        ros_thread.start()
        print("ROS2 node started successfully")
        print("Publishing VR data to topic: /vr_raw_data")
    except Exception as e:
        print(f"Error initializing ROS2: {e}")
        print("Server will start but ROS2 publishing will not work")
    
    # Kill any existing process on the port
    print(f"Checking for existing processes on port {port}...")
    kill_process_on_port(port)
    
    # Initialize certificates
    str_pem_dir = get_resource_path("../else/pem")
    
    # Get IP address from config - use the config value directly
    str_ip_address = get_config()["ip"]
    print(f"Using IP address from config: {str_ip_address}")
    
    # Only initialize certificates if not using localhost
    if str_ip_address != "127.0.0.1":
        print(f"Initializing certificates for IP: {str_ip_address}")
        init_pem(str_pem_dir, str_ip_address)
    
    # Try to connect to VR device via ADB
    print("Attempting to connect to VR device via ADB...")
    if connect_vr_device_via_adb():
        print("VR device connected via ADB")
    else:
        print("Failed to connect to VR device via ADB. Make sure the device is connected and ADB is properly configured.")
    
    app = web.Application()
    
    # Define routes
    app.router.add_get("/", on_request_index)
    app.router.add_get("/{path:.*}", on_request_file)
    app.router.add_post("/poseData", on_post_pose_data)
    app.router.add_post("/log", on_post_log)
    app.router.add_post("/config", on_post_config)
    app.router.add_post("/adbConnect", on_post_adb_connect)
    app.router.add_post("/msg", on_post_request_msg)
    app.router.add_get("/vrData", on_get_vr_data)
    
    # Create SSL context
    ssl_context = create_ssl_context()
    
    # Print server information
    print(f"\n{'='*60}")
    print(f"VR Data Server starting on https://{str_ip_address}:{port}")
    print(f"{'='*60}")
    print("Press Ctrl+C to stop the server")
    print("\nROS2 Integration:")
    print("- Publishing VR data to: /vr_raw_data (std_msgs/String)")
    print("\nHTTPS Endpoints:")
    print("- GET / - Main VR interface")
    print("- GET /{path} - Static files")
    print("- POST /poseData - Receives VR pose data and publishes to ROS2")
    print("- POST /log - Receives client logs")
    print("- POST /config - Returns server configuration")
    print("- POST /adbConnect - Connect to VR device via ADB")
    print("- POST /msg - Message queue communication")
    print(f"{'='*60}\n")
    
    # Start the server
    try:
        web.run_app(app, host=str_ip_address, port=port, ssl_context=ssl_context)
    finally:
        # 清理ROS2资源
        if ros_node is not None:
            print("\nShutting down ROS2...")
            try:
                ros_node.destroy_node()
                rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down ROS2: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dry VR Data Server")
    parser.add_argument("--port", type=int, default=5000, help="Port to run the server on (default: 5000)")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to (default: 0.0.0.0)")
    
    args = parser.parse_args()
    
    try:
        start_server(port=args.port, host=args.host)
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Server error: {e}")