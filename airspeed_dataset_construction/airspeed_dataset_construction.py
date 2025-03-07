import socket
import os
import threading
import json
import csv
import argparse

def create_csv(csv_path, timestamp, dataname, data):
    file_exists = os.path.isfile(csv_path)
    with open(csv_path, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=['timestamp', dataname])
        if not file_exists or os.path.getsize(csv_path) == 0:
            writer.writeheader()
        writer.writerow({'timestamp': timestamp, dataname: data})


def handle_client_connection(client_socket, save_directory):
    name = ''
    while True:
        try:
            # identifier length
            identifier_length_bytes = client_socket.recv(4)
            if not identifier_length_bytes or len(identifier_length_bytes) < 4:
                break
            identifier_length = int.from_bytes(identifier_length_bytes, byteorder='big')
            
            # identifier
            identifier = client_socket.recv(identifier_length).decode('utf-8')
            if not identifier:
                break
            
            # size bytes
            size_bytes = client_socket.recv(4)
            if not size_bytes or len(size_bytes) < 4:
                break
            size = int.from_bytes(size_bytes, byteorder='big')

            # data
            data = b''
            while len(data) < size:
                packet = client_socket.recv(size - len(data))
                if not packet:
                    break
                data += packet
            
            if len(data) != size:
                print("Message truncated or incomplete.")
                continue

            # settings
            if identifier == "settings":
                global motion_execution_data
                global perception_data

                settings = json.loads(data.decode('utf-8', errors='replace'))
                save_directory = os.path.dirname(settings['robot_motion_sample']['execution_data']['motion_execution_data']['joint_motion_data'])
                motion_execution_data = dict(filter(lambda item: item[1], settings['robot_motion_sample']['execution_data']['motion_execution_data'].items()))
                perception_data = dict(filter(lambda item: item[1], settings['robot_motion_sample']['execution_data']['perception_data'].items()))

                if not os.path.exists(save_directory):
                    os.makedirs(save_directory)
                with open(f'{save_directory}/dataset_structure.json', 'w') as f:
                    json.dump(settings, f, indent=4)
                print('Saved dataset_structure.json')

            # frame
            if identifier == "frame":
                frame_json = json.loads(data.decode('utf-8'))
                name = frame_json["timestamp"]
                print("timestamp: ", name)

                for key, value in motion_execution_data.items():
                    execution_data = frame_json['execution_data']['motion_execution_data'][key]
                    create_csv(value, name, key, execution_data)
                print('Saved execution data')

            # rgb
            if identifier == "RGB_image":
                rgb_images_path = perception_data[identifier]
                if not os.path.exists(rgb_images_path):
                    os.makedirs(rgb_images_path)
                with open(f'{rgb_images_path}/{name}.jpg', 'wb') as img_file:
                    img_file.write(data)
                print(f'Saved RGB image')

            # depth
            if identifier == "depth_image":
                depth_image_path = perception_data[identifier]
                if not os.path.exists(depth_image_path):
                    os.makedirs(depth_image_path)
                with open(f'{depth_image_path}/{name}.npy', 'wb') as img_file:
                    img_file.write(data)
                print('Saved depth data')

            # send ack_message
            ack_message = json.dumps({"status": "success", "message": f"Received {identifier}"}).encode('utf-8')
            client_socket.sendall(len(ack_message).to_bytes(4, byteorder='big') + ack_message)
            # print(f"Sent acknowledgment for {identifier}")

        except Exception as e:
            print(f"Error handling message: {e}")
            break
    
    client_socket.close()


def start_server(host, port, save_directory):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(5)
    print(f"Listening on {host}:{port}")

    try:
        while True:
            client_socket, addr = server.accept()
            print()
            print(f"Accepted connection from {addr}")
            client_handler = threading.Thread(target=handle_client_connection, args=(client_socket, save_directory))
            client_handler.start()
    except KeyboardInterrupt:
        print("Server shutting down.")
    finally:
        server.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start a TCP server.")
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=6078, help='Port to listen on (default: 6078)')
    parser.add_argument('--save_directory', default='./datas', help='Directory to save data (default: ./datas)')

    args = parser.parse_args()

    start_server(args.host, args.port, args.save_directory)