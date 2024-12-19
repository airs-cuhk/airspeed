import socket
import os
import threading
import json
import csv

def get_data_keys(json_data, target_keys=None):
    json_data = json_data['execution_data']
    if target_keys is None:
        target_keys = ['motion_execution_data', 'perception_data']
    paths = []
    for key, value in json_data.items():
        if key in target_keys and isinstance(value, dict):
            for sub_key in value.keys():
                paths.append(f"{key}/{sub_key}")
    return paths


def create_csv(csv_path, timestamp, dataname, data):
    file_exists = os.path.isfile(csv_path)
    with open(csv_path, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=['timestamp', dataname])
        if not file_exists or os.path.getsize(csv_path) == 0:
            writer.writeheader()
        writer.writerow({'timestamp': timestamp, dataname: data})


def handle_client_connection(client_socket, save_directory):
    name = ''
    flag = ''
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

            if identifier == "flag":
                flag = data.decode('utf-8', errors='replace')
                # flag = data.decode('utf-8')
                print(f'Flag: {flag}')

            # frame
            if identifier == "frame":
                frame_json = json.loads(data.decode('utf-8'))
                # flag = frame_json["flag"]
                name = frame_json["timestamp"]
                data_keys_paths = get_data_keys(frame_json)
                print("timestamp: ", name)

            # main
            if flag == "start" and identifier == "main":
                main_json = json.loads(data.decode('utf-8'))
                model_name = main_json['robot_motion_sample']['model_data']['robot_hardware_model_name'] + main_json['robot_motion_sample']['model_data']['robot_software_version']
                task_type = main_json['robot_motion_sample']['task_data']['task_type']
                if not os.path.exists(f'{save_directory}/{model_name}/{task_type}'):
                    os.makedirs(f'{save_directory}/{model_name}/{task_type}')

                main_json['robot_motion_sample']['execution_data']['perception_data']['RGB_image'] = f'{save_directory}/{model_name}/{task_type}/rgbs'
                main_json['robot_motion_sample']['execution_data']['perception_data']['depth_image'] = f'{save_directory}/{model_name}/{task_type}/depths'

                for data_key in data_keys_paths:
                    execution_data_path = f'{save_directory}/{model_name}/{task_type}/' + data_key.split('/')[1] + '.csv'
                    main_json['robot_motion_sample']['execution_data'][data_key.split('/')[0]][data_key.split('/')[1]] = execution_data_path

                with open(f'{save_directory}/{model_name}/{task_type}/dataset_structure.json', 'w') as f:
                    json.dump(main_json, f, indent=4)
                print('Saved dataset_structure.json')

            # execution_data
            if (flag == "start" and identifier == "main") or (flag != "start" and identifier == "frame"):
                for data_key in data_keys_paths:
                    execution_data = frame_json['execution_data'][data_key.split('/')[0]][data_key.split('/')[1]]
                    execution_data_path = f'{save_directory}/{model_name}/{task_type}/' + data_key.split('/')[1] + '.csv'
                    create_csv(execution_data_path, name, data_key.split('/')[1], execution_data)
                print('Saved execution data')

            # rgb
            if identifier == "rgb":
                if not os.path.exists(f'{save_directory}/{model_name}/{task_type}/rgbs'):
                    os.makedirs(f'{save_directory}/{model_name}/{task_type}/rgbs')
                with open(f'{save_directory}/{model_name}/{task_type}/rgbs/{identifier}_{name}.jpg', 'wb') as img_file:
                    img_file.write(data)
                print(f'Saved RGB image')

            # depth
            if identifier == "depth":
                if not os.path.exists(f'{save_directory}/{model_name}/{task_type}/depths'):
                    os.makedirs(f'{save_directory}/{model_name}/{task_type}/depths')
                with open(f'{save_directory}/{model_name}/{task_type}/depths/{identifier}_{name}.npy', 'wb') as img_file:
                    img_file.write(data)
                print('Saved depth data')

            # 发送确认消息
            ack_message = json.dumps({"status": "success", "message": f"Received {identifier}"}).encode('utf-8')
            client_socket.sendall(len(ack_message).to_bytes(4, byteorder='big') + ack_message)
            # print(f"Sent acknowledgment for {identifier}")

        except Exception as e:
            print(f"Error handling message: {e}")
            break
    
    client_socket.close()


def start_server(host='0.0.0.0', port=6078, save_directory='./datas'):
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
    start_server()