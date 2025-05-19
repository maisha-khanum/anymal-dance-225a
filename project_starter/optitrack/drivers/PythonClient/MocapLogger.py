# optitrack_csv_logger.py
#
# Copyright © 2018 Naturalpoint
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time
import signal
import csv
import redis

from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

# Redis keys for rigid body data\ nRIGID_BODY_POS_KEY = "sai2::optitrack::rigid_body_pos::"
RIGID_BODY_ORI_KEY = "sai2::optitrack::rigid_body_ori::"

# Control flag for the main loop\ nis_looping = True

def signal_handler(sig, frame):
    global is_looping
    is_looping = False
    print('You pressed Ctrl+C! Terminating program')
    sys.exit(0)

# Connect to Redis
redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

# Callback: receive generic frame data and print arguments
def receive_new_frame(data_dict):
    order_list = [
        "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
        "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged"
    ]
    dump_args = True
    if dump_args:
        out_string = "    "
        for key in data_dict:
            out_string += key + "="
            out_string += str(data_dict[key]) + " "
            out_string += "/"
        print(out_string)

# Callback: receive rigid body updates and store into Redis
def receive_rigid_body_frame(new_id, position, rotation):
    redis_client.set(RIGID_BODY_POS_KEY + str(new_id), '[' + ','.join(map(str, position)) + ']')
    redis_client.set(RIGID_BODY_ORI_KEY + str(new_id), '[' + ','.join(map(str, rotation)) + ']')

# Callback: receive skeleton frame data (if used)
def receive_skeleton_frame(new_id, skeleton):
    for i, rigid_body in enumerate(skeleton.rigid_body_list):
        pos_key = f"{new_id}::{i+1}::pos"
        ori_key = f"{new_id}::{i+1}::ori"
        pos_str = '[' + ','.join(map(str, rigid_body.pos)) + ']'
        ori_str = '[' + ','.join(map(str, rigid_body.rot)) + ']'
        redis_client.set(pos_key, pos_str)
        redis_client.set(ori_key, ori_str)

# Utility: print NatNet configuration
def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print(f"  Client:       {natnet_client.local_ip_address}")
    print(f"  Server:       {natnet_client.server_ip_address}")
    print(f"  Command Port: {natnet_client.command_port}")
    print(f"  Data Port:    {natnet_client.data_port}")
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print(f"  Multicast Group: {natnet_client.multicast_address}")
    else:
        print("  Using Unicast")
    # (Additional configuration printing omitted for brevity)

# Print user commands
def print_commands(can_change_bitstream):
    # (Existing implementation unchanged)
    pass

# Request data descriptions from Motive
def request_data_descriptions(s_client):
    s_client.send_request(
        s_client.command_socket,
        s_client.NAT_REQUEST_MODELDEF,
        "",
        (s_client.server_ip_address, s_client.command_port)
    )

# Test classes helper
def test_classes():
    totals = [0,0,0]
    totals = add_lists(totals, DataDescriptions.test_all())
    totals = add_lists(totals, MoCapData.test_all())
    print(f"[PASS] Count = {totals[0]}")
    print(f"[FAIL] Count = {totals[1]}")
    print(f"[SKIP] Count = {totals[2]}")

# Add two 3-element lists
def add_lists(totals, totals_tmp):
    return [sum(x) for x in zip(totals, totals_tmp)]

# Simple argument parser
def my_parse_args(arg_list, args_dict):
    if len(arg_list) > 1:
        args_dict["serverAddress"] = arg_list[1]
    if len(arg_list) > 2:
        args_dict["clientAddress"] = arg_list[2]
    if len(arg_list) > 3 and arg_list[3][0].upper() == 'U':
        args_dict["use_multicast"] = False
    return args_dict

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # Default network options
    optionsDict = {
        "clientAddress": "172.24.68.64",
        "serverAddress": "172.24.68.67",
        "use_multicast": False
    }
    optionsDict = my_parse_args(sys.argv, optionsDict)

    # Create and configure client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Attach callbacks
    streaming_client.new_frame_listener  = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    streaming_client.skeleton_listener   = receive_skeleton_frame
    streaming_client.set_print_level(0)

    # Start streaming
    if not streaming_client.run():
        print("ERROR: Could not start streaming client.")
        sys.exit(1)
    time.sleep(1)
    if not streaming_client.connected():
        print("ERROR: Could not connect properly. Check Motive streaming.")
        sys.exit(2)

    # --- CSV setup ---
    csv_file = open('rigid_bodies.csv', 'w', newline='')
    csv_writer = csv.writer(csv_file)
    header = ['time_step'] + [f'{t}{i}' for i in ['pos_','ori_'] for t in range(51)]
    # correct header generation
    header = ['time_step']
    for body_id in range(51): header += [f'pos_{body_id}', f'ori_{body_id}']
    csv_writer.writerow(header)
    loop_count = 0
    # ------------------

    # Main loop
    while is_looping:
        time.sleep(1)
        loop_count += 1
        if loop_count % 10 == 0:
            ts = time.time()
            row = [ts]
            for body_id in range(51):
                row.append(redis_client.get(RIGID_BODY_POS_KEY + str(body_id)))
                row.append(redis_client.get(RIGID_BODY_ORI_KEY + str(body_id)))
            csv_writer.writerow(row)
            csv_file.flush()

    # Cleanup
    csv_file.close()
    streaming_client.shutdown()
    print("Exited cleanly.")
