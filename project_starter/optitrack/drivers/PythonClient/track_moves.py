import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import redis 
import signal
import sys


is_looping = True
sleep_time = 2

def signal_handler(sig, frame):
    is_looping = False
    print('You pressed Ctrl+C! Terminating program')
    sys.exit(0)

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
# redis_anymal = redis.Redis(host='192.168.0.234', port=6379) # BAD
redis_anymal = redis.Redis(host='192.168.0.226', port=6379) # GOOD

# === CONFIG ===
skeleton_name = "0"
move_sequence = ['left', 'backward', 'hop', 'stomp_right', 'stomp_left', 'chacha', 'rotate']
thresholds = {
    'left': 0.15,
    'backward': 0.3,
    'hop': 0.9,
    'stomp_left': 0.5,
    'stomp_right': 0.5,
    'chacha': 0.9,
    'rotate': 0.25,
    'forward': 0.3,
    'right': 0.25
}
marker_ids = {
    'toe_l': 47,
    'toe_r': 51,
    'knee_l': 45,
    'knee_r': 49,
    'wrist_l': 9,
    'wrist_r': 28,
    'chest': 3
}

def get_marker_val(marker_key):
    val = redis_client.get(marker_key)
    if val:
        return list(map(float, val.strip('[]').split(',')))
    else:
        return None

if __name__ == "__main__":
    time.sleep(1)

    # print_configuration(streaming_client)
    # print("\n")
    # print_commands(streaming_client.can_change_bitstream_version())

    print("Starting state machine...\n")

    to_home = None

    while True:
        if to_home is None: # TODO
            home_toe_pos = get_marker_val(skeleton_name+"::47::pos") #toe_l marker position
            home_toe_rot = get_marker_val(skeleton_name+"::47::ori") #toe_l orientation
            move_idx = 0
            rotation_counter = 1
            detected = False
            new_sequence = False
            print("New toe_l home: "+get_marker_val(skeleton_name+"::47::pos"))
            
            
        # Retrieve all necessary marker positions
        toe_l_pos = get_marker_val(skeleton_name+"::47::pos")
        toe_r_pos = get_marker_val(skeleton_name+"::51::pos")
        knee_l_pos = get_marker_val(skeleton_name+"::45::pos")
        knee_r_pos = get_marker_val(skeleton_name+"::49::pos")
        wrist_l_pos = get_marker_val(skeleton_name+"::9::pos")
        wrist_r_pos = get_marker_val(skeleton_name+"::28::pos")
        chest_pos = get_marker_val(skeleton_name+"::3::pos")

        toe_l_rot = get_marker_val(skeleton_name+"::47::ori")

        # Skip if markers are missing
        if any(x is None for x in [toe_l_pos, toe_r_pos, knee_l_pos, knee_r_pos, wrist_l_pos, wrist_r_pos]):
            print("Waiting for all markers...")
            continue

        # detected = False
        delta = 0

        if rotation_counter % 2 == 1:
            if abs(toe_l_pos[1] - home_toe_pos[1]) > thresholds['left']:
                redis_anymal.publish('direction', 1)
                print(f"Detected left move: {toe_l_pos[1] - home_toe_pos[1]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(toe_r_pos[0] - home_toe_pos[0]) > thresholds['backward']:
                redis_anymal.publish('direction', 2)
                print(f"Detected backward move: {toe_r_pos[0] - home_toe_pos[0]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(chest_pos[2] - home_toe_pos[2]) < thresholds['hop']:
                redis_anymal.publish('direction', 3)
                print(f"Detected hop move: {chest_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(knee_r_pos[2] - home_toe_pos[2]) > thresholds['stomp_right']:
                redis_anymal.publish('direction', 4)
                print(f"Detected stomp right move: {knee_r_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(knee_l_pos[2] - home_toe_pos[2]) > thresholds['stomp_left']:
                redis_anymal.publish('direction', 5)
                print(f"Detected stomp left move: {knee_l_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(wrist_l_pos[2] - home_toe_pos[2]) > thresholds['chacha']:
                redis_anymal.publish('direction', 6)
                print(f"Detected chacha move: {wrist_l_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)   

            elif abs(toe_l_rot[2] - home_toe_rot[2]) > thresholds['rotate']:
                redis_anymal.publish('direction', 7)
                print(f"Detected rotate move: {toe_l_rot[2] - home_toe_rot[2]}")
                to_home = None
                time.sleep(sleep_time)
            
            elif abs(toe_l_pos[0] - home_toe_pos[0]) > thresholds['forward']:
                redis_anymal.publish('direction', 8)
                print(f"Detected forward move: {toe_l_pos[0] - home_toe_pos[0]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(toe_r_pos[1] - home_toe_pos[1]) > thresholds['right']:
                redis_anymal.publish('direction', 9)
                print(f"Detected right move: {toe_r_pos[1] - home_toe_pos[1]}")
                to_home = None
                time.sleep(sleep_time)
    
        else:
            if abs(toe_l_pos[0] - home_toe_pos[0]) > thresholds['left']:
                redis_anymal.publish('direction', 1)
                print(f"Detected left move: {toe_l_pos[0] - home_toe_pos[0]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(toe_r_pos[1] - home_toe_pos[1]) > thresholds['backward'] and rotation_counter % 4 != 0:
                redis_anymal.publish('direction', 2)
                print(f"Detected backward move: {toe_r_pos[1] - home_toe_pos[1]}")
                to_home = None
                time.sleep(sleep_time)
            
            elif abs(toe_r_pos[1] - home_toe_pos[1]) > 0.4 and rotation_counter % 4 == 0:
                redis_anymal.publish('direction', 2)
                print(f"Detected backward move: {toe_r_pos[1] - home_toe_pos[1]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(chest_pos[2] - home_toe_pos[2]) < thresholds['hop']:
                redis_anymal.publish('direction', 3)
                print(f"Detected hop move: {chest_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(knee_r_pos[2] - home_toe_pos[2]) > thresholds['stomp_right']:
                redis_anymal.publish('direction', 4)
                print(f"Detected stomp right move: {knee_r_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(knee_l_pos[2] - home_toe_pos[2]) > thresholds['stomp_left']:
                redis_anymal.publish('direction', 5)
                print(f"Detected stomp left move: {knee_l_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(wrist_l_pos[2] - home_toe_pos[2]) > thresholds['chacha']:
                redis_anymal.publish('direction', 6)
                print(f"Detected chacha move: {wrist_l_pos[2] - home_toe_pos[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(toe_l_rot[2] - home_toe_rot[2]) > thresholds['rotate']:
                redis_anymal.publish('direction', 7)
                print(f"Detected rotate move: {toe_l_rot[2] - home_toe_rot[2]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(toe_l_pos[1] - home_toe_pos[1]) > thresholds['forward']:
                redis_anymal.publish('direction', 8)
                print(f"Detected forward move: {toe_l_pos[1] - home_toe_pos[1]}")
                to_home = None
                time.sleep(sleep_time)

            elif abs(toe_r_pos[0] - home_toe_pos[0]) > thresholds['right']:
                redis_anymal.publish('direction', 9)
                print(f"Detected right move: {toe_r_pos[0] - home_toe_pos[0]}")
                to_home = None
                time.sleep(sleep_time)

    print("Exited cleanly.")
