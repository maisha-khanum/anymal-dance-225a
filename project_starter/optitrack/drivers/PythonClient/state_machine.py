import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import redis 
import signal
import sys


is_looping = True
def signal_handler(sig, frame):
    is_looping = False
    print('You pressed Ctrl+C! Terminating program')
    sys.exit(0)

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
# redis_anymal = redis.Redis(host='192.168.0.234', port=6379) # BAD
# redis_anymal = redis.Redis(host='192.168.0.226', port=6379) # GOOD

# === CONFIG ===
skeleton_name = "0"
move_sequence = ['left', 'backward', 'hop', 'stomp_right', 'stomp_left', 'chacha', 'rotate']
thresholds = {
    'left': 0.1,
    'backward': 0.3,
    'hop': 1.0,
    'stomp_left': 0.5,
    'stomp_right': 0.5,
    'chacha': 0.9,
    'rotate': 0.25  # add more values to threshold
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

    home_toe_pos = None
    while True:
        if home_toe_pos is None:
            home_toe_pos = get_marker_val(skeleton_name+"::47::pos") #toe_l marker position
            home_toe_rot = get_marker_val(skeleton_name+"::47::ori") #toe_l orientation
            move_idx = 0
            rotation_counter = 1
            detected = False
            new_sequence = False
            print(get_marker_val(skeleton_name+"::47::pos"))
            
            
            
        move = move_sequence[move_idx]
        print(f"Looking for move: {move}")

        # time.sleep(0.5)  # 2-second pause between checks

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
            if move == 'left' and toe_l_pos is not None:
                delta = toe_l_pos[1] - home_toe_pos[1]
                if abs(delta) > thresholds['left']:
                    detected = True

            elif move == 'backward' and toe_r_pos is not None:
                delta = toe_r_pos[0] - home_toe_pos[0]
                if abs(delta) > thresholds['backward']:
                    detected = True

            elif move == 'hop' and chest_pos is not None:
                delta = chest_pos[2] - home_toe_pos[2]
                if abs(delta) < thresholds['hop']:
                    detected = True
                    hopInProgress = False
                    hopComplete = False

            elif move == 'stomp_right' and knee_r_pos is not None:
                delta = knee_r_pos[2] - home_toe_pos[2]
                if abs(delta) > 0.65:
                    hopInProgress = True
                if hopInProgress and abs(delta) < 0.43:
                    hopComplete = True
                if hopComplete and abs(delta) > thresholds['stomp_right']:
                    detected = True

            elif move == 'stomp_left' and knee_l_pos is not None:
                delta = knee_l_pos[2] - home_toe_pos[2]
                if abs(delta) > thresholds['stomp_left']:
                        detected = True

            elif move == 'chacha' and wrist_l_pos is not None and wrist_r_pos is not None:
                delta = wrist_l_pos[2] - home_toe_pos[2]
                if abs(delta) > thresholds['chacha']:
                    detected = True

            elif move == 'rotate' and chest_pos is not None: # TODO
                delta = toe_l_rot[2] - home_toe_rot[2]
                if abs(delta) > thresholds['rotate']:
                    detected = True
                    if not new_sequence:
                        # redis_anymal.publish('direction', move_idx + 1)
                        new_sequence = True
    
        else:
            if move == 'left' and toe_l_pos is not None:
                delta = toe_l_pos[0] - home_toe_pos[0]
                if abs(delta) > thresholds['left']:
                    detected = True

            elif move == 'backward' and toe_r_pos is not None:
                delta = toe_r_pos[1] - home_toe_pos[1]
                if abs(delta) > thresholds['backward'] and rotation_counter % 4 != 0:
                    detected = True
                elif abs(delta) > 0.4 and rotation_counter % 4 == 0:
                    detected = True

            elif move == 'hop' and chest_pos is not None:
                delta = chest_pos[2] - home_toe_pos[2]
                if abs(delta) < thresholds['hop']:
                    detected = True
                    hopInProgress = False
                    hopComplete = False

            elif move == 'stomp_right' and knee_r_pos is not None:
                delta = knee_r_pos[2] - home_toe_pos[2]
                if abs(delta) > 0.65:
                    hopInProgress = True
                if hopInProgress and abs(delta) < 0.43:
                    hopComplete = True
                if hopComplete and abs(delta) > thresholds['stomp_right']:
                    detected = True

            elif move == 'stomp_left' and knee_l_pos is not None:
                delta = knee_l_pos[2] - home_toe_pos[2]
                if abs(delta) > thresholds['stomp_left']:
                        detected = True

            elif move == 'chacha' and wrist_l_pos is not None and wrist_r_pos is not None:
                delta = wrist_l_pos[2] - home_toe_pos[2]
                if abs(delta) > thresholds['chacha']:
                    detected = True

            elif move == 'rotate' and chest_pos is not None:
                delta = toe_l_rot[2] - home_toe_rot[2]
                print(f"Delta for rotation: {delta:.4f}")
                if abs(delta) > thresholds['rotate']:
                    detected = True
                    if not new_sequence:
                        # redis_anymal.publish('direction', move_idx + 1)
                        new_sequence = True

        if detected:
            print(f"Current rotation: {rotation_counter}, Detected move: {move.upper()} (Δ={delta:.4f})")
            if move == 'rotate' and new_sequence:
                delta = toe_l_rot[2] - home_toe_rot[2]

                if abs(delta) > 0.6 and (rotation_counter - 3) % 4 != 0: #think about this rotation counter thing
                    rotation_counter += 1
                    home_toe_pos = get_marker_val(skeleton_name+"::47::pos") #toe_l marker position
                    home_toe_rot = get_marker_val(skeleton_name+"::47::ori") #toe_l orientation
        
                    new_sequence = False

                    move_idx = 0
                    detected = False
                elif abs(delta) > 0.515 and (rotation_counter - 3) % 4 == 0: #need 0.51 for full_dance.csv
                    rotation_counter += 1
                    home_toe_pos = get_marker_val(skeleton_name+"::47::pos") #toe_l marker position
                    home_toe_rot = get_marker_val(skeleton_name+"::47::ori") #toe_l orientation
        
                    new_sequence = False

                    move_idx = 0
                    detected = False

            else:
                # redis_anymal.publish('direction', move_idx + 1)

                move_idx += 1
                detected = False
        else:
            print(f"[ ] No significant movement for {move}. (Δ={delta:.4f})")


    print("Exited cleanly.")
