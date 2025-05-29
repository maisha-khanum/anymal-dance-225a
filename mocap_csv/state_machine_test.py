import sys
import time
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import redis

redis_anymal = redis.Redis(host='192.168.0.234', port=6379)

# === ARGUMENT PARSING ===
if len(sys.argv) < 2:
    print("Usage: python combined_viewer.py <csv_filename> [--show_arrows true|false]")
    sys.exit(1)

csv_name = sys.argv[1]

# === LOAD CSV ===
data = pd.read_csv(csv_name)
timestamps = sorted(data['timestamp'].unique())

# === SETUP FOR MOTION DETECTION ===
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
move_idx = 0
curr_home = None
rotation_counter = 1
prior_move = None

def get_forward_vector_from_quat(q):
    r = R.from_quat(q)
    return r.apply([0, 0, 0.05])

for i, ts in enumerate(timestamps[380:]):  # Adjust slice for skipping idle frames

    frame_data = data[data['timestamp'] == ts].sort_values(by='rigid_body_id')
    pos = frame_data[['pos_x', 'pos_y', 'pos_z']].to_numpy()
    rots = frame_data[['rot_x', 'rot_y', 'rot_z', 'rot_w']].to_numpy()
    ids = frame_data['rigid_body_id'].to_numpy()
    current_marker_ids = ids

    marker_dict = {int(rid): p for rid, p in zip(ids, pos)}

    if marker_ids['toe_l'] in marker_dict and curr_home is None:
        curr_home = marker_dict[marker_ids['toe_l']]
        curr_home_rot = rots[47].copy()
        print(f"Set home position: {curr_home}")

    # Motion Detection Logic
    detected = False
    if curr_home is not None and move_idx < len(move_sequence):
        move = move_sequence[move_idx]
        print(f"\nLooking for move: {move}")

        toe_l = marker_dict.get(marker_ids['toe_l'])
        toe_r = marker_dict.get(marker_ids['toe_r'])
        knee_l = marker_dict.get(marker_ids['knee_l'])
        knee_r = marker_dict.get(marker_ids['knee_r'])
        wrist_l = marker_dict.get(marker_ids['wrist_l'])
        wrist_r = marker_dict.get(marker_ids['wrist_r'])
        chest = marker_dict.get(marker_ids['chest'])
    
        delta = 0

        if rotation_counter % 2 == 1:
            if move == 'left' and toe_l is not None:
                delta = toe_l[1] - curr_home[1]
                if abs(delta) > thresholds['left']:
                    detected = True

            elif move == 'backward' and toe_r is not None:
                delta = toe_r[0] - curr_home[0]
                if abs(delta) > thresholds['backward']:
                    detected = True

            elif move == 'hop' and chest is not None:
                delta = chest[2] - curr_home[2]
                if abs(delta) < thresholds['hop']:
                    detected = True
                    hopInProgress = False
                    hopComplete = False

            elif move == 'stomp_right' and knee_r is not None:
                delta = knee_r[2] - curr_home[2]
                if abs(delta) > 0.65:
                    hopInProgress = True
                if hopInProgress and abs(delta) < 0.43:
                    hopComplete = True
                if hopComplete and abs(delta) > thresholds['stomp_right']:
                    detected = True

            elif move == 'stomp_left' and knee_l is not None:
                delta = knee_l[2] - curr_home[2]
                if abs(delta) > thresholds['stomp_left']:
                        detected = True

            elif move == 'chacha' and wrist_l is not None and wrist_r is not None:
                delta = wrist_l[2] - curr_home[2]
                if abs(delta) > thresholds['chacha']:
                    detected = True

            elif move == 'rotate' and toe_l is not None:
                delta = rots[47][2] - curr_home_rot[2]
                if abs(delta) > thresholds['rotate']:
                    detected = True
                    new_sequence = True

                    redis_anymal.publish(f"{move_idx + 1}")
        else:
            if move == 'left' and toe_l is not None:
                delta = toe_l[0] - curr_home[0]
                if abs(delta) > thresholds['left']:
                    detected = True

            elif move == 'backward' and toe_r is not None:
                delta = toe_r[1] - curr_home[1]
                if abs(delta) > thresholds['backward'] and rotation_counter % 4 != 0:
                    detected = True
                elif abs(delta) > 0.4 and rotation_counter % 4 == 0:
                    detected = True

            elif move == 'hop' and chest is not None:
                delta = chest[2] - curr_home[2]
                if abs(delta) < thresholds['hop']:
                    detected = True
                    hopInProgress = False
                    hopComplete = False

            elif move == 'stomp_right' and knee_r is not None:
                delta = knee_r[2] - curr_home[2]
                if abs(delta) > 0.65:
                    hopInProgress = True
                if hopInProgress and abs(delta) < 0.43:
                    hopComplete = True
                if hopComplete and abs(delta) > thresholds['stomp_right']:
                    detected = True

            elif move == 'stomp_left' and knee_l is not None:
                delta = knee_l[2] - curr_home[2]
                if abs(delta) > thresholds['stomp_left']:
                        detected = True

            elif move == 'chacha' and wrist_l is not None and wrist_r is not None:
                delta = wrist_l[2] - curr_home[2]
                if abs(delta) > thresholds['chacha']:
                    detected = True

            elif move == 'rotate' and toe_l is not None:
                delta = rots[47][2] - curr_home_rot[2]
                if abs(delta) > thresholds['rotate']:
                    detected = True
                    new_sequence = True

                    redis_anymal.publish(f"{move_idx + 1}")


        if detected:
            print(f"✔ Detected move: {move.upper()} (Δ={delta:.4f})")
            if move == 'rotate' and new_sequence:
                delta = rots[47][2] - curr_home_rot[2]
                if abs(delta) > 0.6 and (rotation_counter - 3) % 4 != 0: #think about this rotation counter thing
                    rotation_counter += 1
                    curr_home = marker_dict[marker_ids['toe_l']]
                    curr_home_rot = rots[47].copy()
                    print(f"Rotation detected: {rotation_counter} (Δ={delta:.4f})")
                    new_sequence = False

                    move_idx = 0
                    detected = False

                elif abs(delta) > 0.515 and (rotation_counter - 3) % 4 == 0: #need 0.51 for full_dance.csv
                    rotation_counter += 1
                    curr_home = marker_dict[marker_ids['toe_l']]
                    curr_home_rot = rots[47].copy()
                    print(f"Rotation detected: {rotation_counter} (Δ={delta:.4f})")
                    new_sequence = False

                    move_idx = 0
                    detected = False
            else:
                move_idx += 1
                detected = False

                redis_anymal.publish(f"{move_idx + 1}")
        else:
            print(f"[ ] No significant movement for {move}. Frame {ts:.4f} (Δ={delta:.4f})")