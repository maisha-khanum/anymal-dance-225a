import sys
import time
import pandas as pd

if len(sys.argv) < 2:
    print("Usage: python state_machine_from_csv.py <csv_filename>")
    sys.exit(1)

csv_file = sys.argv[1]

# Load the CSV
df = pd.read_csv(csv_file)

# Preprocess into a dict of {timestamp: {marker_id: position}}
grouped = {}
for _, row in df.iterrows():
    ts = row['timestamp']
    marker_id = int(row['rigid_body_id'])
    pos = [row['pos_x'], row['pos_y'], row['pos_z']]
    if ts not in grouped:
        grouped[ts] = {}
    grouped[ts][marker_id] = pos

timestamps = sorted(grouped.keys())

# ---- Motion detection setup ----

move_sequence = ['left', 'backward', 'hop', 'stomp_left', 'stomp_right', 'chacha', 'rotate']
thresholds = { # TODO: Adjust these thresholds (m), keep positive values relatvie to home position
    'left': 0.05,
    'backward': 0.05,
    'hop': 0.03,
    'stomp_left': 0.03,
    'stomp_right': 0.03,
    'chacha': 0.05,
    'rotate': 0.05
}

# Marker IDs for the skeleton
marker_ids = {
    'toe_l': 47,
    'toe_r': 51,
    'knee_l': 45,
    'knee_r': 49,
    'wrist_l': 9,
    'wrist_r': 28,
    'chest': 3
}

# Get marker pos by ID for a specific timestamp
def get_marker_pos(marker_id, timestamp):
    return grouped.get(timestamp, {}).get(marker_id, None)

# ---- Run state machine ----

is_looping = True
move_idx = 0
curr_home = None

print("Starting state machine from CSV...\n")

for ts in timestamps[380:]: #TODO: truncating the start which had no movement
    time.sleep(0.2)  # simulate faster playback

    if curr_home is None:
        print("Setting home position...")
        toe_l = get_marker_pos(marker_ids['toe_l'], ts)
        if toe_l is None:
            continue
        curr_home = toe_l

    move = move_sequence[move_idx]
    print(f"Looking for move: {move}")

    toe_l = get_marker_pos(marker_ids['toe_l'], ts)
    toe_r = get_marker_pos(marker_ids['toe_r'], ts)
    knee_l = get_marker_pos(marker_ids['knee_l'], ts)
    knee_r = get_marker_pos(marker_ids['knee_r'], ts)
    wrist_l = get_marker_pos(marker_ids['wrist_l'], ts)
    wrist_r = get_marker_pos(marker_ids['wrist_r'], ts)
    chest = get_marker_pos(marker_ids['chest'], ts)

    if any(x is None for x in [toe_l, toe_r, knee_l, knee_r, wrist_l, wrist_r]):
        print("Missing marker data...")
        continue

    detected = False
    delta = 0

    if move == 'left':
        delta = toe_l[1] - curr_home[1]
        if delta > thresholds['left']:
            detected = True

    elif move == 'backward':
        delta = toe_l[0] - curr_home[0]
        if -delta > thresholds['backward']:
            detected = True

    elif move == 'hop':
        # check if both knees are below the threshold
        # delta_l = knee_l[2] - curr_home[2] # not sure how much the knee goes down when winding up to jump
        # delta_r = knee_r[2] - curr_home[2]
        # if -delta_l > thresholds['hop'] and -delta_r > thresholds['hop']:
        #     detected = True

        # Check if chest is below the threshold
        delta_c = chest[2] - curr_home[2]
        if -delta_c > thresholds['hop']:
            detected = True

    elif move == 'stomp_left':
        delta = knee_l[2] - curr_home[2]
        if delta > thresholds['stomp_left']:
            detected = True

    elif move == 'stomp_right':
        delta = knee_r[2] - curr_home[2]
        if delta > thresholds['stomp_right']:
            detected = True

    elif move == 'chacha':
        delta_l = wrist_l[2] - curr_home[2]
        delta_r = wrist_l[2] - curr_home[2]
        if delta_l > thresholds['chacha'] and delta_r < thresholds['chacha']:
            detected = True

    elif move == 'rotate':
        delta = toe_l[0] - curr_home[0]
        if delta > thresholds['rotate']:
            detected = True

    if detected:
        print(f"Detected move: {move.upper()}\n")
        move_idx += 1
        if move_idx >= len(move_sequence):
            print("All moves detected. Done.")
            break
    else:
        print(f"[ ] No significant movement for {move}. Frame {ts:.4f} (delta: {delta:.4f})")

# TODO: need to handle the transformation of the x,y coordinates relative to home after the rotation