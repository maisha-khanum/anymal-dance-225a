import sys
import time
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R

# === ARGUMENT PARSING ===
if len(sys.argv) < 2:
    print("Usage: python combined_viewer.py <csv_filename> [--show_arrows true|false]")
    sys.exit(1)

csv_name = sys.argv[1]
show_arrows = "--show_arrows" in sys.argv and sys.argv[sys.argv.index("--show_arrows") + 1].lower() == "true"

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
    'chacha': 0.82,
    'rotate': 0.15
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

# === VISUALIZATION SETUP ===
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], s=10, picker=True)

ax.set_xlim([-3.5, 3.5])
ax.set_ylim([-3.5, 3.5])
ax.set_zlim([0, 1.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Marker Movement and State Machine Detection')

orientation_quivers = []
current_marker_ids = np.array([])

restart_animation = True
paused = False

def on_key(event):
    global restart_animation, paused
    if event.key == 'z':
        restart_animation = True
        print("Restarting animation...")
    elif event.key == 'x':
        paused = not paused
        print("Paused" if paused else "Resumed")

def on_pick(event):
    ind = event.ind
    for i in ind:
        print(f"Clicked marker ID: {current_marker_ids[i]}")

fig.canvas.mpl_connect('key_press_event', on_key)
fig.canvas.mpl_connect('pick_event', on_pick)

def get_forward_vector_from_quat(q):
    r = R.from_quat(q)
    return r.apply([0, 0, 0.05])

# === MAIN ANIMATION LOOP ===
plt.ion()
plt.show()

elev, azim = ax.elev, ax.azim

for i, ts in enumerate(timestamps[380:]):  # Adjust slice for skipping idle frames
    while paused:
        plt.pause(0.5)

    frame_data = data[data['timestamp'] == ts].sort_values(by='rigid_body_id')
    pos = frame_data[['pos_x', 'pos_y', 'pos_z']].to_numpy()
    rots = frame_data[['rot_x', 'rot_y', 'rot_z', 'rot_w']].to_numpy()
    ids = frame_data['rigid_body_id'].to_numpy()
    current_marker_ids = ids

    marker_dict = {int(rid): p for rid, p in zip(ids, pos)}

    if curr_home is None and marker_ids['toe_l'] in marker_dict:
        curr_home = marker_dict[marker_ids['toe_l']]
        curr_home_rot = rots[47].copy()
        print(f"Set home position: {curr_home}")

    # Motion Detection Logic
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
    

        detected = False
        delta = 0

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
            delta_l = wrist_l[2] - curr_home[2]
            if abs(delta_l) > thresholds['chacha']:
                detected = True

        elif move == 'rotate' and toe_l is not None:
            delta = rots[47][2] - curr_home_rot[2]
            if abs(delta) > thresholds['rotate']:
                detected = True

        if detected:
            print(f"✔ Detected move: {move.upper()} (Δ={delta:.4f})")
            move_idx += 1
            if move_idx >= len(move_sequence):
                print("✅ All moves detected. Done.")
        else:
            print(f"[ ] No significant movement for {move}. Frame {ts:.4f} (Δ={delta:.4f})")

    # === Visualization Update ===
    for q in orientation_quivers:
        q.remove()
    orientation_quivers = []

    sc._offsets3d = (pos[:, 0], pos[:, 1], pos[:, 2])
    colors = np.array([[1, 0, 0] if rid == 51 else [0, 0, 1] for rid in ids])
    sc.set_color(colors)
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(f'{csv_name} — Timestamp: {ts:.4f} (Frame {i+1} of {len(timestamps)})')

    if show_arrows:
        for j in range(len(pos)):
            origin = pos[j]
            quat = rots[j]
            vec = get_forward_vector_from_quat(quat)
            q = ax.quiver(origin[0], origin[1], origin[2],
                          vec[0], vec[1], vec[2],
                          length=1.0, normalize=False, color='gray')
            orientation_quivers.append(q)

    plt.draw()
    plt.pause(0.2)
