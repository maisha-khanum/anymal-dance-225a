import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
from scipy.spatial.transform import Rotation as R

# =====================
# Read CSV and flags from terminal
# =====================
if len(sys.argv) < 2:
    print("Usage: python marker_viewer.py <csv_name_with_extension> [--show_arrows true|false]")
    sys.exit(1)

csv_name = sys.argv[1]
csv_path = f"{csv_name}"

# Check if --show_arrows flag is passed
show_arrows = False
if "--show_arrows" in sys.argv:
    flag_index = sys.argv.index("--show_arrows")
    if flag_index + 1 < len(sys.argv):
        show_arrows = sys.argv[flag_index + 1].lower() == "true"

# Load CSV
data = pd.read_csv(csv_path)
timestamps = data['timestamp'].unique()

first_frame = data[data['timestamp'] == timestamps[0]]
marker_51 = first_frame[first_frame['rigid_body_id'] == 51]
center_51 = marker_51[['pos_x', 'pos_y', 'pos_z']].values[0]

# Global flags and state
restart_animation = True
paused = False
current_marker_ids = np.array([])  # Will store marker IDs per frame
orientation_quivers = []           # Will store quiver arrow handles

# Set up figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], s=10, picker=True)  # Enable picking

range_xyz = 1.0  # or adjust this value for your scene size
ax.set_xlim([center_51[0] - range_xyz, center_51[0] + range_xyz])
ax.set_ylim([center_51[1] - range_xyz, center_51[1] + range_xyz])
ax.set_zlim([0, center_51[2] + range_xyz])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Marker Movement Over Time')

plt.ion()
plt.show()

# === Event handlers ===

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

# Register event handlers
fig.canvas.mpl_connect('key_press_event', on_key)
fig.canvas.mpl_connect('pick_event', on_pick)

# Quaternion to forward vector (Z-axis)
def get_forward_vector_from_quat(q):
    r = R.from_quat(q)
    return r.apply([0, 0, 0.05])  # small forward vector

# === Animation loop ===
while True:
    if restart_animation:
        restart_animation = False
        elev, azim = ax.elev, ax.azim  # Save current camera view

        for i, ts in enumerate(timestamps):
            while paused:
                plt.pause(0.1)

            # Clear previous orientation arrows
            for q in orientation_quivers:
                q.remove()
            orientation_quivers = []

            # Get data for current timestamp
            frame_data = data[data['timestamp'] == ts].sort_values(by='rigid_body_id')
            pos = frame_data[['pos_x', 'pos_y', 'pos_z']].to_numpy()
            rots = frame_data[['rot_x', 'rot_y', 'rot_z', 'rot_w']].to_numpy()
            ids = frame_data['rigid_body_id'].to_numpy()
            current_marker_ids = ids

            # Set colors: marker 51 red, others blue
            colors = np.array([[1, 0, 0] if rid == 51 else [0, 0, 1] for rid in ids])

            # Update scatter plot
            sc._offsets3d = (pos[:, 0], pos[:, 1], pos[:, 2])
            sc.set_color(colors)
            ax.view_init(elev=elev, azim=azim)
            ax.set_title(f'{csv_name} — Timestamp: {ts:.4f} (Frame {i+1} of {len(timestamps)})')

            # Plot orientation arrows if enabled
            if show_arrows:
                for j in range(len(pos)):
                    origin = pos[j]
                    quat = rots[j]  # [x, y, z, w]
                    vec = get_forward_vector_from_quat(quat)
                    q = ax.quiver(origin[0], origin[1], origin[2],
                                  vec[0], vec[1], vec[2],
                                  length=1.0, normalize=False, color='gray')
                    orientation_quivers.append(q)

            plt.draw()
            plt.pause(0.01)
    else:
        plt.pause(0.1)
