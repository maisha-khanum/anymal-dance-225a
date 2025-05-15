import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

# =====================
# Read CSV from terminal
# =====================
if len(sys.argv) < 2:
    print("Usage: python animate_markers.py <csv_name_with_extension>")
    sys.exit(1)

csv_name = sys.argv[1]
csv_path = f"{csv_name}"

# Load CSV
data = pd.read_csv(csv_path)
timestamps = data['timestamp'].unique()

# Global flags and state
restart_animation = True
paused = False
current_marker_ids = np.array([])  # Will store marker IDs per frame

# Set up figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], s=10, picker=True)  # <-- Enable picking

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([0, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Marker Movement Over Time')

plt.ion()
plt.show()

# === Event handlers ===

# press 'z' to restart animation, 'x' to pause/resume

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

# === Animation loop ===
while True:
    if restart_animation:
        restart_animation = False  # reset flag
        elev, azim = ax.elev, ax.azim  # Save current camera view

        for i, ts in enumerate(timestamps):
            while paused:
                plt.pause(0.1)  # Wait while paused

            frame_data = data[data['timestamp'] == ts].sort_values(by='rigid_body_id')
            pos = frame_data[['pos_x', 'pos_y', 'pos_z']].to_numpy()
            ids = frame_data['rigid_body_id'].to_numpy()
            current_marker_ids = ids

            colors = np.array([[1, 0, 0] if rid == 51 else [0, 0, 1] for rid in ids])

            sc._offsets3d = (pos[:, 0], pos[:, 1], pos[:, 2])
            sc.set_color(colors)
            ax.view_init(elev=elev, azim=azim)
            ax.set_title(f'{csv_name} — Timestamp: {ts:.4f} (Frame {i+1} of {len(timestamps)})')

            plt.draw()
            plt.pause(0.01)
    else:
        plt.pause(0.1)
