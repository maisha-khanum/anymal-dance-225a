# mocap_logic_loop.py
# A simple script that polls Redis for OptiTrack rigid-body pose data
# and performs custom logic on the position and orientation.

import redis
import time

# Redis key prefixes (must match your data-publishing script)
RIGID_BODY_POS_KEY = "sai2::optitrack::rigid_body_pos::"
RIGID_BODY_ORI_KEY = "sai2::optitrack::rigid_body_ori::"

# Connect to local Redis instance
redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

# redis-cli                         
# 127.0.0.1:6379> MONITOR
# python3 PythonClient/StreamData.py

def parse_list_string(s):
    """
    Convert a string of form "[x,y,z]" into a list of floats [x, y, z].
    """
    return [float(val) for val in s.strip('[]').split(',')]


def get_rigid_body_ids():
    """
    Fetch all known rigid-body IDs by listing keys in Redis.
    """
    keys = redis_client.keys(RIGID_BODY_POS_KEY + '*')
    # Extract the trailing ID from each key
    return [key.replace(RIGID_BODY_POS_KEY, '') for key in keys]


def perform_logic(positions, orientations):
    """
    Placeholder for your custom logic.
    `positions` and `orientations` are dicts mapping ID -> [x,y,z] or [qx,qy,qz,qw].
    """
    for id, pos in positions.items():
        x, y, z = pos
        # Example logic: alert when Z height exceeds 1.0 meter
        if z > 1.0:
            print(f"Rigid body {id} above 1.0 m: z = {z:.3f}")

    # You can also use orientations:
    # for id, ori in orientations.items():
    #     qx, qy, qz, qw = ori
    #     # compute e.g. yaw, pitch, roll if needed


def main(poll_rate_hz=100):
    sleep_time = 1.0 / poll_rate_hz
    try:
        while True:
            ids = get_rigid_body_ids()
            positions = {}
            orientations = {}

            for id in ids:
                pos_str = redis_client.get(RIGID_BODY_POS_KEY + id)
                ori_str = redis_client.get(RIGID_BODY_ORI_KEY + id)
                if pos_str:
                    positions[id] = parse_list_string(pos_str)
                if ori_str:
                    orientations[id] = parse_list_string(ori_str)

            if positions:
                perform_logic(positions, orientations)

            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("Logic loop terminated by user.")


if __name__ == '__main__':
    main()
