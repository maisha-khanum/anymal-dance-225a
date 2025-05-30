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
redis_anymal = redis.Redis(host='192.168.0.234', port=6379)

# === CONFIG ===
skeleton_name = "Ariane"
move_sequence = ['left', 'right', 'backward', 'hop', 'stomp_left', 'stomp_right', 'chacha', 'rotate']
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

# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.
def receive_new_frame(data_dict):
    order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
    dump_args = True
    if dump_args == True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "="
            if key in data_dict :
                out_string += str(data_dict[key]) + " "
            out_string+="/"
        # print(out_string)

RIGID_BODY_POS_KEY = "sai2::optitrack::rigid_body_pos::"
RIGID_BODY_ORI_KEY = "sai2::optitrack::rigid_body_ori::"

def receive_skeleton_frame(new_id, skeleton):
    timestamp = time.time()
    for i, rigid_body in enumerate(skeleton.rigid_body_list):
        pos = rigid_body.pos  # [x, y, z]
        rot = rigid_body.rot  # [x, y, z, w]
        # Save to Redis (existing behavior)
        position_key = f"{new_id}::{i + 1}::pos"
        orientation_key = f"{new_id}::{i + 1}::ori"
        redis_client.set(position_key, '[' + ', '.join(map(str, pos)) + ']')
        redis_client.set(orientation_key, '[' + ', '.join(map(str, rot)) + ']')

        # Construct Redis keys for position and orientation
        position_key = f"{new_id}::{i + 1}::pos"
        orientation_key = f"{new_id}::{i + 1}::ori"

        # Convert position and orientation to string format
        position_str = '[' + ', '.join(map(str, rigid_body.pos)) + ']'
        orientation_str = '[' + ', '.join(map(str, rigid_body.rot)) + ']'

        # Set the position and orientation in Redis
        redis_client.set(position_key, position_str)
        redis_client.set(orientation_key, orientation_str)

def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals

def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    changeBitstreamString = "  Can Change Bitstream Version = "
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
        changeBitstreamString+="false"
    else:
        print("  Using Unicast")
        changeBitstreamString+="true"

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    MotiveVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))

    print(changeBitstreamString)
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_client.data_socket)))
    print("  PythonVersion    %s"%(sys.version))


def print_commands(can_change_bitstream):
    outstring = "Commands:\n"
    outstring += "Return Data from Motive\n"
    outstring += "  s  send data descriptions\n"
    outstring += "  r  resume/start frame playback\n"
    outstring += "  p  pause frame playback\n"
    outstring += "     pause may require several seconds\n"
    outstring += "     depending on the frame data size\n"
    outstring += "Change Working Range\n"
    outstring += "  o  reset Working Range to: start/current/end frame 0/0/end of take\n"
    outstring += "  w  set Working Range to: start/current/end frame 1/100/1500\n"
    outstring += "Return Data Display Modes\n"
    outstring += "  j  print_level = 0 supress data description and mocap frame data\n"
    outstring += "  k  print_level = 1 show data description and mocap frame data\n"
    outstring += "  l  print_level = 20 show data description and every 20th mocap frame data\n"
    outstring += "Change NatNet data stream version (Unicast only)\n"
    outstring += "  3  Request NatNet 3.1 data stream (Unicast only)\n"
    outstring += "  4  Request NatNet 4.1 data stream (Unicast only)\n"
    outstring += "General\n"
    outstring += "  t  data structures self test (no motive/server interaction)\n"
    outstring += "  c  print configuration\n"
    outstring += "  h  print commands\n"
    outstring += "  q  quit\n"
    outstring += "\n"
    outstring += "NOTE: Motive frame playback will respond differently in\n"
    outstring += "       Endpoint, Loop, and Bounce playback modes.\n"
    outstring += "\n"
    outstring += "EXAMPLE: PacketClient [serverIP [ clientIP [ Multicast/Unicast]]]\n"
    outstring += "         PacketClient \"192.168.10.14\" \"192.168.10.14\" Multicast\n"
    outstring += "         PacketClient \"127.0.0.1\" \"127.0.0.1\" u\n"
    outstring += "\n"
    print(outstring)

def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF,    "",  (s_client.server_ip_address, s_client.command_port) )

def test_classes():
    totals = [0,0,0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d"%totals[0])
    print("[FAIL] Count = %3.1d"%totals[1])
    print("[SKIP] Count = %3.1d"%totals[2])

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

def get_marker_val(marker_key):
    val = redis_client.get(marker_key)
    if val:
        return list(map(float, val.strip('[]').split(',')))
    else:
        return None

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    optionsDict = {}
    optionsDict["clientAddress"] = "172.24.69.101"
    optionsDict["serverAddress"] = "172.24.68.67"
    optionsDict["use_multicast"] = False

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.new_frame_listener = receive_new_frame 
    streaming_client.skeleton_listener = receive_skeleton_frame
    
    # Set print level
    streaming_client.set_print_level(0)

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")


    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    # print_configuration(streaming_client)
    # print("\n")
    # print_commands(streaming_client.can_change_bitstream_version())

    print("Starting chest stream to anymal...\n")

    # while is_looping:
    #     time.sleep(1)


    while is_looping:
        time.sleep(0.01)

        chest_pos = get_marker_val(skeleton_name+"::3::pos")
        chest_rot = get_marker_val(skeleton_name+"::3::rot")

        redis_anymal.publish('follow', chest_pos)
        redis_anymal.publish('follow', chest_rot)


    streaming_client.shutdown()
    print("Exited cleanly.")