import socket
import struct
import time
from util import (
    q2mat, 
    URaa2rpy
)

# Request command constants
RC_PICKIT_NO_COMMAND                    = -1
RC_PICKIT_CHECK_MODE                    = 0
RC_PICKIT_SHUTDOWN_SYSTEM               = 2
RC_PICKIT_FIND_CALIB_PLATE              = 10
RC_PICKIT_CONFIGURE_CALIB               = 11
RC_PICKIT_COMPUTE_CALIB                 = 12
RC_PICKIT_VALIDATE_CALIB                = 13
RC_PICKIT_LOOK_FOR_OBJECTS              = 20
RC_PICKIT_LOOK_FOR_OBJECTS_WITH_RETRIES = 21
RC_PICKIT_CAPTURE_IMAGE                 = 22
RC_PICKIT_PROCESS_IMAGE                 = 23
RC_PICKIT_NEXT_OBJECT                   = 30
RC_PICKIT_CONFIGURE                     = 40
RC_PICKIT_SET_CYLINDER_DIM              = 41
RC_SAVE_ACTIVE_SETUP                    = 42
RC_SAVE_ACTIVE_PRODUCT                  = 43
RC_PICKIT_SAVE_SCENE                    = 50
RC_PICKIT_BUILD_BACKGROUND              = 60
RC_PICKIT_GET_PICK_POINT_DATA           = 70

# Response status constants
PICKIT_UNKNOWN_COMMAND                  = -99
PICKIT_ROBOT_MODE                       =   0
PICKIT_IDLE_MODE                        =   1
PICKIT_SHUTDOWN_REQUEST_ACCEPTED        =   5
PICKIT_SHUTDOWN_REQUEST_REJECTED        =   6
PICKIT_FIND_CALIB_PLATE_OK              =  10
PICKIT_FIND_CALIB_PLATE_FAILED          =  11
PICKIT_CONFIGURE_CALIB_OK               =  12
PICKIT_CONFIGURE_CALIB_FAILED           =  13
PICKIT_COMPUTE_CALIB_OK                 =  14
PICKIT_COMPUTE_CALIB_FAILED             =  15
PICKIT_VALIDATE_CALIB_OK                =  16
PICKIT_VALIDATE_CALIB_FAILED            =  17
PICKIT_OBJECT_FOUND                     =  20
PICKIT_NO_OBJECTS                       =  21
PICKIT_NO_IMAGE_CAPTURED                =  22
PICKIT_EMPTY_ROI                        =  23
PICKIT_IMAGE_CAPTURED                   =  26
PICKIT_INVALID_LICENSE                  =  27
PICKIT_CONFIG_OK                        =  40
PICKIT_CONFIG_FAILED                    =  41
PICKIT_SAVE_SNAPSHOT_OK                 =  50
PICKIT_SAVE_SNAPSHOT_FAILED             =  51
PICKIT_BUILD_BKG_CLOUD_OK               =  60
PICKIT_BUILD_BKG_CLOUD_FAILED           =  61
PICKIT_GET_PICK_POINT_DATA_OK           =  70
PICKIT_GET_PICK_POINT_DATA_FAILED       =  71

# Object type constants
PICKIT_TYPE_SQUARE                =  21
PICKIT_TYPE_RECTANGLE             =  22
PICKIT_TYPE_CIRCLE                =  23
PICKIT_TYPE_ELLIPSE               =  24
PICKIT_TYPE_CYLINDER              =  32
PICKIT_TYPE_SPHERE                =  33
PICKIT_YTPE_POINTCLOUD            =  35
PICKIT_TYPE_BLOB                  =  50

PICKIT_IP = '192.168.100.32'  # Change if needed
PICKIT_PORT = 5001
MULT = 10000

# Message sizes
REQUEST_SIZE = 48
RESPONSE_SIZE = 64

# Orientation convention and protocol version (see docs)
# Convention that is being used to encode object or robot flange orientations. The following conventions are supported:
# 1: Angle-axis (3D vector consisting of the unit axis multiplied by the angle in radians) → UNIVERSAL ROBOTS
# 2: Quaternions (w,x,y,z) → GENERIC, ABB
# 3: Euler Angles (x-y’-z”, in degrees) → STÄUBLI
# 4: Fixed Angles (x-y-z, in degrees) → FANUC, NACHI, OMRON TM, YASKAWA
# 5: Euler Angles (z-y’-x”, in degrees) → HANWHA, KUKA
# 6: Euler Angles (z-y’-z”, in degrees) → COMAU, DOOSAN, OMRON

ORIENTATION_CONVENTION = 2  # 1 = GENERIC (quaternions)
PROTOCOL_VERSION = 11

# New impl
def build_request(cmd=RC_PICKIT_CHECK_MODE):
    # position (int32[3]), orientation (int32[4]), command (int32), payload (int32[2]), meta (int32[2])
    position = [0, 0, 0]
    orientation = [0, 0, 0, 0]
    command = cmd
    payload = [0, 0]
    meta = [ORIENTATION_CONVENTION, PROTOCOL_VERSION]
    # Pack as big-endian int32s
    fmt = '>3i4i1i2i2i'  # 3+4+1+2+2 = 12 int32s
    packed = struct.pack(fmt, *(position + orientation + [command] + payload + meta))
    assert len(packed) == REQUEST_SIZE
    return packed

def parse_response(data):
    # position (int32[3]), orientation (int32[4]), payload (int32[6]), status (int32), meta (int32[2])
    fmt = '>3i4i6i1i2i'
    unpacked = struct.unpack(fmt, data)
    position = [x / MULT for x in unpacked[0:3]]
    orientation = [x / MULT for x in unpacked[3:7]]
    payload = unpacked[7:13]
    status = unpacked[13]
    meta = unpacked[14:16]
    return {
        'position': position,
        'orientation': orientation,
        'payload': payload,
        'status': status,
        'meta': meta
    }

# PICKIT_NO_OBJECTS                       =  21
# PICKIT_NO_IMAGE_CAPTURED                =  22
# PICKIT_EMPTY_ROI                        =  23
# PICKIT_IMAGE_CAPTURED                   =  26
def pickit_search():

    results_to_save = []

    with socket.create_connection((PICKIT_IP, PICKIT_PORT), timeout=8) as s:
        command = RC_PICKIT_LOOK_FOR_OBJECTS # runs capture and process in 1 command
        command_txt = "Look for objects"
        req = build_request(cmd=command)
        # print(f'Connected to Pickit at {PICKIT_IP}:{PICKIT_PORT}')
        s.sendall(req)
        # print(f'Sent ~{command_txt}~ request')
        resp = b''
        while len(resp) < RESPONSE_SIZE:
            chunk = s.recv(RESPONSE_SIZE - len(resp))
            if not chunk:
                raise RuntimeError('Connection closed by Pickit')
            resp += chunk
        result = parse_response(resp)
        status = result['status']
        # print(f'Response {status = }')
        # print('Response:', result, '\n', "0"+"-"*30)
        if status == 20:
            results_to_save.append(result)

        # command = RC_PICKIT_PROCESS_IMAGE
        # command_txt = f"{RC_PICKIT_PROCESS_IMAGE=}"
        # req = build_request(cmd=command)
        # print(f'Connected to Pickit at {PICKIT_IP}:{PICKIT_PORT}')
        # s.sendall(req)
        # print(f'Sent ~{command_txt}~ request')
        # resp = b''
        # while len(resp) < RESPONSE_SIZE:
        #     chunk = s.recv(RESPONSE_SIZE - len(resp))
        #     if not chunk:
        #         raise RuntimeError('Connection closed by Pickit')
        #     resp += chunk
        # result = parse_response(resp)
        # status = result['status']
        # print(f'Response {status = }')
        # print('Response:', result, '\n', "0"+"-"*30)

        # while status != PICKIT_NO_OBJECTS:
        for i in range(1, 100):
            if status == PICKIT_NO_OBJECTS:
                break
            # command = RC_PICKIT_GET_PICK_POINT_DATA
            # command_txt = "Get pick point data"
            # req = build_request(cmd=command)
            # print(f'Connected to Pickit at {PICKIT_IP}:{PICKIT_PORT}')
            # s.sendall(req)
            # print(f'Sent ~{command_txt}~ request')
            # resp = b''
            # while len(resp) < RESPONSE_SIZE:
            #     chunk = s.recv(RESPONSE_SIZE - len(resp))
            #     if not chunk:
            #         raise RuntimeError('Connection closed by Pickit')
            #     resp += chunk
            # result = parse_response(resp)
            # print(f'{result["status"] = }')
            # print('Response:', result)
            
            command = RC_PICKIT_NEXT_OBJECT
            command_txt = "Next object"
            req = build_request(cmd=command)
            # print(f'Connected to Pickit at {PICKIT_IP}:{PICKIT_PORT}')
            s.sendall(req)
            # print(f'Sent ~{command_txt}~ request')
            resp = b''
            while len(resp) < RESPONSE_SIZE:
                chunk = s.recv(RESPONSE_SIZE - len(resp))
                if not chunk:
                    raise RuntimeError('Connection closed by Pickit')
                resp += chunk
            result = parse_response(resp)
            status = result['status']
            # print(f'Response {status = }')
            # print('Response:', result, '\n', i, "-"*30)

            if status == 20:
                results_to_save.append(result)

            # if input("scan more? >> ") == ord('q'):
            #     break

            # if count > 4:
            #     break
            # count += 1
        

        return results_to_save

        # save results to file
        lines = ''
        if len(results_to_save) > 0:
            with open('data/results_.txt', 'w') as f:
            # if True:
                for index, result in enumerate(results_to_save):
                    q = result['orientation']
                    rmat = q2mat(q, False)
                    lines += f' ----------{index}. m12_screw ----------\n'
                    lines += "position:\n"
                    lines += f"  x: {result['position'][0]}\n"
                    lines += f"  y: {result['position'][1]}\n"
                    lines += f"  z: {result['position'][2]}\n"
                    lines += "rotation\n"
                    lines += "[\n"
                    # lines += f"  w: {result['orientation'][0]}\n"
                    # lines += f"  x: {result['orientation'][1]}\n"
                    # lines += f"  y: {result['orientation'][2]}\n"
                    # lines += f"  z: {result['orientation'][3]}\n"
                    lines += f"  {rmat[0][0]}, {rmat[0][1]}, {rmat[0][2]} \n"
                    lines += f"  {rmat[1][0]}, {rmat[0][1]}, {rmat[1][2]} \n"
                    lines += f"  {rmat[2][0]}, {rmat[2][1]}, {rmat[2][2]} \n"
                    lines += "]\n"
                f.write(lines)

if __name__ == '__main__':            
    # run the program
    results = pickit_search()
    print(f"\n\n\n")
    for i in results:
        print(i)