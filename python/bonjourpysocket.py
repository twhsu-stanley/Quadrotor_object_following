from cmath import pi
from math import atan2
from zeroconf import ServiceBrowser, Zeroconf
import socket
import sys
sys.path.append("/usr/local/lib")
import json
import struct
# from random import random
# import time
# import datetime
# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt


BBPORT = 8080
IPHONE_FOV = 63.3
CENTER_DATUM = .5
PRINT_OBJOBS = True
PRINT_POSE = True
NUM_MSGS = 1000
MSG_TYPEDICT = {"ArKIT_POSE":11,"Object_Observation":25}
i=0
j=0

# timeanalysis = {"pose":np.zeros(NUM_MSGS), "objectobs":np.zeros(NUM_MSGS)}

def calc_yawangle(x):
    delta_yaw = pi*((x - .5) * IPHONE_FOV)/ 180
    return delta_yaw

def calc_depthviaBBox(area):
    
    depth = area/100

    return depth

def parse_iphonemsg(msgbytes):
    data = msgbytes.decode()
    # print(data)
    data=data.split('ENDENTRY')
    jsondict = json.loads(data[0])

    return jsondict

def pack_message4beaglbone(jsondict):
    global i,j
    if jsondict["Type"] == "ArKIT_POSE":
        # timeanalysis["pose"][i]=time.time_ns()
        i+=1
        msgpayload = pack_pose_msg(jsondict)

    if jsondict["Type"] == "Object_Observation":
        # timeanalysis["objectobs"][j]=time.time_ns()
        j+=1
        msgpayload = pack_objectobs_msg(jsondict)

    return msgpayload

def pack_pose_msg(pose_dict):
    pose = pose_dict
    # position = objectobservation['X']
    # delta_yaw = calc_yawangle(position)
    # position_y = objectobservation['Y']
    # depth = objectobservation['Height']*objectobservation['Width']
    if(PRINT_POSE):
        print(f"X: {pose['X']}\tY: {pose['Y']}\tZ: {pose['Z']}\tRoll: {pose['Roll']}\tPitch: {pose['Pitch']}\tYaw: {pose['Yaw']}")
    
    msgtype = struct.pack("Q",MSG_TYPEDICT[pose_dict['Type']])  
    pose_xytrpy_msg = msgtype + struct.pack("d",pose['X']) + struct.pack("d",pose['Y']) + struct.pack("d",pose['Z'])
    pose_xytrpy_msg += struct.pack("d",pose['Roll']) + struct.pack("d",pose['Pitch']) + struct.pack("d",pose['Yaw'])
    # pose_xytrpy_msg = "dumb"
    return pose_xytrpy_msg

def pack_objectobs_msg(objectobservation):

    position = objectobservation['X']
    delta_yaw = calc_yawangle(position)
    position_y = objectobservation['Y']
    depth = 0.2189 * (objectobservation['Height']*objectobservation['Width']) ** (-0.5)
    if(PRINT_OBJOBS):
        print(f"Class: {objectobservation['Class']} \t Area: {depth}\t Angle: {delta_yaw} \t Depth: {objectobservation['Depth']}\tWidth: {objectobservation['Width']}\tHeight: {objectobservation['Height']}")

    msgtype = struct.pack("Q",MSG_TYPEDICT[objectobservation['Type']])    
    payload = msgtype + struct.pack("d", delta_yaw)+struct.pack("d", position_y)+struct.pack("d", depth)

    return payload

class MyListener:
    def __init__(self):
        self.info = None

    def remove_service(self, zeroconf, type, name):
        print("Service %s removed" % (name,))

    def add_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)

        self.info = info
        print("Finding Ipad bonjour service named WiTap\n\n")
        print("Service %s added, service info: %s" % (name, info))
    
    def update_service(self, zeroconf, type, name):
        print("shutup the errors")



zeroconf = Zeroconf()
listener = MyListener()
browser = ServiceBrowser(zeroconf, "_superapp._tcp.local.", listener)
# browser = ServiceBrowser(zeroconf, "_superapp._udp.local.", listener)
try:
    input("Press enter to exit...\n\n")
finally:
    zeroconf.close()



#TCP
mysocket = socket.socket()
mysocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
beaglebonesocket = socket.socket()
beaglebonesocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
port = listener.info.port
host = socket.inet_ntoa(listener.info.addresses[0])
mysocket.connect((host,port))
print('gotti')


#UDP
# mysocket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
# mysocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# beaglebonesocket = socket.socket()
# beaglebonesocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# port = listener.info.port
# host = socket.inet_ntoa(listener.info.addresses[0])
# mysocket.bind((host,port))

beaglehost = "127.0.0.1"
beaglebonesocket.connect((beaglehost,BBPORT))


# start = dt.datetime.now()
# while True:


while True:

    # for timing analysis    
    # if j >= NUM_MSGS or i >= NUM_MSGS:
    #     break

    try:
        data = mysocket.recv(1024)
        # data = mysocket.recvfrom(1024)
        if data:

            data_jsondict = parse_iphonemsg(data)
            if data_jsondict['Class'] != 'sports ball':
                continue
            msg = pack_message4beaglbone(data_jsondict)
            beaglebonesocket.send(msg)


    except KeyboardInterrupt:
        print('user exit requested')
        exit()
    except:
        print(f'\t\texception from receive attempt\n\t\t{data}')

# print('all the messages got')
# df=pd.DataFrame(timeanalysis)
# # plt.plot(df.pose.diff() * 1e-6)
# dftemp=df.query('objectobs >0')
# plt.plot(dftemp.pose)
# plt.plot(dftemp.objectobs)
# plt.legend(['pose','objectobs'])
# plt.show()
# df.to_clipboard()
# input("Press enter to exit...\n\n")
# c.send("Server stopped\n")
# print("server stopped")
# c.close()
