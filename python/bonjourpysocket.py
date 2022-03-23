from cmath import pi
from zeroconf import ServiceBrowser, Zeroconf
import socket
import sys
sys.path.append("/usr/local/lib")
from lcmtypes import pose_xyt_t
import lcm
import json
import struct
from random import random
import time


BBPORT = 8080

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
try:
    input("Press enter to exit...\n\n")
finally:
    zeroconf.close()




mysocket = socket.socket()
mysocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
beaglebonesocket = socket.socket()
beaglebonesocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

port = listener.info.port
host = socket.inet_ntoa(listener.info.addresses[0])



beaglehost = "127.0.0.1"
beaglebonesocket.connect((beaglehost,BBPORT))

mysocket.connect((host,port))
print('gotti')
while True:

    try:
        data = mysocket.recv(1024)
        if data:

            # print(data)
            data = data.decode()
            data=data.split('ENDENTRY')
            objectobservation = json.loads(data[0])
            print(objectobservation)
            position = objectobservation['X']
            position_y = objectobservation['Y']
            # beaglebonesocket.send(position.encode()+position.encode()+position.encode())
            depth = random()*10
            payload = struct.pack("d", position)+struct.pack("d", position_y)+struct.pack("d", depth)
            # print(position,position_y,depth)
            # print(payload)
            beaglebonesocket.send(payload)

            time.sleep(.1)



    except KeyboardInterrupt:
        print('user exit requested')
        exit()
    except:
        print('exception from receive attempt')
# c.send("Server stopped\n")
# print("server stopped")
# c.close()
