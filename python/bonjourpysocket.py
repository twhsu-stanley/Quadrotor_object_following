from cmath import pi
from zeroconf import ServiceBrowser, Zeroconf
import socket
import sys
sys.path.append("/usr/local/lib")
from lcmtypes import pose_xyt_t
from lcmtypes import image_data_t
import lcm

HEADERSIZE = 8
PAYLOADSIZE = 8

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



lc = lcm.LCM()
dogpose = pose_xyt_t()
image_data = image_data_t()
mysocket = socket.socket()
mysocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# host = socket.gethostbyname(listener.info.name)
# hostname = socket.gethostname()
# address = socket.gethostbyname("%s.local" % hostname) 
# addr = address, port
# print(addr)
port = listener.info.port
host = socket.inet_ntoa(listener.info.addresses[0])
# if host == "127.0.1.1":
#     import commands
#     host = commands.getoutput("hostname -I")
# print "host = " + host

# #Prevent socket.error: [Errno 98] Address already in use
# mysocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# mysocket.bind((host, port))

# mysocket.listen(5)

# c, addr = mysocket.accept()
mysocket.connect((host,port))
print('gotti')
while True:
    # a = input("enter header:\t")
    # b = input("enterpayload:\t")

    # a = a.rjust(2,"0")
    # # print("new msg len:",msg[:HEADERSIZE])
    # b = b.encode("utf-16")
    # PAYLOADSIZE = len(b)
    # length = str(PAYLOADSIZE).rjust(2,"0")

    # # print(f"full message length: {msglen}")
    # msg = f"{a:0<{HEADERSIZE}}".encode()+f"{length:0<{PAYLOADSIZE}}".encode()
    # print(msg)
    # print(len(msg))
    # # mysocket.sendall(msg)
    # # mysocket.sendall(b)
    # # b'\x01\x00\x06\x00' +'🙉'.encode("utf-16")

    # mysocket.sendall(b'\x01\x00\x00\x00\x06\x00\x00\x00\xff\xfe=\xd8I\xde')

    try:
        data = mysocket.recv(1024)
        # data2 = mysocket.recv(1024)
        if data:


            # data = data.replace("\r\n", '') #remove new line character
            # inputStr = "Received " + data.decode() + ' ' + data2.decode() + " from ipad"
            inputStr = "Received " + data.decode() + ' ' + " from ipad"
            print(inputStr)
            # c.send("Hello from Raspberry Pi!\nYou sent: " + data + "\nfrom: " + addr[0] + "\n")
            if "person" in inputStr:
                dogpose.x = 1.0
                dogpose.y = 2.0
                dogpose.theta = pi
                lc.publish("POSE",dogpose.encode())
            # if data == "Quit": break

            if "ball" in inputStr:

    except:
        print('exception from receive attempt')

# c.send("Server stopped\n")
# print("server stopped")
# c.close()
