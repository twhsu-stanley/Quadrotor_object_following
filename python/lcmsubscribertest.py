import sys
sys.path.append("/opt/homebrew/opt/lcm/lib")
import lcm
from lcmtypes import pose_xyt_t


def myhandler(channel,msg):
    stuff = pose_xyt_t.decode(msg)
    print(stuff.x, stuff.y, stuff.theta)

print('doing something with lcm subscriber')

lc = lcm.LCM()
lcm_sub = lc.subscribe("POSE",myhandler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass