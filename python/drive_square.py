#! /usr/bin/python
import lcm
from time import sleep
import sys
import math
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t


class WaypointFollower():
    def __init__(self):
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)

        # Choose from different waypoint combs
        self.wp_type = 2
        if self.wp_type == 0:
            print("Representative waypoints")
            self.waypoints = [[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,math.pi/2],[1.0,1.0,math.pi/2],[1.0,1.0,math.pi], \
            [-1.0,1.0,math.pi],[-1.0,1.0,-math.pi/2],[-1.0,-2.0,-math.pi/2],[-1.0,-2.0,0],[2.0,-2.0,0],[2.0,-2.0,math.pi/4], \
            [3.0,-1.0,math.pi/4],[3.0,-1.0,math.pi/2],[3.0,3.0,math.pi/2],[3.0,3.0,3*math.pi/4],[2.0,4.0,3*math.pi/4], 
            [2.0,4.0,math.pi],[-1.0,4.0,math.pi],[-1.0,4.0,-3*math.pi/4],[-3.0,2.0,-3*math.pi/4],[-3.0,2.0,-math.pi/2],
            [-3.0,-2.0,-math.pi/2],[-3.0,-2.0,0],[-2.0,-2.0,0.0],[-2.0,-2.0,math.pi/2],[-2.0,3.0,math.pi/2],[-2.0,3.0,0.0], \
            [-1.0,3.0,0.0],[-1.0,3.0,-math.pi/2],[-1.0,-2.0,-math.pi/2],[-1.0,-2.0,0.0],[0.0,-2.0,0.0],[0.0,-2.0,math.pi/2], [0.0,0.0,math.pi/2], [0.0,0.0,0.0]]
        elif self.wp_type == 1:
            print("Waypoints for Check point 1 submission") # made to line 3
            self.waypoints = [[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,math.pi/2],[1.0,1.0,math.pi/2],[1.0,1.0,math.pi], \
            [-1.0,1.0,math.pi],[-1.0,1.0,-math.pi/2],[-1.0,-2.0,-math.pi/2],[-1.0,-2.0,0],[2.0,-2.0,0],[2.0,-2.0,math.pi/4], \
            [3.0,-1.0,math.pi/4],[3.0,-1.0,math.pi/2],[3.0,3.0,math.pi/2],[3.0,3.0,3*math.pi/4],[2.0,4.0,3*math.pi/4], 
            [2.0,4.0,math.pi],[-1.0,4.0,math.pi],[-1.0,4.0,-3*math.pi/4],[-3.0,2.0,-3*math.pi/4],[-3.0,2.0,-math.pi/2],
            [-3.0,-2.0,-math.pi/2],[-3.0,-2.0,0],[-2.0,-2.0,0.0],[-2.0,-2.0,math.pi/2],[-2.0,3.0,math.pi/2],[-2.0,3.0,0.0], \
            [-1.0,3.0,0.0],[-1.0,3.0,-math.pi/2],[-1.0,-2.0,-math.pi/2],[-1.0,-2.0,0.0],[0.0,-2.0,0.0],[0.0,-2.0,math.pi], [0.0,0.0,math.pi]]
        elif self.wp_type == 2:
            print("Drive a square")
            path_len = 1.0
            self.waypoints = [[0.0,0.0,0.0],[path_len,0.0,0.0],[path_len,0.0,math.pi/2],[path_len,path_len,math.pi/2],[path_len,path_len,math.pi],\
            [0.0,path_len,math.pi],[0.0,path_len,-math.pi/2],[0.0,0.0,-math.pi/2],[0.0,0.0,0.0],[path_len,0.0,0.0],[path_len,0.0,math.pi/2],[path_len,path_len,math.pi/2],[path_len,path_len,math.pi],\
            [0.0,path_len,math.pi],[0.0,path_len,-math.pi/2],[0.0,0.0,-math.pi/2],[0.0,0.0,0.0],[path_len,0.0,0.0],[path_len,0.0,math.pi/2],[path_len,path_len,math.pi/2],[path_len,path_len,math.pi],\
            [0.0,path_len,math.pi],[0.0,path_len,-math.pi/2],[0.0,0.0,-math.pi/2],[0.0,0.0,0.0],[path_len,0.0,0.0],[path_len,0.0,math.pi/2],[path_len,path_len,math.pi/2],[path_len,path_len,math.pi],\
            [0.0,path_len,math.pi],[0.0,path_len,-math.pi/2],[0.0,0.0,-math.pi/2],[0.0,0.0,0.0]]
        else:
            print("Other testing code")
            # self.waypoints = [[0.0,0.0,0.0],[0.0,0.0,math.pi/2]]
            # self.waypoints = [[0.0,0.0,0.0],[0.0,0.0,math.pi/2],[0.5,0.0,math.pi/2]]

        # Get the stats about the waypoints
        self.wpt_len = len(self.waypoints) # num of waypoints
        self.wpt_num = 0 # Counting the current index of the waypoint

        # Initialize the vehicle current pose
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        # Threshold for determining waypoint switching
        self.wpt_thre_xy = 0.05 #0.1 # Threshold for the x and y positions
        self.wpt_thre_thata = 0.05 # Threshold for theta

        # Setup for the waypoint controller
        self.guidance_law = 2 # we implement 3 guidance law for striaght-line motion differs in turning velocity computation: 1) pure pursuit, 2) lecture slide, 3) LOS
        self.K_rho = 0.2 # gain for forward velocity
        self.K_alpha_SL = 0.5 #0.2 # gain for turning velocity (straight-line)
        self.K_alpha_T = 0.2 # gain for turning velocity (turning)
        self.K_beta = -0.5 # used only in 2) for turning velocity (straight-line)
        self.d_los = 0.2 #0.1 # LOS lookahead distance

    # Handling the odometry signal
    def odometry_handler(self, channel, data):
        msg = odometry_t.decode(data)
        self.current_x = msg.x + 0
        self.current_y = msg.y + 0
        self.current_theta = msg.theta + 0
        #print(self.current_x,self.current_y,self.current_theta)

    # Wrap angle to -pi to pi
    def wrap_to_pi(self,angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    # Compute the command based on the vehicle current state and publish the commands
    def motor_cmd_publish(self):
        # Initialize the command message
        msg = mbot_motor_command_t()
        msg.utime = 0.0
        # Check if we have reached the last waypoint
        if self.wpt_num >= self.wpt_len:
            print("Stoppping mode")
            msg.trans_v = 0
            msg.angular_v = 0
        else:
            # ==================================================================================================================================================== #
            # Since we always operate by move -> rotate -> move pattern, we can separate the opeartion mode into straight-line (move) and turning (rotate) modes
            # In the straight-line mode, we make sure the bot reach the desired x,y positions. In the turning mode, we make sure the bot reach the desired heading
            # ==================================================================================================================================================== #

            # Get the current and last waypoint
            Current_WP = self.waypoints[self.wpt_num]
            Past_WP = self.waypoints[self.wpt_num-1]
            print("Current WP:" ,Current_WP)
            #print("Last WP:" ,Past_WP)

            # Forward velocity computation
            alpha_to_WP = self.wrap_to_pi(-self.current_theta+math.atan2(Current_WP[1]-self.current_y,Current_WP[0]-self.current_x)) # difference between the current heading and the heading to the waypoint
            rho = math.sqrt((Current_WP[0]-self.current_x)**2+(Current_WP[1]-self.current_y)**2) # distance to the waypoint
            # Use the heading error to waypoint to determine if we have overshoot the waypoint
            # if alpha_to_WP < math.pi/2 and alpha_to_WP > -math.pi/2:
            #     msg.trans_v = self.K_rho*rho
            # else:
            #     msg.trans_v = self.K_rho*(-rho)

            # Turning velcoity computation
            if Current_WP[2] != Past_WP[2]:
                print("Turning mode")
                # Purely based on the heading error
                alpha = self.wrap_to_pi(-self.current_theta+Current_WP[2])
                msg.angular_v = self.K_alpha_T*alpha
                # Separate speed law
                if alpha_to_WP < math.pi/2 and alpha_to_WP > -math.pi/2:
                    msg.trans_v = self.K_rho*rho
                else:
                    msg.trans_v = self.K_rho*(-rho)
            else:
                print("Straight-line mode")
                if self.guidance_law == 0:
                    # Pure pursuit 
                    msg.angular_v = self.K_alpha_SL*alpha_to_WP 
                elif self.guidance_law == 1:
                    # Lecture slides
                    beta = self.wrap_to_pi(-self.current_theta - alpha_to_WP)
                    msg.angular_v = self.K_alpha_SL*alpha_to_WP + self.K_beta*beta
                elif self.guidance_law == 2:
                    # LOS
                    path_slope = math.atan2(Current_WP[1]-Past_WP[1],Current_WP[0]-Past_WP[0])
                    epsilon = -math.sin(path_slope)*(self.current_x-Past_WP[0]) + math.cos(path_slope)* (self.current_y-Past_WP[1]) # cross track error
                    alpha_los = self.wrap_to_pi(path_slope-math.atan2(epsilon,self.d_los)) # LOS angle
                    msg.angular_v = self.K_alpha_SL*self.wrap_to_pi(alpha_los-self.current_theta)
                else:
                    print("Not implemented")
                # Separate speed law
                msg.trans_v = self.K_rho*rho

        # Constrain the maximum fwd setpoint to avoid drift
        if msg.trans_v > 0.1:
           msg.trans_v = 0.1
         
        # Assign the setpoint
        print(msg.trans_v,msg.angular_v)
        self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())

    # Waypoint switching logic
    def switch_waypoint(self):
        Current_WP = self.waypoints[self.wpt_num]
        Past_WP = self.waypoints[self.wpt_num-1]
        if self.wpt_num < self.wpt_len:
            if Current_WP[2] == Past_WP[2]:
                # If straight-line mode, we only check the satisfaction of x and y positions
                if abs(Current_WP[0]-self.current_x) < self.wpt_thre_xy and \
                    abs(Current_WP[1]-self.current_y) < self.wpt_thre_xy:
                    self.wpt_num += 1
            else:
                # If turning mode, we only check the satisfaction of heading
                if abs(self.wrap_to_pi(Current_WP[2]-self.current_theta)) < self.wpt_thre_thata:
                    self.wpt_num += 1


# Initialize the wapoint follower
MBOT_WP = WaypointFollower()
# Loop through all the waypoints
while MBOT_WP.wpt_num < MBOT_WP.wpt_len:
    # Retrieve information from the odometry
    MBOT_WP.lc.handle()
    # Check if we have reached the current waypoint and need to move to the next waypoint
    MBOT_WP.switch_waypoint()
    # Pulish the commad for forward and truning velocity
    MBOT_WP.motor_cmd_publish()
    # (Potentially) need to pulish whether we have perform a waypoint switch



