#!/usr/bin/env python
# ROS python API
import rospy
import math
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Global parameters
x_cor = 10
y_cor = 0
yaw_in_radians = 0
yaw_rate = 0
flag = 0

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def _init_(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set."%e)

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        self.sv = Twist()
        
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask =  PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                             + PositionTarget.IGNORE_YAW_RATE
        
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 1
        
        # update the setpoint message with the required altitude
        self.sp.velocity.z = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point()

        # initial values for setpoints
        self.sp.velocity.x = 0.0
        self.sp.velocity.y = 0.0
        
        self.sv.linear.x = 2

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        global x_cor,y_cor,yaw_rate,flag

        self.sv.linear.x = 2


# Main function
def main():

    # initiate node
    rospy.init_node('square', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    sv_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)


    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateSp()
        sv_pub.publish(cnt.sv)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
