#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from TrackRegulator import TrackRegulator
import numpy as np

# global
c_p = np.array([0,0,0])


def coordinates_callback(data):
    data_splitted = str(data)[6:].split()
    global c_p = np.array([float(data_splitted[i]) for i in range(3)])

def command_callback(data):
    # parse name,type
    data_splitted = str(data)[6:].split()
    action_name = data_splitted[0]
    action_type = int(data_splitted[1])

    if action_type == 0xA1:
        # parse args
        args = data_splitted[2:]
        args = [float(args[i]) for i in range(3)]
        t_p = np.array(args)

        # start movement
        regulator.start_move(t_p, global c_p)
        rate.sleep()
        
        # regulation
        while regulator.is_moving: 
            global c_p = integrator.integrate(dpoint)
            speeds = regulator.regulate(global c_p)
            speeds = str(speeds[0]) + ' ' + str(speeds[1]) + ' ' + str(speeds[2])
            pub_response.publish("set_speed 0x06 " + speeds)
            rate.sleep()
        
        # publish response
        pub_response.publish(action_name + " done")

if __name__ == '__main__':
    try:
        regulator = TrackRegulator()
        rospy.init_node('track_regulator', anonymous=True)
        rate = rospy.Rate(100)
        rospy.Subscriber("robot_command", String, command_callback)
        rospy.Subscriber("coordinates", String, coordinates_callback)
        pub_response = rospy.Publisher("command_response", String, queue_size=10) 
        pub_command = rospy.Publisher("robot_command", String, queue_size=10) 

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

