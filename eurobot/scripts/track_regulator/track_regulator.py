#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from TrackRegulator import TrackRegulator
import numpy as np

c_p = np.array([0,0,0])


def coordinates_callback(data):
    data_splitted = data.data.split()
    global c_p
    c_p = np.array([float(data_splitted[i]) for i in range(3)])

def command_callback(data):
    global c_p
    # parse name,type
    data_splitted = data.data.split()
    cmd_id = data_splitted[0]
    action_type = data_splitted[1]

    if action_type == "MOVE":
        # parse args
        args = data_splitted[2:]
        args = [float(args[i]) for i in range(3)]
        t_p = np.array(args)

        # start movement
        regulator.start_move(t_p, c_p)
        rate.sleep()
        
        # regulation
        while regulator.is_moving:
            speeds = regulator.regulate(c_p)
            speeds = str(speeds[0]) + ' ' + str(speeds[1]) + ' ' + str(speeds[2])
            pub_command.publish("SETSPEED 8 " + speeds)
            rate.sleep()
        
        # publish response
        pub_response.publish(cmd_id + " DONE")

if __name__ == '__main__':
    try:
        regulator = TrackRegulator()
        rospy.init_node('track_regulator', anonymous=True)
        rate = rospy.Rate(100)
        rospy.Subscriber("move_command", String, command_callback)
        rospy.Subscriber("stm/coordinates", String, coordinates_callback)
        pub_response = rospy.Publisher("response", String, queue_size=10) 
        pub_command = rospy.Publisher("stm_command", String, queue_size=10) 

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

