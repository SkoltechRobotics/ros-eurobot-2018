#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from TrackRegulator import TrackRegulator
import numpy as np

c_p = np.array([0,0,0])
cmd_id = None

def coordinates_callback(data):
    data_splitted = data.data.split()
    global c_p
    c_p = np.array([float(data_splitted[i]) for i in range(3)])

def command_callback(data):
    global c_p
    global cmd_id
    global regulator
    global pub_command
    # parse name,type
    data_splitted = data.data.split()
    cmd_id = data_splitted[0]
    action_type = data_splitted[1]
    print("Receive command " + data.data) 
    if action_type == "MOVE":
        # parse args
        args = data_splitted[2:]
        args = [float(args[i]) for i in range(3)]
        t_p = np.array(args)

        # start movement
        regulator.start_move(t_p, c_p) 
    elif action_type == "STOP":
        regulator.is_moving = False
        pub_command.publish("SETSPEED 8 0 0 0")
    elif action_type == "RETURN":
        x = rospy.get_param("/main_robot/start_x")
        y = rospy.get_param("/main_robot/start_y")
        a = rospy.get_param("/main_robot/start_a")
        t_p = np.array([x, y, a])
        regulator.start_move(t_p, c_p)
    elif action_type == "MOVECUBE":
        args = data_splitted[2:]
        n_cube = int(args[0])
        x = rospy.get_param("/field/cube" + str(n_cube) + "c_x")
        y = rospy.get_param("/field/cube" + str(n_cube) + "c_y")
        t_p = np.array([x, y, 0])
        regulator.start_move(t_p, c_p)
        

if __name__ == '__main__':
    try:
        regulator = TrackRegulator()
        rospy.init_node('track_regulator', anonymous=True)
        rate = rospy.Rate(10)
        pub_command = rospy.Publisher("stm_command", String, queue_size=10) 
        pub_response = rospy.Publisher("response", String, queue_size=10) 
        pub_speed = rospy.Publisher("track_regulator/speed", String, queue_size=1) 
        rospy.Subscriber("move_command", String, command_callback)
        rospy.Subscriber("stm/coordinates", String, coordinates_callback)

        while not rospy.is_shutdown():
            if not cmd_id is None:
                # regulation
                while regulator.is_moving:
                    speeds = regulator.regulate(c_p)
                    print("speeds", speeds)
                    speeds = str(speeds[0]) + ' ' + str(speeds[1]) + ' ' + str(speeds[2])
                    pub_command.publish("SETSPEED 8 " + speeds)
                    pub_speed.publish(str(speeds))
                    rate.sleep()
                # publish response
                pub_response.publish(cmd_id + " finished")
                cmd_id = None
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

