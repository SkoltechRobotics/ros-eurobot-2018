#!/usr/bin/env python  
import rospy
import tf
from std_msgs.msg import String

def parse(msg):
    return map(float, msg.data.split())

def broadcast_robot_tf(msg, robot_name):
    br = tf.TransformBroadcaster()
    coords = parse(msg)
    br.sendTransform((coords[0]/1000, coords[1]/1000, 0),
                     tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                     rospy.Time.now(),
                     robot_name,
                     "world")
    br.sendTransform((.0, .0, .40),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "%s_laser" % robot_name,
                     robot_name)

def broadcast_stm_tf(msg, robot_name):
    br = tf.TransformBroadcaster()
    coords = parse(msg)
    br.sendTransform((coords[0]/1000, coords[1]/1000, 0),
                     tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                     rospy.Time.now(),
                     "%s_stm" % robot_name,
                     "world")
    br.sendTransform((.0, .0, .40),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "%s_stm_laser" % robot_name,
                     "%s_stm" % robot_name)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    robot_name = rospy.get_param('robot_name')
    rospy.Subscriber("stm/coordinates",
                     String,
                     broadcast_stm_tf,
                     robot_name)
    rospy.Subscriber("particle_filter/coordinates",
                     String,
                     broadcast_robot_tf,
                     robot_name)
    rospy.spin()
