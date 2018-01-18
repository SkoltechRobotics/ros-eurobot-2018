#!/usr/bin/env python  
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

def parse(msg):
    return map(float, msg.data.split())

def broadcast_robot_tf(msg, robot_name):
    br = tf.TransformBroadcaster()
    coords = parse(msg)
    br.sendTransform((coords[0]/1000, coords[1]/1000, 0),
                     tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                     rospy.Time.now(),
                     robot_name,
                     "table")
    br.sendTransform((.0, .0, .40),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "%s_lidar" % robot_name,
                     robot_name)

def broadcast_stm_tf(msg, robot_name):
    br = tf.TransformBroadcaster()
    coords = parse(msg)
    br.sendTransform((coords[0]/1000, coords[1]/1000, 0),
                     tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                     rospy.Time.now(),
                     "%s_stm" % robot_name,
                     "table")
    br.sendTransform((.0, .0, .40),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "%s_stm_lidar" % robot_name,
                     "%s_stm" % robot_name)
    # for visualizing LIDAR scan from stm_coords prospective
    br.sendTransform((.0, .0, .0),
                     tf.transformations.quaternion_from_euler(0, 0, 1.570796),
                     rospy.Time.now(),
                     "laser",
                     "%s_stm_lidar" % robot_name)
    # for visualizing particles
    br.sendTransform((.0, .0, .0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "particles",
                     "world")
    # TBD transfer to another node
    # pub cubes

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

    # initial position of cubes
    #points = [Point(x=x, y=y, z=.029) for i in range(len(0))]
    #header = Header(frame_id="table")
    #cubes = PointCloud(header=header, points=points)
    #pub_cubes = rospy.Publisher("cubes", PintCloud, queue_size=1)
    rospy.spin()
