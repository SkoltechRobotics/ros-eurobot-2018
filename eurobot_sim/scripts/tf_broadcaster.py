#!/usr/bin/env python  
import rospy
import tf
from std_msgs.msg import String, Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
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
    # for visualizing LIDAR scan from stm_coords prospective
    br.sendTransform((.0, .0, .41),
                     tf.transformations.quaternion_from_euler(0, 0, 1.570796),
                     rospy.Time.now(),
                     "laser",
                     robot_name)

def broadcast_stm_tf(msg, robot_name):
    br = tf.TransformBroadcaster()
    coords = parse(msg)
    br.sendTransform((coords[0]/1000, coords[1]/1000, 0),
                     tf.transformations.quaternion_from_euler(0, 0, coords[2]),
                     rospy.Time.now(),
                     "%s_stm" % robot_name,
                     "world")
    # for visualizing particles
    br.sendTransform((.0, .0, .0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "particles",
                     "world")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    robot_name = rospy.get_param('robot_name')
    rospy.Subscriber("stm/coordinates",
                     String,
                     broadcast_stm_tf,
                     robot_name)
    rospy.Subscriber("kalman_filter/coordinates",
                     String,
                     broadcast_robot_tf,
                     robot_name)

    # cube colors
    yellow = [247, 181, 0]
    blue = [0, 124, 176]
    black = [14, 14, 16]
    green = [97, 153, 59]
    orange = [208, 93, 40]

    # initial position of cubes
    d = 0.058
    z = .029
    cubes = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "world"
    marker.ns = 'cubes'
    marker.id = 11
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = d
    marker.scale.y = d
    marker.scale.z = d
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = rospy.get_param("/field/cube1c_x") / 1000
    marker.pose.position.y = rospy.get_param("/field/cube1c_y") / 1000
    marker.pose.position.z = z
    #markers.append(Marker(header=header, type=1, action=0, pose=Pose(Point(0,0,0), Quaternion(w=1)), scale=Vector3(.058,.058,.058), color=ColorRGBA(1,0,0,.5), lifetime=rospy.Duration.from_sec(100)))
    #markers.append(Marker(header=header, type=1, action=0, pose=Pose(Point(x=.850, y=.540, z=.029), Quaternion(w=1)), scale=Vector3(.058,.058,.058), color=ColorRGBA(1,0,0,.5), lifetime=rospy.Duration.from_sec(1)))
    #points.append(Point(.850, .482, .029))
    #points.append(Point(.850, .598, .029))
    #points.append(Point(.792, .540, .029))
    #points.append(Point(.908, .540, .029))
    #cubes = MarkerArray(markers=markers)

    cubes.markers.append(marker)
    #pub_cubes = rospy.Publisher("/field/cubes", MarkerArray, queue_size=1)
    #pub_cubes.publish(cubes)
    rospy.spin()
