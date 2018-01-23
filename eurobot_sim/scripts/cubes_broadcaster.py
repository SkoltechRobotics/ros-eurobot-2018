#!/usr/bin/env python  
import rospy
from visualization_msgs.msg import Marker, MarkerArray

def show_callback(event):
    pub_cubes.publish(cubes)

if __name__ == '__main__':
    rospy.init_node('cubes_broadcaster')
    pub_cubes = rospy.Publisher("cubes", MarkerArray, queue_size=1)

    # cube colors [yellow, blue, black, green, orange]
    COLORS = [[247, 181, 0], [0, 124, 176], [14, 14, 16], [97, 153, 59], [208, 93, 40]]

    # size of cubes
    d = 0.058

    # initial coords of cubes
    coords = []

    # cubes in initial positions
    cubes = MarkerArray()
    n_groups = 6
    n_cubes_in_group = 5
    for n in range(n_groups):
        x = rospy.get_param("cube" + str(n+1) + "c_x") / 1000
        y = rospy.get_param("cube" + str(n+1) + "c_y") / 1000
        coords.append([[x,y],[x,y+d],[x,y-d],[x-d,y],[x+d,y]])
        for m in range(n_cubes_in_group):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.ns = 'cubes'
            marker.id = 10 * (n+1) + (m+1)
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = d
            marker.scale.y = d
            marker.scale.z = d
            marker.color.a = 1.0
            marker.color.r = COLORS[m][0] / 255.0
            marker.color.g = COLORS[m][1] / 255.0
            marker.color.b = COLORS[m][2] / 255.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = coords[n][m][0]
            marker.pose.position.y = coords[n][m][1]
            marker.pose.position.z = d / 2
            cubes.markers.append(marker)

    rospy.Timer(rospy.Duration(1), show_callback)
    rospy.spin()
