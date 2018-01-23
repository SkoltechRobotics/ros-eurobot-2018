#!/usr/bin/env python  
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

def show_callback(event):
    pub_cubes.publish(cubes)

def coords_callback(data):
    global coords
    coords = np.array(map(float, data.data.split()))

def cube_index(heap_num, cube_num):
    return n_cubes_in_heap * heap_num + cube_num

def cube_marker(heap_num, cube_num):
    return cubes.markers[cube_index(heap_num, cube_num)]

def take_cube2(heap_num, cube_num, manipulator_num, floor):
    cube = cube_marker(heap_num, cube_num)
    cube.header.frame_id = "main_robot_stm"
    cube.pose.position.x = manipulator[manipulator_num][0]
    cube.pose.position.y = manipulator[manipulator_num][1]
    cube.pose.position.z = d * (.5 + floor)

def take_cube(manipulator_num):
    for h in range(n_heaps):
        for c in range(n_cubes_in_heap):
            p = cube_marker(h, c).pose.position
            if ((p.x-coords[0]/1000)**2 + (p.y-coords[1]/1000)**2) < (d/2)**2:
                take_cube2(h, c, manipulator_num, next_floor[manipulator_num])
                next_floor[manipulator_num] += 1
                return True
    return False

if __name__ == '__main__':
    rospy.init_node('cubes_broadcaster')
    pub_cubes = rospy.Publisher("cubes", MarkerArray, queue_size=1)
    coords = np.array([0,0,0])
    rospy.Subscriber("/main_robot/stm/coordinates", String, coords_callback, queue_size=1)

    # cube colors [yellow, blue, black, green, orange]
    COLORS = [[247, 181, 0], [0, 124, 176], [14, 14, 16], [97, 153, 59], [208, 93, 40]]

    # size of cubes
    d = 0.058

    # initial coords of cubes
    heap_coords = []

    # cubes in initial positions
    cubes = MarkerArray()
    n_heaps = 6
    n_cubes_in_heap = 5
    for n in range(n_heaps):
        x = rospy.get_param("cube" + str(n+1) + "c_x") / 1000
        y = rospy.get_param("cube" + str(n+1) + "c_y") / 1000
        heap_coords.append([[x,y],[x-d,y],[x,y+d],[x+d,y],[x,y-d]])
        for m in range(n_cubes_in_heap):
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
            marker.pose.position.x = heap_coords[n][m][0]
            marker.pose.position.y = heap_coords[n][m][1]
            marker.pose.position.z = d / 2
            cubes.markers.append(marker)

    # params of manipulators
    l = 0.005
    manipulator = [[d-l,d+l],[0,0],[d+l,d+l]]
    next_floor = [1,1,1]

    rospy.Timer(rospy.Duration(.03), show_callback)
    rospy.spin()
