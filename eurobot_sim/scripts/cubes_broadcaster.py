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


def response_callback(manipulator_num, action_id):
    def response_callback_(event):
        take_cube(manipulator_num)
        pub_response.publish(action_id + ' finished')

    return response_callback_


def command_callback(data):
    # parse data
    data_splitted = data.data.split()
    action_id = data_splitted[0]
    action_type = int(data_splitted[1])

    if action_type == 0xb0:
        manipulator_num = int(data_splitted[2])
        rospy.Timer(rospy.Duration(1), response_callback(manipulator_num, action_id), oneshot=True)


def cube_index(heap_num, cube_num):
    return n_cubes_in_heap * heap_num + cube_num


def cube_marker(heap_num, cube_num):
    return cubes.markers[cube_index(heap_num, cube_num)]


def take_cube2(heap_num, cube_num, manipulator_num):
    # lift all cubes in this manipulator
    for h in range(n_heaps):
        for c in range(n_cubes_in_heap):
            if where_cube[h][c] == manipulator_num:
                cube_marker(h, c).pose.position.z += d
    # grab a new cube
    cube = cube_marker(heap_num, cube_num)
    cube.header.frame_id = "main_robot_stm"
    cube.pose.position.x = manipulator[manipulator_num][0]
    cube.pose.position.y = manipulator[manipulator_num][1]
    cube.pose.position.z = d * 1.5


def take_cube(manipulator_num):
    c = coords.T.copy()
    c[:2] /= 1000
    M = np.array([[np.cos(c[2]), -np.sin(c[2])], [np.sin(c[2]), np.cos(c[2])]])
    manipulator_coords = np.matmul(M, manipulator[manipulator_num].T) + c[:2]
    for h in range(n_heaps):
        for c in range(n_cubes_in_heap):
            p = cube_marker(h, c).pose.position
            if p.z < d and ((p.x - manipulator_coords[0]) ** 2 + (p.y - manipulator_coords[1]) ** 2) < (d / 2) ** 2:
                # print 'cube', c, 'in heap', h, 'found'
                take_cube2(h, c, manipulator_num)
                where_cube[h][c] = manipulator_num
                return True
    return False


if __name__ == '__main__':
    rospy.init_node('cubes_broadcaster')
    pub_cubes = rospy.Publisher("cubes", MarkerArray, queue_size=1)
    pub_response = rospy.Publisher("/main_robot/response", String, queue_size=10)
    coords = np.array([0, 0, 0])
    rospy.Subscriber("/main_robot/coordinates", String, coords_callback, queue_size=1)
    rospy.Subscriber("/main_robot/stm_command", String, command_callback, queue_size=10)

    # cube colors [yellow, green, blue, orange, black]
    COLORS = [[247, 181, 0], [97, 153, 59], [0, 124, 176], [208, 93, 40], [14, 14, 16]]

    # size of cubes
    d = 0.058

    # initial coords of cubes
    heap_coords = []

    # cubes in initial positions
    cubes = MarkerArray()
    n_heaps = 6
    n_cubes_in_heap = 5
    for n in range(n_heaps):
        x = rospy.get_param("cube" + str(n + 1) + "c_x") / 1000
        y = rospy.get_param("cube" + str(n + 1) + "c_y") / 1000
        heap_coords.append([[x, y], [x - d, y], [x, y + d], [x + d, y], [x, y - d]])
        for m in range(n_cubes_in_heap):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.ns = 'cubes'
            marker.id = 10 * (n + 1) + (m + 1)
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = d
            marker.scale.y = d
            marker.scale.z = d
            marker.color.a = 1.0
            color = COLORS[m]
            if n >= 3:
                if m == 1:
                    color = COLORS[3]
                elif m == 3:
                    color = COLORS[1]
            marker.color.r = color[0] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[2] / 255.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = heap_coords[n][m][0]
            marker.pose.position.y = heap_coords[n][m][1]
            marker.pose.position.z = d / 2
            cubes.markers.append(marker)

    # params of manipulators
    manipulator = np.array([[d, d], [0, 0], [-d, d]])
    where_cube = np.ones((n_heaps, n_cubes_in_heap)) * (-1)

    rospy.Timer(rospy.Duration(.03), show_callback)
    rospy.spin()
