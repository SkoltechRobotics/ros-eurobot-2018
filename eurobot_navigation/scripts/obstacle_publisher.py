#!/usr/bin/env python
import numpy as np
import rospy
from costmap_converter.msg import ObstacleMsg, ObstacleArrayMsg
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Int16


class ObstaclesPublisher():
    def __init__(self):
        rospy.init_node('obstacles_publisher', anonymous=True)
        self.pub_obstacles = rospy.Publisher("/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=1)
        rospy.Subscriber("field/heap_picked", Int16, self.heap_picked_callback, queue_size=1)
        
        # initial coords of cubes
        d = 0.058
        self.heaps = []
        self.heap = np.array([[0,0,0],[-d,0,0],[0,d,0],[d,0,0],[0,-d,0]])

        # cubes in initial positions
        self.n_heaps = 6
        self.n_cubes_in_heap = 5
        for n in range(self.n_heaps):
            x = rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 1000
            y = rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 1000
            self.heaps.append(self.heap + [x,y,0])

        # wether a heap is picked
        self.picked = np.array([False, False, False, False, False, False])

        # field polygon
        self.field = ObstacleMsg()
        self.field.header.frame_id = "map"
        self.field.polygon.points = [Point32(0,0,0), Point32(3,0,0), Point32(3,2,0), Point32(0,2,0)]

        rospy.Timer(rospy.Duration(0.1), self.pub_timer_callback)

    def heaps2cubes(self):
        cubes = []
        for i in range(len(self.heaps)):
            if self.picked[i]:
                continue
            for j in range(self.n_cubes_in_heap):
                cube = ObstacleMsg()
                cube.header.stamp = rospy.Time.now()
                cube.header.frame_id = "map"
                cube.polygon.points = [Point32(*self.heaps[i][j])]
                cubes.append(cube)
        return cubes

    def heap_picked_callback(self, data):
        if data.data >= 0 and data.data < 6:
            self.picked[data.data] = True

    def pub_timer_callback(self, event):
        obstacle_msg = ObstacleArrayMsg() 
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = "map"
        
        obstacle_msg.obstacles = self.heaps2cubes()
        obstacle_msg.obstacles.append(self.field)

        self.pub_obstacles.publish(obstacle_msg)

if __name__ == '__main__':
    obstacles_publisher = ObstaclesPublisher()
    rospy.spin()
