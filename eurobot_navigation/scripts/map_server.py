#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import String

class MapServer():
    def __init__(self):
        rospy.init_node('map_server', anonymous=True)
        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.pub_response = rospy.Publisher("/main_robot/response", String, queue_size=10)
        rospy.Subscriber("map_server/cmd", String, self.cmd_callback, queue_size=1)
        s = rospy.Service('/static_map', GetMap, self.handle_get_map)

        self.size = (204, 304)
        self.border = 2
        self.FREE = 0
        self.OCCUPIED = 100

        self.field = np.zeros(self.size, dtype=np.int8)
        # borders
        self.field[:self.border, :] = self.OCCUPIED
        self.field[-self.border:, :] = self.OCCUPIED
        self.field[:,:self.border] = self.OCCUPIED
        self.field[:,-self.border:] = self.OCCUPIED
        # wastewater treatment plant
        self.field[-27:,92:212] = self.OCCUPIED
        # oponent's starting area
        self.field[:67,-42:] = self.OCCUPIED
        # oponent's buildinging area
        self.field[:20,-98:] = self.OCCUPIED
        # water towers
        self.field[82:90,:12] = self.OCCUPIED
        self.field[82:90,-12:] = self.OCCUPIED
        self.field[-12:,59:67] = self.OCCUPIED
        self.field[-12:,236:245] = self.OCCUPIED
        # cube mask
        self.cube = np.full((18,18), False, dtype=bool)
        self.cube[6:12,:] = True
        self.cube[:,6:12] = True

        # initial coords of cubes
        self.heaps = []
        # cubes in initial positions
        self.n_heaps = 6
        for n in range(self.n_heaps):
            x = int(rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 10) + 2
            y = int(rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 10) + 2
            self.heaps.append([x,y])
        self.heaps = np.array(self.heaps)

        for i in range(self.n_heaps):
            self.add_heap(i)

        # is the heap picked?
        self.picked = np.array([False, False, False, False, False, False])

        #rospy.Timer(rospy.Duration(0.1), self.pub_timer_callback)

        # create grid object
        self.grid = OccupancyGrid()
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "map"
        self.grid.info.map_load_time = rospy.Time.now()
        self.grid.info.resolution = 0.01
        self.grid.info.width = 304
        self.grid.info.height = 204
        self.grid.info.origin.position.x = -0.02
        self.grid.info.origin.position.y = -0.02
        self.grid.info.origin.orientation.w = 1

        self.grid.data = self.field.flatten()

        #rospy.sleep(1)
        # initial pub
        self.pub()


    def add_heap(self, n):
        x,y = self.heaps[n]
        self.field[y-9:y+9, x-9:x+9][self.cube] = self.OCCUPIED
        rospy.loginfo("Added heap number " + str(n) + " to the map.")


    def remove_heap(self, n):
        x,y = self.heaps[n]
        self.field[y-9:y+9, x-9:x+9][self.cube] = self.FREE
        rospy.loginfo("Removed heap number " + str(n) + " from the map.")


    def pub(self):
        self.pub_map.publish(self.grid)
        rospy.loginfo("Published the field map.")


    def cmd_callback(self, data):
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd = data_splitted[1]
        if cmd == "pub":
            self.pub()
        else:
            n = int(data_splitted[2])
            if cmd == "rm":
                self.remove_heap(n)
                self.grid.data = self.field.flatten()
                self.pub()
            elif cmd == "add":
                self.add_heap(n)
                self.grid.data = self.field.flatten()
                self.pub()
        print cmd_id + " finished"
        rospy.sleep(0.1) # TODO: delete
        self.pub_response.publish(cmd_id + " finished")


    def handle_get_map(req):
        rospy.loginfo("Sending map")
        return self.grid


if __name__ == '__main__':
    map_server = MapServer()
    # this solves some bug with move_base:
    for i in range(7):
        rospy.sleep(1)
        map_server.pub()
    rospy.spin()
