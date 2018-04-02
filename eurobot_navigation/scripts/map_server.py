#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import String
import tf
from people_msgs.msg import People, Person


class MapServer():
    def __init__(self):
        rospy.init_node('map_server', anonymous=True)
        self.pub_main_map = rospy.Publisher("/main_robot/map", OccupancyGrid, queue_size=1)
        self.pub_secondary_map = rospy.Publisher("/secondary_robot/map", OccupancyGrid, queue_size=1)
        self.pub_response = rospy.Publisher("/main_robot/response", String, queue_size=10)
        self.pub_social_main = rospy.Publisher("/main_robot/people", People, queue_size=10)
        self.pub_social_secondary = rospy.Publisher("/secondary_robot/people", People, queue_size=10)
        rospy.Subscriber("/map_server/cmd", String, self.cmd_callback, queue_size=1)
        s = rospy.Service('/static_map', GetMap, self.handle_get_map)
        self.listener = tf.TransformListener()

        # grid cell size in meters
        self.resolution = 0.01

        # get sizes of our robots
        self.size_main = np.array([rospy.get_param('/main_robot/dim_x'), rospy.get_param('/main_robot/dim_y')]) / self.resolution / 1000
        self.radius_main = rospy.get_param('/main_robot/dim_r')
        self.size_secondary = np.array([rospy.get_param('/secondary_robot/dim_x'), rospy.get_param('/secondary_robot/dim_y')]) / self.resolution / 1000
        self.radius_secondary = rospy.get_param('/secondary_robot/dim_r')

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


        # this solves some bug with move_base:
        rospy.Timer(rospy.Duration(.1), self.timer_callback)


    def add_heap(self, n):
        x,y = self.heaps[n]
        self.field[y-9:y+9, x-9:x+9][self.cube] = self.OCCUPIED
        rospy.loginfo("Added heap number " + str(n) + " to the map.")


    def remove_heap(self, n):
        x,y = self.heaps[n]
        self.field[y-9:y+9, x-9:x+9][self.cube] = self.FREE
        rospy.loginfo("Removed heap number " + str(n) + " from the map.")


    def pub(self):
        #self.pub_map.publish(self.grid)
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
        rospy.sleep(0.1) # TODO
        self.pub_response.publish(cmd_id + " finished")


    def handle_get_map(self, req):
        rospy.loginfo("Sending map")
        return self.grid


    def robot(self, size, coords):
        # 'occupy' all cells
        robot = np.full(self.size, True, dtype='bool')

        x, y = np.meshgrid(np.arange(0, self.size[1]), np.arange(0,self.size[0]))
        
        # upper point
        x1 = coords[0] / self.resolution - size[1] / 2 * np.sin(coords[2])
        y1 = coords[1] / self.resolution + size[1] / 2 * np.cos(coords[2])

        # lower point
        x2 = coords[0] / self.resolution + size[1] / 2 * np.sin(coords[2])
        y2 = coords[1] / self.resolution - size[1] / 2 * np.cos(coords[2])

        # left point
        x3 = coords[0] / self.resolution - size[0] / 2 * np.cos(coords[2])
        y3 = coords[1] / self.resolution - size[0] / 2 * np.sin(coords[2])

        # right point
        x4 = coords[0] / self.resolution + size[0] / 2 * np.cos(coords[2])
        y4 = coords[1] / self.resolution + size[0] / 2 * np.sin(coords[2])

        # 'free' cells outside of each side of the robot
        a = coords[2] % (2 * np.pi)
        if a < np.pi / 2 or a > 3 * np.pi / 2:
            robot[y - y1 > (x - x1) * np.tan(coords[2])] = False
            robot[y - y2 < (x - x2) * np.tan(coords[2])] = False
            robot[y - y3 > (x - x3) * np.tan(np.pi/2 + coords[2])] = False
            robot[y - y4 < (x - x4) * np.tan(np.pi/2 + coords[2])] = False
        else:
            robot[y - y1 < (x - x1) * np.tan(coords[2])] = False
            robot[y - y2 > (x - x2) * np.tan(coords[2])] = False
            robot[y - y3 > (x - x3) * np.tan(np.pi/2 + coords[2])] = False
            robot[y - y4 < (x - x4) * np.tan(np.pi/2 + coords[2])] = False

        return robot


    def timer_callback(self, event):
        try:
            # get secondary robot coords
            (trans,rot) = self.listener.lookupTransform('/map', '/secondary_robot', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            coords_secondary = np.array([trans[0], trans[1], yaw])
            
            # get main robot coords
            (trans,rot) = self.listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            coords_main = np.array([trans[0], trans[1], yaw])

            # this copies will be processed and sent
            field_main = self.field.copy()
            field_secondary = self.field.copy()
            
            # put the other robot on the map of each robot
            #field_main[self.robot(self.size_secondary, coords_secondary)] = self.OCCUPIED
            #field_secondary[self.robot(self.size_main, coords_main)] = self.OCCUPIED

            # publish both maps
            self.grid.data = field_secondary.flatten()
            self.pub_secondary_map.publish(self.grid)
            self.grid.data = field_main.flatten()
            self.pub_main_map.publish(self.grid)

            # publish robots for the social costmap layer
            people = People()
            people.header.frame_id = '/map'
            people.header.stamp = rospy.Time.now()
     
            main_rob = Person()
            main_rob.name = 'main_robot'
            main_rob.position.x = coords_main[0]
            main_rob.position.y = coords_main[1]
            main_rob.reliability = 1

            sec_rob = Person()
            sec_rob.name = 'secondary_robot'
            sec_rob.position.x = coords_secondary[0]
            sec_rob.position.y = coords_secondary[1]
            sec_rob.reliability = 1

            people.people = [sec_rob]
            #self.pub_social_main.publish(people)
            
            people.people = [main_rob]
            #self.pub_social_secondary.publish(people)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # pub maps without robots in case of tf listener failure
            self.grid.data = self.field.flatten()
            self.pub_main_map.publish(self.grid)
            self.pub_secondary_map.publish(self.grid)
            pass

if __name__ == '__main__':
    map_server = MapServer()
    rospy.spin()
