#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
import tf
from people_msgs.msg import People, Person
from copy import deepcopy


class MapServer():
    def __init__(self):
        rospy.init_node('map_server', anonymous=True)

        # grid cell size in meters
        self.resolution = 0.01

        # get sizes of our robots
        self.size_main = np.array([rospy.get_param('/main_robot/dim_x'), rospy.get_param('/main_robot/dim_y')]) / self.resolution / 1000
        self.radius_main = rospy.get_param('/main_robot/dim_r')
        self.size_secondary = np.array([rospy.get_param('/secondary_robot/dim_x'), rospy.get_param('/secondary_robot/dim_y')]) / self.resolution / 1000
        self.radius_secondary = rospy.get_param('/secondary_robot/dim_r')

        self.coords_main = np.array([rospy.get_param('/main_robot/start_x') / 1000.0, rospy.get_param('/main_robot/start_y') / 1000.0, rospy.get_param('/main_robot/start_a')])
        self.coords_secondary = np.array([rospy.get_param('/secondary_robot/start_x') / 1000.0, rospy.get_param('/secondary_robot/start_y') / 1000.0, rospy.get_param('/secondary_robot/start_a')])
        self.ROBOT_R = 0.2
        # detected robots close to field walls alogn axis X will not count (those are probably our beacons
        self.WALL_DIST_X = 0.05
        self.SEPARATE_ROBOT_DISTANCE = 0.2
        self.robots = []
        self.robots_upd_time = rospy.Time.now()

        self.size = (204, 304)
        self.size_in_meters = (3, 2)
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

        self.grid_main = deepcopy(self.grid)
        self.grid_secondary = deepcopy(self.grid)

        self.pub_main_map = rospy.Publisher("/main_robot/map", OccupancyGrid, queue_size=1)
        self.pub_secondary_map = rospy.Publisher("/secondary_robot/map", OccupancyGrid, queue_size=1)
        self.pub_social_main = rospy.Publisher("/main_robot/people", People, queue_size=10)
        self.pub_response_main_robot = rospy.Publisher("/main_robot/response", String, queue_size=1)
        self.pub_social_secondary = rospy.Publisher("/secondary_robot/people", People, queue_size=10)
        self.pub_opponent_robots = rospy.Publisher("/map_server/opponent_robots", PointCloud, queue_size=10)
        rospy.Subscriber("/map_server/cmd", String, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/spy/detected_robots", PointCloud, self.detected_robots_callback, queue_size=1)
        self.service_main = rospy.Service('/main_robot/static_map', GetMap, self.handle_get_map_main)
        self.service_secondary = rospy.Service('/secondary_robot/static_map', GetMap, self.handle_get_map_secondary)
        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(1. / 100), self.timer_callback)
        

    def add_heap(self, n):
        x,y = self.heaps[n]
        self.field[y-9:y+9, x-9:x+9][self.cube] = self.OCCUPIED
        rospy.loginfo("Added heap number " + str(n) + " to the map.")


    def remove_heap(self, n):
        x,y = self.heaps[n]
        self.field[y-9:y+9, x-9:x+9][self.cube] = self.FREE
        rospy.loginfo("Removed heap number " + str(n) + " from the map.")


    def pub(self):
        self.pub_main_map.publish(self.grid_main)
        self.pub_secondary_map.publish(self.grid_secondary)
        #rospy.loginfo("Published the field map.")


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
        rospy.Timer(rospy.Duration(0.05), lambda e: self.pub_response_main_robot.publish(cmd_id + " finished"), oneshot=True)


    def handle_get_map_main(self, req):
        rospy.loginfo("Sending map (for main robot) via service")
        return self.grid_main


    def handle_get_map_secondary(self, req):
        rospy.loginfo("Sending map (for secondary robot) via service")
        return self.grid_secondary


    def opponent_robots(self):
        mask = np.full(self.size, False, dtype='bool')
        if (rospy.Time.now() - self.robots_upd_time).secs < 1:
            x,y = np.meshgrid(np.arange(0, self.size[1]), np.arange(0, self.size[0]))
            x = (x - self.border) * self.resolution
            y = (y - self.border) * self.resolution
            xy = np.concatenate((x[:,:,np.newaxis], y[:,:,np.newaxis]), axis=2)
            for robot in self.robots:
                mask[np.linalg.norm((xy - robot), axis=2) <= self.ROBOT_R] = True
        return mask


    def our_robot(self, size, coords):
        # 'occupy' all cells
        robot = np.full(self.size, True, dtype='bool')

        x, y = np.meshgrid(np.arange(0, self.size[1]), np.arange(0, self.size[0]))
        
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
        else:
            robot[y - y1 < (x - x1) * np.tan(coords[2])] = False
            robot[y - y2 > (x - x2) * np.tan(coords[2])] = False
        if a < np.pi:
            robot[y - y3 < (x - x3) * np.tan(np.pi/2 + coords[2])] = False
            robot[y - y4 > (x - x4) * np.tan(np.pi/2 + coords[2])] = False
        else:
            robot[y - y3 > (x - x3) * np.tan(np.pi/2 + coords[2])] = False
            robot[y - y4 < (x - x4) * np.tan(np.pi/2 + coords[2])] = False

        return robot


    def timer_callback(self, event): 
        # this copies of obstacle map will be processed and sent
        field_main = self.field.copy()
        # put opponent robots on the map
        field_main[self.opponent_robots()] = self.OCCUPIED
        field_secondary = field_main.copy()

        # put our robots on the maps
        try:
            # get secondary robot coords
            (trans,rot) = self.listener.lookupTransform('/map', '/secondary_robot', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            self.coords_secondary = np.array([trans[0], trans[1], yaw])
            
            # get main robot coords
            (trans,rot) = self.listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            self.coords_main = np.array([trans[0], trans[1], yaw])

            # put the other robot on the map of each robot
            field_main[self.our_robot(self.size_secondary, self.coords_secondary)] = self.OCCUPIED
            field_secondary[self.our_robot(self.size_main, self.coords_main)] = self.OCCUPIED

            # publish robots for the social costmap layer
            #people = People()
            #people.header.frame_id = '/map'
            #people.header.stamp = rospy.Time.now()
     
            #main_rob = Person()
            #main_rob.name = 'main_robot'
            #main_rob.position.x = coords_main[0]
            #main_rob.position.y = coords_main[1]
            #main_rob.reliability = 1

            #sec_rob = Person()
            #sec_rob.name = 'secondary_robot'
            #sec_rob.position.x = coords_secondary[0]
            #sec_rob.position.y = coords_secondary[1]
            #sec_rob.reliability = 1

            #people.people = [sec_rob]
            #self.pub_social_main.publish(people)
            
            #people.people = [main_rob]
            #self.pub_social_secondary.publish(people)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #rospy.loginfo("map_server failed to get TF of one or two robot")
            pass
        
        # publish both maps
        self.grid_main.data = field_main.flatten()
        self.grid_secondary.data = field_secondary.flatten()
        self.pub()

    def detected_robots_callback(self, data):
        robots = np.array([[robot.x, robot.y] for robot in data.points])
        if robots.shape[0] != 0:
            # exclude our robots
            ind = np.where(np.logical_and(np.linalg.norm(robots - self.coords_main[:2], axis=1) > self.SEPARATE_ROBOT_DISTANCE, np.linalg.norm(robots - self.coords_secondary[:2], axis=1) > self.SEPARATE_ROBOT_DISTANCE))
            robots = robots[ind]
            # exclude objects outside the field (beacons, ets.)
            ind = np.where(np.logical_and(np.logical_and(robots[:,0] > self.WALL_DIST_X, robots[:,0] < self.size_in_meters[0] - self.WALL_DIST_X), np.logical_and(robots[:,1] > 0, robots[:,1] < self.size_in_meters[1])))
            robots = robots[ind]
        self.robots = robots
        self.robots_upd_time = data.header.stamp 

        # pub opponent robots
        array = PointCloud()
        array.header.frame_id = "map"
        array.header.stamp = rospy.Time.now()
        array.points = [Point(x=robot[0], y=robot[1]) for robot in robots]
        self.pub_opponent_robots.publish(array)

if __name__ == '__main__':
    map_server = MapServer()
    rospy.spin()
