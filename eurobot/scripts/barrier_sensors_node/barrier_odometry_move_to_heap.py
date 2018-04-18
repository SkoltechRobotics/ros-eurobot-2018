#!/usr/bin/env python
import rospy
import numpy as np
import serial
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
import tf

#
# comment on how this piece of shit works:
#     two group of cases:
#     1) [0,2,3] - when we use rf which control cubes from opposite sides: (examples)
#           |[]             |[]
#         |[][][]|     or    [][]|
#     2) [1,4,5,6,7,8,9] when all our rf by one side of cubes:
#         |[]
#       |[][] (example)
#
#     solutions:
#     1) for y: same as 2
#        for x: if we have symmetric rf data, then do nothing!
#               if not symmetric rf data ([1,0] or [0,1]), then move to the other side until change or until symmetric!
#     2) for x, y one need to first go until trigger some 1 rf, then back to 0



class OdometryRFChangeOperator:
    def __init__(self, bn, mask):
        self.start_sensors = bn.sensors
        self.last_sensors = bn.sensors
        self.started_coords = None
        self.timer = None
        self.mask = mask
        self.bn = bn
        self.listener = tf.TransformListener()
        self.changes = []
        self.check_period = 0.005
        self.last_coords = None
        self.big_mask = np.array(mask.tolist()*2)
        self.CMD_NAME = "ODOM_MEASURE"
        self.i = 0
        self.odom = {}

        def cb(msg):
            data_splitted = msg.data.split()
            name = data_splitted[0]
            status = data_splitted[1]
            if name in self.odom and not 'odom' in self.odom[name]:
                rospy.loginfo(msg.data)
                self.odom[name]["odom"] = [float(v) for v in data_splitted[2:]]
                

        self.wait_sub = rospy.Subscriber(self.bn.response_pub_name, String, cb)
        self.get_odom_trans()
        


    def get_odom_trans(self):
        #(trans, rot) = self.bn.listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
        self.i += 1
        name = self.CMD_NAME + str(self.i)
        self.odom[name] = {}
        self.odom[name]['sensors'] = self.bn.sensors
        self.bn.command_pub.publish(name + ' ' + str(0x0A))
        while not 'odom' in self.odom[name]:
            rospy.sleep(0.001)
        self.last_coords = self.odom[name]['odom']
        #self.last_coords = self.listener.lookupTransform('/map', '/main_robot_odom', rospy.Time(0))[0]
        rospy.loginfo(self.last_coords)
        return self.last_coords

    def odometry_callback(self,e):
        if np.any(self.last_sensors * self.big_mask != bn.sensors * self.big_mask):
            self.changes.append((bn.sensors, rospy.get_time(), self.get_odom_trans()))
            self.last_sensors = bn.sensors
            rospy.loginfo('AAAAAa')
            rospy.loginfo(self.changes)

    def find_changes(self, odometry_movement):
        rospy.loginfo('find_changes')
        self.changes = []
        self.started_coords = self.get_odom_trans()
        self.timer = rospy.Timer(rospy.Duration(self.check_period), self.odometry_callback)
        self.bn.wait_for_movement(*odometry_movement)
        self.timer.shutdown()
        return self.changes



class BarrierNavigator():
    masks = {
            0: np.array([1, 1, 1]),
            1: np.array([1, 1, 0]),
            2: np.array([1, 0, 1]),
            3: np.array([0, 1, 1]),
            7: np.array([1, 0, 0]),
            8: np.array([0, 1, 0]),
            9: np.array([0, 0, 1])
            }
    shifts = { #by color, mm
        'x': {
        0: 0.0008,
        1: 0.0008,
        2: 0.0008,
        3: 0.0008,
        4: 0.0008
        },
        'y': {
        0: 0.0008,
        1: 0.0008,
        2: 0.0008,
        3: 0.0008,
        4: 0.0008
        }
    }
    final_shifts = {
        'x' : {
            7 : {
                0: 0,
                1: -0.003,
                2: -0.005,
                3: -0.003,
                4: -0.003,
            },
            8: {
                0: -0.003,
                1: -0.0015,
                2: -0.004,
                3: -0.002,
                4: -0.002,
            },
            9: {
                0: 0,
                1: 0.002,
                2: 0.003,
                3: +0.003,
                4: +0.003,
            }
        },
        'y': {
            7: {
                0: -0.002,
                1: 0,
                2: 0,
                3: 0,
                4: 0,
            },
            8: {
                0: -0.003,
                1: 0.005,
                2: 0.005,
                3: 0.005,
                4: 0.005,
            },
            9: {
                0: -0.002,
                1: +0.0035,
                2: 0,
                3: 0,
                4: +0.0035,
            }
        }
    }
    def __init__(self, stm_command_publisher_name, response_pub_name, corrected_rf_data_pub_name, unstable_rf_pub_name):
        self.sensors = []
        self.phase = 0
        self.sensors_goals = []
        self.dx_quant = 0.003
        self.dy_quant = 0.003
        self.dx_finish = 0.002
        self.dy_finish = 0.002
        self.divider = 3
        self.tolerance = 0.0007
        self.min_quant = 0.0008
        self.speed_xy = 0.2
        self.speed_z = 0.1
        self.command_name = str(0xa2)
        self.ready = True
        self.command_pub_name = stm_command_publisher_name
        self.command_pub = rospy.Publisher(stm_command_publisher_name, String, queue_size=10)
        self.response_pub_name = response_pub_name
        self.corrected_rf_data = rospy.Publisher(corrected_rf_data_pub_name, Int32MultiArray, queue_size=10)
        self.stability_rf_data = rospy.Publisher(unstable_rf_pub_name, Int32MultiArray, queue_size=10)
        self.response_pub = rospy.Publisher(response_pub_name, String, queue_size=2)
        self.sensors_queue = np.zeros((5, 6), dtype=np.int)
        self.listener = tf.TransformListener()
        self.i = 0
        self.started_sensors = None
        self.CMD_NAME = "MOVEODOM"
        self.rf_broken_flag = False
        self.has_moved = {}

        def cb(msg):
            data_splitted = msg.data.split()
            name = data_splitted[0]
            status = data_splitted[1]
            if name in self.has_moved:
                self.has_moved[name] = status

        self.wait_sub = rospy.Subscriber(self.response_pub_name, String, cb)

    def barrier_sensors_callback(self):
        def cb(data):
            sd = np.array(data.data)

            rf_nav_status = sd[-6:]
            if not np.any(rf_nav_status):
                self.rf_broken_flag = False
                self.sensors_queue = np.roll(self.sensors_queue, -1, axis=0)
                self.sensors_queue[0] = sd[10:16]
            else:
                self.rf_broken_flag = True
            # self.sensors_queue = np.roll(self.sensors_queue, -1, axis=0)
            # self.sensors_queue[0] = sd
            # print(self.sensors_queue)
            self.sensors = np.sum(self.sensors_queue, axis=0) >= 3
            self.sensors_unstable = np.logical_and(np.sum(self.sensors_queue, axis=0) > 1,(np.sum(self.sensors_queue, axis=0) < 4))
            # print(self.sensors)
            # rospy.loginfo(self.sensors_queue)
            # rospy.loginfo(self.sensors)
            self.corrected_rf_data.publish(data=self.sensors)
            self.stability_rf_data.publish(data=self.sensors_unstable)

        return cb




    def move_cycle_odom(self, mask = np.array([1,1,1])):
        Y_finished = False
        X_finished = False
        self.started_sensors = self.sensors
        rospy.loginfo("started from " + str(self.sensors))
        rospy.loginfo(mask)
        started_x = np.sum(self.sensors[3:] * mask)
        started_y = np.sum(self.sensors[:3] * mask)
        reversed_x = mask[2]
        X_touched = False
        Y_touched = False
        rospy.loginfo("started from : x %d y %d" % (started_x, started_y))
        oo = OdometryRFChangeOperator(self, mask)
        # while not rospy.is_shutdown():

        if self.rf_broken_flag:
            rospy.sleep(0.05)
            # continue
        s_x = np.sum(self.sensors[3:] * mask)
        s_y = np.sum(self.sensors[:3] * mask)
        
        
    
        
    def move_cycle_one_new(self, mask, case, color):
        Y_finished = False
        X_finished = False
        self.started_sensors = self.sensors
        rospy.loginfo("started from " + str(self.sensors))
        started_x = np.sum(self.sensors[3:]*mask)
        started_y = np.sum(self.sensors[:3]*mask)
        reversed_x = mask[2]
        X_touched = False
        Y_touched = False
        rospy.loginfo("started from : x %d y %d"%(started_x, started_y))
        # color = self.colors[0]
        dx_quant = self.dx_quant
        dy_quant = self.dy_quant
        if self.hardness == 'lite':
            dx_quant = self.shifts['x'][color]
            dy_quant = self.shifts['y'][color]

        while not rospy.is_shutdown():

            if self.rf_broken_flag:
                rospy.sleep(0.05)
                continue
            s_x = np.sum(self.sensors[3:]*mask)
            s_y = np.sum(self.sensors[:3]*mask)

            # XXX code
            if started_x == 0:
                if s_x == 0:
                    dx = dx_quant
                if s_x == 1:
                    X_touched = True
            if started_x == 1:
                if s_x == 0:
                    dx = dx_quant
                    started_x = 0
                if s_x == 1:
                    dx = -dx_quant
            if X_touched and s_x:
                dx = -self.shifts['x'][color]
            if X_touched and not s_x:
                dx = 0

            dx = -dx
            if reversed_x:
                dx = -dx

            # YYY code
            if started_y == 0:
                if s_y == 0:
                    dy = dy_quant
                if s_y == 1:
                    Y_touched = True
            if started_y == 1:
                if s_y == 0:
                    d = dy_quant
                    started_y = 0
                if s_y == 1:
                    dy = -dy_quant
            if Y_touched and s_y:
                dy = -self.shifts['y'][color]
            if Y_touched and not s_y:
                dy = 0
            
            rospy.loginfo(self.get_command_dx_dy(dx,dy))
            rospy.loginfo("X_touched %d Y_touched %d s_x %d s_y %d "%(X_touched, Y_touched, s_x, s_y))
            command, name = self.get_command_dx_dy(dx,dy)
            if dx == 0 and dy == 0:
                rospy.loginfo("FINISHED by dx dy")
                last_x = self.final_shifts['x'][case][color]
                last_y = self.final_shifts['y'][case][color]
                if last_x:
                    self.wait_for_movement(self.get_command_dx_dy(last_x, 0)[0], "MOVEODOM" + str(self.i))
                if last_y:
                    self.wait_for_movement(self.get_command_dx_dy(0, last_y)[0], "MOVEODOM" + str(self.i))
                break
            
            # self.command_pub.publish(command)
            self.wait_for_movement(command, "MOVEODOM" + str(self.i))




    def move_cycle_one_new_2(self, mask, case, color):
        
        Y_finished = False
        X_finished = False
        self.started_sensors = self.sensors
        rospy.loginfo("started from " + str(self.sensors))
        started_x = np.sum(self.sensors[3:]*mask)
        started_y = np.sum(self.sensors[:3]*mask)
        reversed_x = mask[2]
        X_touched = False
        Y_touched = False
        rospy.loginfo("started from : x %d y %d"%(started_x, started_y))
        # color = self.colors[0]
        prev_dx = 0
        prev_dy = 0
        dx_quant = self.dx_quant
        dy_quant = self.dy_quant
        prev_sensors = self.sensors
        prev_x = started_x
        prev_y = started_y
        while not rospy.is_shutdown():

            if self.rf_broken_flag:
                rospy.sleep(0.05)
                continue
            s_x = np.sum(self.sensors[3:]*mask)
            s_y = np.sum(self.sensors[:3]*mask)


            #if dx_quant < self.min_quant:
            #    dx_quant = self.min_quant

            #if dy_quant < self.min_quant:
            #    dy_quant = self.min_quant


            if s_x == 0:
                dx = dx_quant
            else:
                dx = -dx_quant

            if s_y == 0:
                dy = dy_quant
            else:
                dy = -dy_quant

            dx = -dx
            if reversed_x:
                dx = -dx

            if prev_dx * dx < 0:
                dx /= self.divider
                dx_quant /= self.divider

            if prev_dy * dy < 0:
                dy /= self.divider
                dy_quant /= self.divider
                
            prev_dy = dy
            prev_dx = dx

            rospy.loginfo("X_touched %d Y_touched %d s_x %d s_y %d "%(X_touched, Y_touched, s_x, s_y))
            
            if np.abs(dx) < self.tolerance:
                dx_quant = 0
            if np.abs(dy) < self.tolerance:
                dy_quant = 0

            if np.abs(dx) < self.tolerance and np.abs(dy) < self.tolerance:
                rospy.loginfo("FINISHED by dx dy")
                last_x = self.final_shifts['x'][case][color]
                last_y = self.final_shifts['y'][case][color]
                if last_x:
                    self.wait_for_movement(*self.get_command_dx_dy(last_x, 0))
                if last_y:
                    self.wait_for_movement(*self.get_command_dx_dy(0, last_y))
                break

            
            rospy.loginfo(self.get_command_dx_dy(dx,dy)[0])
            self.wait_for_movement(*self.get_command_dx_dy(dx,dy))




    def get_allowed_mask(self, case):
        if case in self.masks:
            return self.masks[case]
        else:
            return self.masks

    def angle_calibration(self):
        (trans, rot) = self.listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        yaw = yaw % (np.pi / 2)
        a = - (yaw if yaw < np.pi / 4 else yaw - np.pi / 2)

        rospy.loginfo("rotation on angle " + str(a))

        #def tm(e):
        #    self.command_pub.publish("move_heap_121 162 0 0 " + str(a) + ' 0 0 0.5')


        #rospy.sleep(0.3)
        self.i += 1
        name = "rotate_on_heap" + str(self.i)
        cmd = name + " 162 0 0 " + str(a) + ' 0 0 0.5'
        self.wait_for_movement(cmd, name)
        rospy.loginfo("rotation finished")

    def start_command_callback(self):
        def cb(data):
            data_splitted = data.data.split()


            time_now = rospy.get_time()
            if len(data_splitted) > 1:
                action_type = data_splitted[1]
            else:
                return
            if action_type == "MOVETOHEAP":
                rospy.loginfo("Receive command " + data.data)
               
                case = int(data_splitted[2])
 
                try:
                    #if len(sys.argv) < 5:
                    if case <= 10:
                        self.angle_calibration()
                except:
                    pass
               
                self.command_pub.publish("col_off 224 0")
                n_mans = int((len(data_splitted) - 4)/2)
                self.colors = [int(c) for c in data_splitted[3:3+n_mans]]
                self.mans   = [int(m) for m in data_splitted[-n_mans-1:-1]]
                self.hardness = data_splitted[-1]
                rospy.loginfo(case)
                rospy.loginfo("COLORS "+str(self.colors))
                rospy.loginfo("MANS "+str(self.mans))
                yellow_fix = case // 20
                case = case % 20
                rospy.loginfo(str(yellow_fix) + ' ' + str(case))
                rospy.loginfo("Start move to heap by barrier sensors")

                
                if case in [4,5,6]:
                    case += 3

                mask = self.get_allowed_mask(case)
                
                if case > 10:
                    self.move_cycle_odom()

                if case in [0,2,3]:
                    left_right = False
                    for m,c in zip(self.mans, self.colors):
                        if m in [0,2]:
                            self.move_cycle_one_new(self.get_allowed_mask(m+7),m+7, c)
                            left_right = True
                    if not left_right:
                        m = self.mans[0]
                        c = self.colors[0]
                        self.move_cycle_one_new(self.get_allowed_mask(m+7),m+7,c)



                if case in [7, 8, 9]:
                    self.move_cycle_one_new(mask, case, self.colors[0])
                elif case == 1:
                    self.move_cycle_one_new(self.get_allowed_mask(7),7,self.colors[0])
                rospy.loginfo("MOVETOHEAP FINNISH %f sec"%(rospy.get_time() - time_now))
                self.response_pub.publish(data_splitted[0] + ' finished')
                
        return cb

    def wait_for_movement(self, cmd, name="MOVEODOM"):
        self.has_moved[name] = "started"
        self.command_pub.publish(cmd)
        while not self.has_moved[name] == "finished":
            rospy.sleep(0.05)
            # msg = rospy.wait_for_message(self.response_pub_name, String, timeout=3)
            # if msg.data == name + " finished":
            #    break

        return

    def get_command_dx_dy(self, dx, dy):
        dz = 0
        self.i += 1
        command_string = "MOVEODOM" + str(self.i)
        command_name = command_string
        command_string += " " + self.command_name + ' '
        command_string += str(dx) + ' ' + str(dy) + ' ' + str(dz) + ' '
        command_string += str(self.speed_xy) + ' ' + str(self.speed_xy) + ' ' + str(self.speed_z)
        return command_string, command_name




import sys
if __name__ == '__main__':
    try:
        rospy.init_node('barrier_move_node', anonymous=True)
        urfdata = "/main_robot/unstable_sensors_data"
        bn = BarrierNavigator("/main_robot/stm_command", "/main_robot/response", "/main_robot/corrected_barrier_rangefinder_data",urfdata)
        # pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, bn.start_command_callback())
        rospy.Subscriber("/main_robot/barrier_rangefinders_data", Int32MultiArray, bn.barrier_sensors_callback())
        # pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)
        rospy.loginfo(sys.argv)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
