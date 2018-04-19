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
                0: 0.004,
                1: -0.003,
                2: -0.005,
                3: -0.003,
                4: -0.003,
            },
            8: {
                0: -0.0015,
                1: -0.0015,
                2: -0.004,
                3: -0.002,
                4: -0.002,
            },
            9: {
                0: -0.005,
                1: 0.00,
                2: 0.000,
                3: +0.003,
                4: +0.005,
            }
        },
        'y': {
            7: {
                0: -0.004,
                1: 0,
                2: 0,
                3: 0,
                4: 0,
            },
            8: {
                0: -0.004,
                1: 0.005,
                2: 0.001,
                3: 0.003,
                4: 0.002,
            },
            9: {
                0: -0.004,
                1: +0.005,
                2: 0,
                3: -0.002,
                4: +0.000,
            }
        }
    }
    def __init__(self, stm_command_publisher_name, response_pub_name, corrected_rf_data_pub_name):
        self.sensors = []
        self.phase = 0
        self.sensors_goals = []
        self.dx_quant = 0.002
        self.dy_quant = 0.002
        self.dx_finish = 0.001
        self.dy_finish = 0.002
        self.speed_xy = 0.2
        self.speed_z = 0.1
        self.command_name = str(0xa2)
        self.ready = True
        self.command_pub_name = stm_command_publisher_name
        self.command_pub = rospy.Publisher(stm_command_publisher_name, String, queue_size=10)
        self.corrected_rf_data = rospy.Publisher(corrected_rf_data_pub_name, Int32MultiArray, queue_size=10)
        self.response_pub_name = response_pub_name
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
            # print(self.sensors)
            # rospy.loginfo(self.sensors_queue)
            # rospy.loginfo(self.sensors)
            self.corrected_rf_data.publish(data=self.sensors)

        return cb

    def move_cycle_new(self, mask = np.array([1,1,1])):
        Y_finished = False
        X_finished = False
        self.started_sensors = self.sensors
        rospy.loginfo("started from " + str(self.sensors))
        steps_FFF_x = 0
        while not rospy.is_shutdown() and (not X_finished or not Y_finished):
            rospy.loginfo(self.sensors)
            if self.rf_broken_flag:
                rospy.sleep(0.05)
                continue
            # YYYYYYYYYYYYYYYYYYYYYYYYYYY axis
            dY = 0
            dX = 0
            if not Y_finished:
                dY = -1 if np.any(self.started_sensors[:3]*mask) else +1
                dY *= self.dy_quant
                if (np.any(self.started_sensors[:3]*mask) and not np.any(self.sensors[:3]*mask)) or \
                        (not np.any(self.started_sensors[:3]*mask) and np.any(self.sensors[:3]*mask)):
                    # finished by Y!
                    dY = -1*self.dy_finish if np.any(self.sensors[:3]*mask) else 1*self.dy_finish
                    Y_finished = True

            if not X_finished:
                ds = self.sensors[3:]*mask
                # dX = -ds[2] * self.dx_quant + (ds[0] or ds[1]) * self.dx_quant
                st_sensors_x = self.started_sensors[3:]*mask
                sensors_x = self.sensors[3:]*mask
                if not np.any(st_sensors_x) or (not np.any(st_sensors_x[:2]) and st_sensors_x[2]):
                    steps_FFF_x += 1 if np.all(sensors_x == 0) else 0
                    dX = -self.dx_quant
                    if np.any(sensors_x[:2]):
                        if np.any(sensors_x[:2]) and sensors_x[2]:
                            dX = 0
                            rospy.loginfo("X BOTH")
                        else:
                            dX = steps_FFF_x * self.dx_quant / 2 # 3*self.dx_finish
                            rospy.loginfo("X 1")
                        X_finished = True
                if np.any(st_sensors_x[:2]) and not st_sensors_x[2]:
                    dX = self.dx_quant
                    if np.any(sensors_x[:2]) and sensors_x[2]:
                        dX = 0
                        rospy.loginfo("X BOTH")
                        X_finished = True
                    elif not np.any(sensors_x[:2]):
                        dX = -steps_FFF_x * self.dx_quant / 2 # 3*self.dx_finish
                        rospy.loginfo("X 2")
                        X_finished = True
                if np.any(st_sensors_x[:2]) and st_sensors_x[2]:
                    dX = 0
                    rospy.loginfo("X 3")
                    X_finished = True




            cmd, _, _ = self.get_command_dx_dy(dX,dY)
            rospy.loginfo((dX, dY))
            # self.command_pub.publish(cmd)
            self.wait_for_movement(cmd, self.CMD_NAME + str(self.i))

    def move_cycle(self, phase, mask = np.array([1,1,1])):
        while not rospy.is_shutdown():
            rospy.loginfo(self.sensors)
            # if np.all(np.array(self.sensors) - np.array(self.sensors_goals) == 0):
            #     rospy.loginfo("FINISHED by sensor_goals")
            #     break
            if np.all(self.sensors[:3]*mask == 0) and phase == 0:
                rospy.loginfo("FINISHED by y sensor PHASE 0")
                break
            if np.any(self.sensors[:3]*mask) and phase == 1:
                rospy.loginfo("FINISHED by y sensor PHASE 1")
                break
            command, dx, dy = self.get_command(phase, mask)
            if dx == 0 and dy == 0:
                rospy.loginfo("FINISHED by dx dy")
                break
            rospy.loginfo(command)
            # self.command_pub.publish(command)
            self.wait_for_movement(command, "MOVEODOM" + str(self.i))

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
        while not rospy.is_shutdown():

            if self.rf_broken_flag:
                rospy.sleep(0.05)
                continue
            s_x = np.any(self.sensors[3:]*mask)
            s_y = np.any(self.sensors[:3]*mask)

            # XXX code
            if started_x == 0:
                if s_x == 0:
                    dx = self.dx_quant
                if s_x == 1:
                    X_touched = True
            if started_x == 1:
                if s_x == 0:
                    dx = self.dx_quant
                    started_x = 0
                if s_x == 1:
                    dx = -self.dx_quant
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
                    dy = self.dy_quant
                if s_y == 1:
                    Y_touched = True
            if started_y == 1:
                if s_y == 0:
                    dy = self.dy_quant
                    started_y = 0
                if s_y == 1:
                    dy = -self.dy_quant
            if Y_touched and s_y:
                dy = -self.shifts['y'][color]
            if Y_touched and not s_y:
                dy = 0

            rospy.loginfo("X_touched %d Y_touched %d s_x %d s_y %d "%(X_touched, Y_touched, s_x, s_y))
            command, dx, dy = self.get_command_dx_dy(dx,dy)
            if dx == 0 and dy == 0:
                rospy.loginfo("FINISHED by dx dy")
                last_x = self.final_shifts['x'][case][color]
                last_y = self.final_shifts['y'][case][color]
                rospy.loginfo("FINISHED by dx dy %f %"%(last_x, last_y))
                if last_x:
                    self.wait_for_movement(self.get_command_dx_dy(last_x, 0)[0], "MOVEODOM" + str(self.i))
                if last_y:
                    self.wait_for_movement(self.get_command_dx_dy(0, last_y)[0], "MOVEODOM" + str(self.i))
                break
            rospy.loginfo(command)
            # self.command_pub.publish(command)
            self.wait_for_movement(command, "MOVEODOM" + str(self.i))




    def move_cycle_one(self, phases_x, phases_y, case):
        i_x = 0
        i_y = 0
        phase_y = phases_y[0]
        phase_x = phases_x[0]
        allowed_mask = self.get_allowed_mask(case)
        X_finished = False
        Y_finished = False
        while not rospy.is_shutdown():
            if self.rf_broken_flag:
                rospy.sleep(0.05)
                continue
            phase_0_cond = np.any(self.sensors[:3] * allowed_mask) and phase_y == 0
            phase_1_cond = np.all(self.sensors[:3] * allowed_mask == 0) and phase_y == 1
            if not Y_finished:
                if phase_0_cond or phase_1_cond:
                    if i_y == len(phases_y) - 1:
                        phase_y = 1 - phase_y
                        Y_finished = True
                    else:
                        i_y += 1
                        rospy.loginfo("PHASE Y CHANGED")
                        phase_y = phases_y[i_y]
            else:
                phase_y = -1
            phase_0_cond = np.any(self.sensors[3:] * allowed_mask) and phase_x == 0
            phase_1_cond = np.all(self.sensors[3:] * allowed_mask == 0) and phase_x == 1
            if not X_finished:
                if phase_0_cond or phase_1_cond:
                    if i_x == len(phases_x) - 1:
                        phase_x = 1 - phase_x
                        X_finished = True
                    else:
                        i_x += 1
                        rospy.loginfo("PHASE X CHANGED")
                        phase_x = phases_x[i_x] if sum(np.array([1, 1, 0]) * allowed_mask) >= 1 else 1 - phases_x[i_x]
            else:
                phase_x = -1
            command, dx, dy = self.get_command_one(phase_x, phase_y, case)
            rospy.loginfo(str(phase_x) + ' ' + str(dx) + ' ' + str(phase_y) + ' ' + str(dy))
            if dx == 0 and dy == 0:
                rospy.loginfo("FINISED by dx dy")
                break
            rospy.loginfo(command)
            # self.command_pub.publish(command)
            self.wait_for_movement(command, "MOVEODOM" + str(self.i))


    def get_allowed_mask(self, case):
        if case in self.masks:
            return self.masks[case]
        else:
            return self.masks[0]

    def angle_calibration(self):
        rospy.loginfo('entered angle calibration')
        (trans, rot) = self.listener.lookupTransform('/map', '/main_robot', rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        yaw = yaw % (np.pi / 2)
        a = - (yaw if yaw < np.pi / 4 else yaw - np.pi / 2)

        rospy.loginfo("rotation on angle " + str(a))

        def tm(e):
            self.command_pub.publish("move_heap_121 162 0 0 " + str(a) + ' 0 0 0.5')

        # rospy.Timer(rospy.Duration(0.3), tm, oneshot=True)
        tm(1)
        rospy.sleep(0.3)
        # self.wait_for_movement("move_heap_121")
        rospy.loginfo("rotation finished")

    def start_command_callback(self):
        def cb(data):
            data_splitted = data.data.split()

            if len(data_splitted) > 1:
                action_type = data_splitted[1]
            else:
                return
            if action_type == "MOVETOHEAP":
                rospy.loginfo("Receive command " + data.data)

                try:
                    self.angle_calibration()
                    rospy.loginfo("successfully rotated")
                except Exception, error:
                    rospy.loginfo( str(error) )

                
                case = int(data_splitted[2])
                n_mans = int((len(data_splitted) - 3)/2)
                self.colors = [int(c) for c in data_splitted[3:3+n_mans]]
                self.mans   = [int(m) for m in data_splitted[-n_mans:]]
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
                    # self.move_cycle_new(mask)
                # if case in [0, 2, 3]:
                #     if np.any(self.sensors[:3]):    #  SOME TRIGGERED
                #         self.set_sensors_goals(0)
                #         self.move_cycle(0, mask)
                #         rospy.loginfo("PHASE 0 FINISHED")
                #
                #     # rospy.sleep(5)
                #     else:                               # ALL NOT TRIGGERED
                #         self.set_sensors_goals(1)
                #         self.move_cycle(1, mask)
                #         rospy.loginfo("PHASE 1 FINISHED")
                #
                #         self.set_sensors_goals(0)
                #         self.move_cycle(0, mask)
                #         rospy.loginfo("PHASE 0 FINISHED")
                #     self.move_cycle(2, mask)



                if case in [7, 8, 9]:
                    self.move_cycle_one_new(mask, case, self.colors[0])
                elif case == 1:
                    self.move_cycle_one_new(self.get_allowed_mask(7),7,self.colors[0])
                rospy.loginfo("MOVETOHEAP FINNISH")
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


    def set_sensors_goals(self, phase):
        if phase == 0:
            self.sensors_goals = np.zeros((6), dtype=np.int)
            self.sensors_goals[1] = 1
        if phase == 1:
            self.sensors_goals = np.zeros((6), dtype=np.int)

    def get_command(self, phase, mask = np.array([1,1,1])):
        # rospy.loginfo(self.sensors)
        ds = self.sensors*np.array(mask.tolist()*2) - self.sensors_goals
        dx = -ds[5] * self.dx_quant + (ds[3] or ds[4]) * self.dx_quant
        # rospy.loginfo(np.array(self.sensors[:3]))
        # rospy.loginfo(np.any(np.array(self.sensors[:3])))
        rospy.loginfo(int(np.any(ds[:3])))
        dy = 0
        if phase == 0:
            dy = -self.dy_quant
        if phase == 1:
            dy = +self.dy_quant
        dz = 0
        self.i += 1
        command_string += str(dx) + ' ' + str(dy) + ' ' + str(dz) + ' '
        command_string += str(self.speed_xy) + ' ' + str(self.speed_xy) + ' ' + str(self.speed_z)
        return command_string, dx, dy

    def get_command_dx_dy(self, dx, dy):
        dz = 0
        self.i += 1
        command_string = "MOVEODOM" + str(self.i) + " " + self.command_name + ' '
        command_string += str(dx) + ' ' + str(dy) + ' ' + str(dz) + ' '
        command_string += str(self.speed_xy) + ' ' + str(self.speed_xy) + ' ' + str(self.speed_z)
        return command_string, dx, dy

    def get_command_one(self, phase_x, phase_y, case):
        rospy.loginfo(self.sensors)

        if case in [1, 7, 8, 9]:
            if phase_x == 0:
                dx = -self.dx_quant
            elif phase_x == 1:
                dx = +self.dx_quant
            else:
                dx = 0

            if case == 9:
                dx = -dx
            if phase_y == 0:
                dy = +self.dy_quant
            elif phase_y == 1:
                dy = -self.dy_quant
            else:
                dy = 0
            return self.get_command_dx_dy(dx, dy)


def get_sensor_goal_values(sensor_data):
    # 0 - move to y+ (biggest x)
    # 1 - move to y+ (middle x)
    # 2 - move to y+ (smallest x)
    # 3 - move to x- (0)
    # 4 - move to x- (1)
    # 5 - move to x+ (2)
    pass

import sys
if __name__ == '__main__':
    try:
        rospy.init_node('barrier_move_node', anonymous=True)
        bn = BarrierNavigator("/main_robot/stm_command", "/main_robot/response", "/main_robot/corrected_barrier_rangefinder_data")
        # pub_command = rospy.Publisher("/main_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/main_robot/move_command", String, bn.start_command_callback())
        rospy.Subscriber("/main_robot/barrier_rangefinders_data", Int32MultiArray, bn.barrier_sensors_callback())
        # pub_response = rospy.Publisher("/main_robot/response", String, queue_size=2)
        rospy.loginfo(sys.argv)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
