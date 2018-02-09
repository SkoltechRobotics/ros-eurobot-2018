#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np

class stm_node():
    def __init__(self):
        # ROS
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        self.pub_stm_coords = rospy.Publisher('stm/coordinates', String, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)

        # high-level commands info (for handling response)
        self.actions_in_progress = [''] # action_names, indexing corresponds to types indexing
        self.action_types = [] # list of high-level action types only
        self.actions_with_response = [0x0E]

        self.pack_format = {
            0x01: "=BBBB",
            0x03: "=Bf",
            0x04: "=B",
            0x05: "=B",
            0x08: "=fff",
            0x09: "=",
            0x0a: "=",
            0x0b: "=BH",
            0x0c: "=B",
            0x0d: "=B",
            0xa0: "=fff",
            0xa1: "=fff",
            0xb0: "=B",
            0xc0: "=BB",
            0xb1: "=B",
            0x0e: "=fff",
            0x0f: "=",
            0xb0: "=B",
        }

        self.unpack_format = {
            0x01: "=BBBB",
            0x03: "=BB",
            0x04: "=BB",
            0x05: "=BB",
            0x08: "=BB",
            0x09: "=fff",
            0x0a: "=fff",
            0x0b: "=BB",
            0x0c: "=f",
            0x0d: "=BB",
            0xa0: "=Bfff",
            0xa1: "=BB",
            0xb0: "=BB",
            0xc0: "=BB",
            0xb1: "=BB",
            0x0e: "=BB",
            0x0f: "=fff",
            0xb0: "=BB",
        }

        self.freq = 100
        self.rate = rospy.Rate(self.freq) # 100Hz

        self.coords = np.array([rospy.get_param('start_x') / 1000.0, rospy.get_param('start_y') / 1000.0, rospy.get_param('start_a')])
        self.vel = np.array([0.0, 0.0, 0.0])

        rospy.Timer(rospy.Duration(1./80), self.pub_timer_callback)

    def parse_data(self, data):
        data_splitted = data.data.split()
        action_name = data_splitted[0]
        action_type = int(data_splitted[1])
        args_str = data_splitted[2:]
        # TBD: split any chars in Strings like 'ECHO'->['E','C','H','O']
        action_args_dict = {'B':ord, 'H':int, 'f':float}
        args = [action_args_dict[t](s) for t,s in zip(self.pack_format[action_type][1:], args_str)]
        return action_name,action_type,args

    def stm_command_callback(self, data):
        # parse data
        action_name,action_type,args = self.parse_data(data)
        rospy.loginfo(str(action_type))

        # simulate STM32 response
        successfuly = True
        args_response = "Ok"
        if action_type == 0x08:
            vel = np.array(args)
            self.vel[0] = vel[0]*np.cos(self.coords[2]) - vel[1]*np.sin(self.coords[2])
            self.vel[1] = vel[1]*np.cos(self.coords[2]) + vel[0]*np.sin(self.coords[2])
            self.vel[2] = vel[2]
        elif action_type == 0x09:
            args_response = self.vel
        elif action_type == 0x0E:
            self.coords = np.array(args)
        elif action_type == 0x0F:
            args_response = self.coords
            
        # high-level commands handling
        if action_type in self.action_types:
            # store action_name
            # rospy.loginfo(str(action_type))
            self.actions_in_progress[self.action_types[action_type]] = action_name

        # low-level commands handling
        elif action_type in self.actions_with_response:
            rospy.loginfo(action_name + " finished")
            def delayed_cb(e):
                self.pub_response.publish(action_name + " finished")
            rospy.Timer(rospy.Duration(0.2), delayed_cb, oneshot=True)

    def publish_coords(self):
        self.pub_stm_coords.publish(' '.join(map(str, [self.coords[0]*1000, self.coords[1]*1000, self.coords[2]])))

    def handle_response(self, status):
        """Handles response for high-lvl commands (only)."""
        l = len(status)
        for i in range(l):
            # mind that indeces in status[] correspond to indeces in actions_in_progress[]
            rospy.loginfo(status[i] + ' ' + str(self.action_in_progress[i]))
            if status[i] == '0' and len(self.actions_in_progress[i]) > 0:
                self.actions_in_progress[i] = ''                                    # stop storing this action_name
                self.pub_response.publish(self.actions_in_progress[i] + " done")    # publish responce

            self.rate.sleep()

    def integrate(self):
        while not rospy.is_shutdown():
            noise = np.random.normal(size=3)
            noise *= 0.1 * self.vel / self.freq
            #noise *= 0.96 # simulate bad estimation of wheel size, etc.
            self.coords = self.coords + self.vel / self.freq + noise
            self.coords[2] = self.coords[2] % (2 * np.pi)
            self.rate.sleep()

    def pub_timer_callback(self, event):
        self.publish_coords()

if __name__ == '__main__':
    try:
        stm = stm_node()

        stm.integrate()
    except rospy.ROSInterruptException:
        pass
