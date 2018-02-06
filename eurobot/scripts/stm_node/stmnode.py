#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from STMprotocol import STMprotocol
from threading import Lock


class STMnode(STMprotocol):
    def __init__(self, serial_port):
        super(STMnode, self).__init__(serial_port)
        self.mutex = Lock()

        # ROS
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)

        # high-level commands info (for handling response)
        # TODO: test and debug
        self.actions_in_progress = ['']  # action_names, indexing corresponds to types indexing
        self.action_types = []  # list of high-level action types only

    def parse_data(self, data):
        data_splitted = data.data.split()
        action_name = data_splitted[0]
        action_type = int(data_splitted[1])
        args_str = data_splitted[2:]
        # TBD: split any chars in Strings like 'ECHO'->['E','C','H','O']
        action_args_dict = {'B': ord, 'H': int, 'f': float}
        args = [action_args_dict[t](s) for t, s in zip(self.pack_format[action_type][1:], args_str)]
        return action_name, action_type, args

    def stm_command_callback(self, data):
        # Lock() is used to prevent mixing bytes of diff commands to STM
        self.mutex.acquire()
        action_name, action_type, args = self.parse_data(data)
        self.send(action_name, action_type, args)
        self.mutex.release()

    def send(self, action_name, action_type, args):
        # send command to STM32
        successfully, args_response = self.send_command(action_type, args)

        # high-level commands handling
        if action_type in self.action_types:
            # store action_name
            self.actions_in_progress[self.action_types[action_type]] = action_name

        return successfully, args_response

    def handle_response(self, status):  # TODO
        """Handles response for high-lvl commands (only)."""
        l = len(status)
        for i in range(l):
            # mind that indeces in status[] correspond to indexes in actions_in_progress[]
            if status[i] == '0' and len(self.actions_in_progress[i]) > 0:
                self.actions_in_progress[i] = ''  # stop storing this action_name
                self.pub_response.publish(self.actions_in_progress[i] + " done")  # publish response

    def publish_coords(self, coords):
        self.pub_stm_coords.publish(' '.join(map(str, [coords[0] * 1000, coords[1] * 1000, coords[2]])))


if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = STMnode(serial_port)

    # turn stm inverse kinematics handler ON
    stm.send("set_inverse_kinematics_ON", 13, [1])

    # set initial coords in STM
    initial_coords = [rospy.get_param('start_x') / 1000.0, rospy.get_param('start_y') / 1000.0, rospy.get_param('start_a')];
    stm.send("set_initial_coords", 14, initial_coords)

    # Periodically publish STM status
    rate = rospy.Rate(80)  # in hz
    while not rospy.is_shutdown():
        successfully, coords = stm.send('request_stm_status', 15, [])
        if successfully:
            stm.publish_coords(coords)
        # TODO: status = ...
        # TODO: stm.handle_response(status)
        rate.sleep()