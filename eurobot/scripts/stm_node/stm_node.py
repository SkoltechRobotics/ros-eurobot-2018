#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
import struct
import datetime
from STMprotocol import STMprotocol


class stm_node(STMprotocol):
    def __init__(self, serial_port):
        super(stm_node, self).__init__(serial_port)
        
        # ROS
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        self.pub_stm_coords = rospy.Publisher('stm/coordinates', String, queue_size=10)
        self.pub_response = rospy.Publisher("response", String, queue_size=10) 

        # rate of publishing
        self.rate = rospy.Rate(40)

        # high-level commands info (for handling response)
        self.actions_in_progress = [''] # action_names, indexing corresponds to types indexing
        self.action_types = [] # list of high-level action types only

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

        ## Command handling
        # send command to STM32
        successfuly, args_response = self.send_command(action_type, args)
        if successfuly:
            print 'STM responded to cmd', action_name, '\twith args:', args_response

        # high-level commands handling
        if action_type in self.action_types:
            # store action_name
            self.actions_in_progress[self.action_types[action_type]] = action_name

        # low-level commands handling - not required
        #else:
        #    self.pub_response.publish(action_name + " ok")

        # pub stm/coordinates whenever stm status is requested
        if action_type == 0x0f:
            if successfuly:
                 self.pub_stm_coords.publish(' '.join(map(str, [args_response[0]*1000, args_response[1]*1000, args_response[2]])))

    def handle_response(self, status):
        """Handles response for high-lvl commands (only)."""
        l = len(status)
        for i in range(l):
            # mind that indeces in status[] correspond to indeces in actions_in_progress[]
            if status[i] == '0' and len(self.actions_in_progress[i]) > 0:
                self.actions_in_progress[i] = ''                                    # stop storing this action_name
                self.pub_response.publish(self.actions_in_progress[i] + " done")    # publish responce
        
        
    # infinite publishing cycle
    def publish_infinitely(self):
        while not rospy.is_shutdown():
            # TBD
            #status,x,y,a = self.send_command(0xA0, [0, 0, 0])
            #status = str(status)
            #self.pub_delta.publish(' '.join(map(str, [x,y,a])))
            #self.handle_response(status) # it will publish responce where needed
            self.rate.sleep()

if __name__ == '__main__':
    try:
        serial_port = "/dev/ttyUSB0"

        stm = stm_node(serial_port)
        stm.publish_infinitely()
    except rospy.ROSInterruptException:
        pass
