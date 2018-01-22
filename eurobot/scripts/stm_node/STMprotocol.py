import serial
import struct
import datetime


class STMprotocol(object):
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, baudrate=64000, timeout=0.2)
        self.pack_format = {
            0x01: "=BBBB",
            0x03: "=Bf",
            0x04: "=B",
            0x05: "=B",
            0x06: "=Bffff",
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
        }

        self.unpack_format = {
            0x01: "=BBBB",
            0x03: "=BB",
            0x04: "=BB",
            0x05: "=BB",
            0x06: "=BB",
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
        }
    def pure_send_command(self, cmd, args):
        # Clear buffer
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        # Sending command
        parameters = bytearray(struct.pack(self.pack_format[cmd], *args))
        msg_len = len(parameters) + 5
        msg = bytearray([0xfa, 0xaf, msg_len, cmd]) + parameters
        crc = sum(msg) % 256
        msg += bytearray([crc])
        self.ser.write(msg)
        #Receiving data
        data = self.ser.read()
        if len(data) == 0:
            raise Exception("No data received")

        sync = ord(data[0])
        if sync != 0xfa:
            raise Exception("Incorrect byte of syncronization", sync)
        
        data = self.ser.read()
        if len(data) == 0:
            raise Exception("No adress received")
        adr = ord(data[0])
        
        if adr != 0xfa:
            raise Exception("Incorrect adress", adr)
        answer_len = ord(self.ser.read()[0])
        answer = bytearray(self.ser.read(answer_len - 3))
        
        if (sync + adr + answer_len + sum(answer[:-1])) % 256 != answer[-1]:
            raise Exception("Error with check sum", sync, adr, answer_len, answer)
        args = struct.unpack(self.unpack_format[cmd], answer[1:-1])
        return True, args
    def send_command(self, cmd, args, n_repeats = 5):
        for i in range(n_repeats):
            try:
                return self.pure_send_command(cmd, args)
            except Exception as exc:
                if i == n_repeats-1:
                    print('Exception:\t', exc)
                    print('At time:\t', datetime.datetime.now())
                    print('--------------------------')
        return False, None
