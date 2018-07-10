# coding=utf-8
"""
Driver for PNS(Poly Navigation System) Rover.
Based on PySerial https://github.com/pyserial/pyserial
Created on 2018-07-01
@author: Ocean
"""

import sys
import threading
import datetime
import time
import operator
import struct
import glob
import serial
try:
    from queue import Queue  # python3
except ImportError:
    from Queue import Queue  # python2


class GrabRoverData:
    def __init__(self):
        ''' initialization
        '''
        self.port = ""
        self.baud = 0
        self.ser = None  # the active UART
        self.threads = []  # thread of receiver and paser
        self.data_queue = Queue()  # data container
        self.exit_thread = False  # flag of exit threads
        self.exit_lock = threading.Lock()  # lock of exit_thread
        self.data_lock = threading.Lock()  # lock of data_queue
        print('Program start at:{0}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')))
        self.framenum = 0

    def reinit(self):
        ''' re-init parameters when occur SerialException.
        '''
        self.port = ""
        self.baud = 0
        if not self.data_queue.empty():
            self.data_queue.get()
        self.exit_thread = False
        self.threads = []  # clear threads

    def receiver(self):
        ''' receive serial data and push data into data_queue.
            return when occur SerialException
        '''
        while True:
            try:
                serial_data = bytearray(self.ser.read(100))
            except serial.SerialException:
                print('serial exception')
                self.exit_lock.acquire()
                self.exit_thread = True  # Notice thread paser to exit.
                self.exit_lock.release()
                return  # exit thread receiver

            if len(serial_data):
                # print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:') + ' '.join('0X{0:x}'.format(serial_data[i]) for i in range(len(serial_data))))
                self.data_lock.acquire()
                for i in range(len(serial_data)):
                    self.data_queue.put(serial_data[i])
                self.data_lock.release()

    def parser(self):
        ''' get serial data from data_queue and parse data into one whole frame.
            return when occur SerialException in thread receiver.
        '''
        HEADER = [0XAF, 0X20, 0X05]
        PAYLOAD_LEN_IDX = 4
        MSG_SUB_ID_IDX = 3

        header_tp = []
        find_header = False
        frame = []
        payload_len = 0

        while True:
            self.exit_lock.acquire()
            if self.exit_thread:
                self.exit_lock.release()
                return  # exit thread parser
            self.exit_lock.release()

            self.data_lock.acquire()
            if self.data_queue.empty():
                self.data_lock.release()
                time.sleep(0.01)
                continue
            else:
                data = self.data_queue.get()
                self.data_lock.release()
                # print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:') + hex(data))

                if find_header:
                    frame.append(data)
                    if PAYLOAD_LEN_IDX + 2 == len(frame):
                        payload_len = 256 * frame[PAYLOAD_LEN_IDX + 1] + frame[PAYLOAD_LEN_IDX]
                    elif 6 + payload_len + 2 == len(frame):  # 6: len of header; 2:len of checksum.
                        find_header = False
                        # checksum
                        result = self.checksum(frame[PAYLOAD_LEN_IDX + 2:PAYLOAD_LEN_IDX + payload_len + 2])
                        if result[0] == frame[-2] and result[1] == frame[-1]:
                            # find a whole frame
                            if 0X01 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_01(frame)  # Kalman Filter Navigation Message
                            elif 0X02 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_02(frame)  # Satellite Signal Strength
                            elif 0X03 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_03(frame)  # SV Visibility
                            elif 0X04 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_04(frame)  # Install Parameters
                        else:
                            print("Checksum error!")
                    else:
                        pass
                else:  # if haven't found header [0XAF, 0X20, 0X05].
                    if len(header_tp) >= 3:
                        del header_tp[0]
                    header_tp.append(data)
                    if operator.eq(header_tp, HEADER):
                        frame = header_tp[:]  # header_tp.copy()
                        header_tp = []
                        find_header = True

    def open_serial_port(self):
        ''' open serial port
            :returns: true when successful
        '''
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            return True
        except (OSError, serial.SerialException):
            print(serial.SerialException)
            return False

    def start_log(self):
        ''' start two threads: receiver and parser.
        '''
        if not self.open_serial_port():
            return

        funcs = [self.receiver, self.parser]
        for i in range(len(funcs)):
            t = threading.Thread(target=funcs[i], args=())
            t.start()
            self.threads.append(t)

        for i in range(len(self.threads)):
            self.threads[i].join()

        if self.ser is not None:
            self.ser.close()

    def find_device(self):
        ''' Finds active ports and then autobauds units, repeats every 0.1 seconds
        '''
        while not self.autobaud(self.find_ports()):
            time.sleep(0.1)

    def find_ports(self):
        ''' Lists serial port names. Code from
            https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
            Successfully tested on Windows 8.1 x64, Windows 10 x64, Mac OS X 10.9.x / 10.10.x / 10.11.x and Ubuntu 14.04 / 14.10 / 15.04 / 15.10 with both Python 2 and Python 3.
            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        '''
        print('scanning ports')
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                print('Trying: ' + port)
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def autobaud(self, ports):
        '''Autobauds unit - first check for stream_mode / continuous data, then check by polling unit
           Converts resets polled unit (temporarily) to 100Hz ODR
           :returns:
                true when successful
        '''
        for port in ports:
            for baud in [230400, 115200]:
                try:
                    s = serial.Serial(port, baud, timeout=2)  # Assume we can grab at least one whole frame in 2 secondss.
                    serial_data = bytearray(s.read(300))  # Assume max_len of a frame is less than 300 bytes.
                    s.close()
                    if serial_data.find((b'\xAF\x20\x05')) > -1:  # if find the header "0XAF 0X20 0X05"
                        self.port = port
                        self.baud = baud
                        print('Connected: {0}  {1}'.format(self.port, self.baud))
                        return True
                except (OSError, serial.SerialException):
                    if s is not None:
                        s.close()
        return False

    def checksum(self, data):
        ''' Calculate the checksum
        '''
        checksum_a = checksum_b = 0
        for i in data:
            checksum_a += i
            checksum_b += checksum_a

        checksum_a %= 256
        checksum_b %= 256
        return checksum_a, checksum_b

    def handle_msg_01(self, data):
        '''Kalman Filter Navigation Message
        '''
        pi = 3.14159265
        system_time = struct.unpack('<d', struct.pack('8B', *(data[6:14])))[0]
        GPS_time = struct.unpack('<d', struct.pack('8B', *(data[14:22])))[0]
        latitude = (struct.unpack('<d', struct.pack('8B', *(data[22:30])))[0]) * 180/pi
        longitude = struct.unpack('<d', struct.pack('8B', *(data[30:38])))[0] * 180/pi
        ellipsoidal_height = struct.unpack('<d', struct.pack('8B', *(data[38:46])))[0]
        velocity_north = struct.unpack('<d', struct.pack('8B', *(data[46:54])))[0]
        velocity_east = struct.unpack('<d', struct.pack('8B', *(data[54:62])))[0]
        velocity_down = struct.unpack('<d', struct.pack('8B', *(data[62:70])))[0]
        roll = struct.unpack('<d', struct.pack('8B', *(data[70:78])))[0]
        pitch = struct.unpack('<d', struct.pack('8B', *(data[78:86])))[0]
        heading = struct.unpack('<d', struct.pack('8B', *(data[86:94])))[0]
        position_mode = data[94]
        velocity_mode = data[95]
        attitude_status = data[96]

        # print msg
        print('\r\nKalman Filter Navigation Message:\r\n system_time:{0} GPS_time: {1} latitude: {2} longitude: {3} ellipsoidal_height: {4} \
        velocity_north:{5} velocity_east:{6} velocity_down:{7} roll:{8} \
        pitch:{9} heading:{10} position_mode:{11} velocity_down:{12} attitude_status:{13}'
        .format(system_time, GPS_time, latitude, longitude, ellipsoidal_height,
        velocity_north, velocity_east, velocity_down, roll, pitch,
        heading, position_mode, velocity_mode, attitude_status))

    def handle_msg_02(self, data):
        '''Satellite Signal Strength
        '''
        system_time = struct.unpack('<d', struct.pack('8B', *(data[6:14])))[0]
        GPS_time = struct.unpack('<d', struct.pack('8B', *(data[14:22])))[0]
        receiver_id = data[22]
        antenna_id = data[23]
        nsv_idx = 24
        n_sv = data[nsv_idx]
        print('\r\nSatellite Signal Strength Message:\r\n system_time: {0:3.2f} GPS_time: {1:3.2f} Satellites_Number: {2} receiver_id: {3} antenna_id: {4}'
        .format(system_time, GPS_time, n_sv, receiver_id, antenna_id))

        for i in range(n_sv):
            sv_system = data[nsv_idx + i*10 + 1]
            SVID = data[nsv_idx + i*10 + 2]
            L1CN = struct.unpack('<f', struct.pack('4B', *(data[nsv_idx + i*10 + 3: nsv_idx + i*10 + 7])))[0]
            L2CN = struct.unpack('<f', struct.pack('4B', *(data[nsv_idx + i*10 + 7: nsv_idx + i*10 + 11])))[0]
            print('Satellite-{0} sv_system: {1} SVID: {2} L1CN: {3:3.2f} L2CN: {4:3.2f}'.format(i, sv_system, SVID, L1CN, L2CN))

    def handle_msg_03(self, data):
        pass

    def handle_msg_04(self, data):
        pass


def main():
    '''main'''
    grab = GrabRoverData()
    while True:
        grab.reinit()
        grab.find_device()
        grab.start_log()


if __name__ == '__main__':
    main()
