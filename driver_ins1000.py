# coding=utf-8
"""
Driver for PNS(Poly Navigation System) Rover.
Based on PySerial https://github.com/pyserial/pyserial
Created on 2018-07-01
@author: Ocean
"""

import sys
import os
import threading
import datetime
import time
import operator
import struct
import glob
import math
import json
import collections
import serial
try:
    from queue import Queue  # python3
except ImportError:
    from Queue import Queue  # python2


class RoverDriver:
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
        self.setting_folder = os.path.join(os.getcwd(), r'setting')  # use to store some configuration files.
        self.connection_file = os.path.join(self.setting_folder, 'connection.json')
        self.rover_file = os.path.join(self.setting_folder, 'rover.json')
        if not self.load_configuration():
            os._exit(1)
        print('Program start at:{0}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')))

    def reinit(self):
        ''' re-init parameters when occur SerialException.
        '''
        self.port = ""
        self.baud = 0
        self.close_serial_port()
        if not self.data_queue.empty():
            self.data_queue.get()
        self.exit_thread = False
        self.threads = []  # clear threads

    def receiver(self):
        ''' receive serial data and push data into data_queue.
            return when occur SerialException
        '''
        while True:
            self.exit_lock.acquire()
            if self.exit_thread:
                self.exit_lock.release()
                self.close_serial_port()
                return
            self.exit_lock.release()

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

        sync_pattern = collections.deque(3*[0], 3)
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
                        result = self.check_sum(frame[PAYLOAD_LEN_IDX + 2:PAYLOAD_LEN_IDX + payload_len + 2])
                        if result[0] == frame[-2] and result[1] == frame[-1]:
                            # find a whole frame
                            if 0X01 == frame[MSG_SUB_ID_IDX]:
                                self.set_packet_type('KFN')
                                self.handle_msg_01_(frame) # just for verify, will be deleted in future.
                                self.handle_msg_01(frame)  # Handle Kalman Filter Navigation Message
                            elif 0X02 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_02(frame)  # Handle Satellite Signal Strength
                            elif 0X03 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_03(frame)  # Handle SV Visibility
                            elif 0X04 == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_04(frame)  # Handle Install Parameters
                            elif 0X0A == frame[MSG_SUB_ID_IDX]:
                                self.handle_msg_0A(frame)  # Handle Repackaged GSV Message
                        else:
                            print("Checksum error!")
                    else:
                        pass
                else:  # if haven't found header [0XAF, 0X20, 0X05].
                    sync_pattern.append(data)
                    if operator.eq(list(sync_pattern), HEADER):
                        frame = HEADER[:]  # header_tp.copy()
                        find_header = True

    def handle_KeyboardInterrupt(self):
        ''' handle KeyboardInterrupt.
            returns: True when occur KeyboardInterrupt.
                     False when receiver and parser threads exit.
        '''
        while True:
            self.exit_lock.acquire()
            if self.exit_thread:
                self.exit_lock.release()
                return False  # return when receiver and parser threads exit
            self.exit_lock.release()

            try:
                time.sleep(0.1)
            except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
                self.exit_lock.acquire()
                self.exit_thread = True  # Notice thread receiver and paser to exit.
                self.exit_lock.release()
                print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
                return True

    def open_serial_port(self, port=False, baud=115200, timeout=0.1):
        ''' open serial port
            returns: true when successful
        '''
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
        except Exception as e:
            print('serial port open exception, port:{0} baud:{1} timeout:{2}'.format(port, baud, timeout) )
            self.ser = None

    def close_serial_port(self):
        '''close serial port
        '''
        if self.ser is not None:
            if self.ser.isOpen():
                self.ser.close()

    def start_log(self):
        ''' start two threads: receiver and parser.
            returns False when user trigger KeyboardInterrupt to stop this program.
            otherwise returns True.
        '''
        self.open_serial_port(self.port, self.baud, 0.1)
        if not self.ser:
            return True

        funcs = [self.receiver, self.parser]
        for i in range(len(funcs)):
            t = threading.Thread(target=funcs[i], args=())
            t.start()
            print("Thread[{0}({1})] started.".format(t.name, t.ident))
            self.threads.append(t)

        if self.handle_KeyboardInterrupt():
            return False

        for i in range(len(self.threads)):
            self.threads[i].join()
            print("Thread[{0}({1})] stoped.".format(self.threads[i].name, self.threads[i].ident))

        self.close_serial_port()
        return True

    def find_device(self):
        ''' Finds active ports and then autobauds units
        '''
        if self.try_last_port():
            return True
        else:
            while not self.autobaud(self.find_ports()):
                time.sleep(0.1)
        return True
        # while not self.autobaud(self.find_ports()):
        #     time.sleep(0.1)

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
                self.open_serial_port(port, baud, 1.0)  # Assume at least one whole frame can be grabed in 1 seconds.
                if self.ser:
                    serial_data = bytearray(self.ser.read(300))  # Assume max_len of a frame is less than 300 bytes.
                    self.ser.close()
                    if self.find_header(serial_data):
                        self.port = port
                        self.baud = baud
                        print('Connected: {0}  {1}'.format(self.port, self.baud))
                        self.save_last_port()
                        return True
        return False

    def try_last_port(self):
        '''try to open serial port based on the port and baud read from connection.json.
           try to find frame header in serial data.
           returns: True if find header
                    False if not find header.
        '''
        connection = None
        try:
            with open(self.connection_file) as json_data:
                connection = json.load(json_data)
            if connection:
                self.open_serial_port(port=connection['port'], baud=connection['baud'], timeout=1)
                if self.ser:
                    serial_data = bytearray(self.ser.read(300))  # Assume max_len of a frame is less than 300 bytes.
                    self.ser.close()
                    if self.find_header(serial_data):
                        self.port = connection['port']
                        self.baud = connection['baud']
                        print('Connected: {0}  {1}'.format(self.port, self.baud))
                        return True
                    else:
                        return False
                else:
                    return False
        except:
            return False

    def save_last_port(self):
        if not os.path.exists(self.setting_folder):
            try:
                os.mkdir(self.setting_folder)
            except:
                return

        connection = {"port" : self.ser.port, "baud" : self.ser.baudrate }
        try:
            with open(self.connection_file, 'w') as outfile:
                json.dump(connection, outfile)
        except:
            pass

    def load_configuration(self):
        '''
        load properties from 'rover.json'
        returns: True when load successfully.
                 False when load failed.
        '''
        try:
            with open(self.rover_file) as json_data:
                self.rover_properties = json.load(json_data)
            return True
        # except (ValueError, KeyError, TypeError) as error:
        except Exception as e:
            print(e)
            return False

    def find_header(self, data):
        if data.find((b'\xAF\x20\x05')) > -1:  # if find the header "0XAF 0X20 0X05"
            return True
        return False

    def check_sum(self, data):
        ''' Calculate the checksum
        '''
        checksum_a = checksum_b = 0
        for i in data:
            checksum_a += i
            checksum_b += checksum_a

        checksum_a %= 256
        checksum_b %= 256
        return checksum_a, checksum_b

    def get_packet_type(self):
        return self.packet_type

    def set_packet_type(self, packet_type):  #consider add lock when other thread invoke this function.
        self.packet_type = packet_type

    def parse_frame(self, frame):
        '''Parses packet payload using rover.json as reference
        '''
        PAYLOAD_LEN_IDX = 4
        payload_len = 256 * frame[PAYLOAD_LEN_IDX + 1] + frame[PAYLOAD_LEN_IDX]
        payload = frame[6:payload_len+6]   # extract the payload
        data = [] 

        # Find the packet in the imu_properties from unit's JSON description
        output_packet = next((x for x in self.rover_properties['userMessages']['outputPackets'] if x['name'] == self.packet_type), None)
        input_packet = next((x for x in self.rover_properties['userMessages']['inputPackets'] if x['name'] == self.packet_type), None)

        if output_packet:
            self.data = self.unpack_output_packet(output_packet, payload)
            self.change_scale(output_packet, self.data)
        elif input_packet:
            data = self.unpack_input_packet(input_packet['responsePayload'], payload) 
        return data

    def unpack_output_packet(self, output_message, payload):
        length = 0
        pack_fmt = '<'
        for value in output_message['payload']:
            if value['type'] == 'float':
                pack_fmt += 'f'
                length += 4
            elif value['type'] == 'uint32':
                pack_fmt += 'I'
                length += 4
            elif value['type'] == 'int32':
                pack_fmt += 'i'
                length += 4
            elif value['type'] == 'int16':
                pack_fmt += 'h'
                length += 2
            elif value['type'] == 'uint16':
                pack_fmt += 'H'
                length += 2
            elif value['type'] == 'double':
                pack_fmt += 'd'
                length += 8
            elif value['type'] == 'int64':
                pack_fmt += 'q'
                length += 8
            elif value['type'] == 'uint64':
                pack_fmt += 'Q'
                length += 8
            elif value['type'] == 'char':
                pack_fmt += 'c'
                length += 1
            elif value['type'] == 'uchar':
                pack_fmt += 'B'
                length += 1
            elif value['type'] == 'uint8':
                pack_fmt += 'B'
                length += 1
        len_fmt = '{0}B'.format(length)
        b = struct.pack(len_fmt, *payload)
        data = struct.unpack(pack_fmt, b)
        out = [(value['name'], data[idx]) for idx, value in enumerate(output_message['payload'])]
        data = collections.OrderedDict(out)
        return data

    def change_scale(self, output_message, data):
        '''change scaling if fild with a "Scaling" attribute.
            For example, unit of Latitude in payload is radian, but we are used to deg, so it's necessary to change scaling from radian to deg.
            returns: False when some fild changed failed, should check the rover.json.
                     True when successful.
        '''
        for value in output_message['payload']:
            try:
                Scaling = value['Scaling']
            except KeyError:  # if there is no 'Scaling' key.
                continue

            try:
                data[value['name']] = data[value['name']] * eval(Scaling)
            except Exception as e:  # value['name']  or Scaling is incorrect.
                print(e)
                return False
        return True

    def handle_msg_01(self, frame):
        self.parse_frame(frame)

    def handle_msg_01_(self, frame):
        '''Kalman Filter Navigation Message
        '''
        system_time = struct.unpack('<d', struct.pack('8B', *(frame[6:14])))[0]
        GPS_time = struct.unpack('<d', struct.pack('8B', *(frame[14:22])))[0]
        latitude = (struct.unpack('<d', struct.pack('8B', *(frame[22:30])))[0]) * 180/math.pi
        longitude = struct.unpack('<d', struct.pack('8B', *(frame[30:38])))[0] * 180/math.pi
        ellipsoidal_height = struct.unpack('<d', struct.pack('8B', *(frame[38:46])))[0]
        velocity_north = struct.unpack('<d', struct.pack('8B', *(frame[46:54])))[0]
        velocity_east = struct.unpack('<d', struct.pack('8B', *(frame[54:62])))[0]
        velocity_down = struct.unpack('<d', struct.pack('8B', *(frame[62:70])))[0]
        roll = struct.unpack('<d', struct.pack('8B', *(frame[70:78])))[0]
        pitch = struct.unpack('<d', struct.pack('8B', *(frame[78:86])))[0]
        heading = struct.unpack('<d', struct.pack('8B', *(frame[86:94])))[0]
        position_mode = frame[94]
        velocity_mode = frame[95]
        attitude_status = frame[96]
        print('\r\n{0} Kalman Filter Navigation Message:'.format(datetime.datetime.now().strftime('%Y%m%d_%H_%M_%S')))
        # print msg
        print('system_time:{0} GPS_time: {1} latitude: {2} longitude: {3} ellipsoidal_height: {4} \
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

    def handle_msg_0A(self, data):
        print('{0}:Repackaged GSV Message'.format(datetime.datetime.now().strftime('%Y%m%d_%H_%M_%S')))


def main():
    '''main'''
    driver = RoverDriver()
    while True:
        try:
            driver.reinit()
            driver.find_device()
        except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
            print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
            break
        if not driver.start_log():
            break


if __name__ == '__main__':
    main()
