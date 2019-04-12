# -*- coding: utf-8 -*
"""
Driver for INS1000 Rover.
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
from message import Msg_060C_topic_0A
if sys.version_info[0] > 2:
    from queue import Queue
else:
    from Queue import Queue
import utility


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
        self.web_clients_lock = threading.Lock()  # lock of data_queue
        self.app = None
        self.web_clients = []
        self.msgs = {} 
        self.cmds = {}
        self.cmds['queryProductId'] = b'\xAF\x20\x06\x0B\x01\x00\x01\x01\x01'
        self.cmds['queryEngineVersion'] = b'\xAF\x20\x06\x0B\x01\x00\x02\x02\x02'
        self.cmds['queryFirmwareVersion'] = b'\xAF\x20\x06\x0B\x01\x00\x0C\x0C\x0C'
        self.cmds['queryInternalLeverArm'] = b'\xAF\x20\x06\x0B\x01\x00\x04\x04\x04'
        self.cmds['queryIMURotationMatrix'] = b'\xAF\x20\x06\x0B\x01\x00\x0A\x0A\x0A'
        self.cmds['queryGNSSAntennaLeverArm'] = self.cmds['queryIMURotationMatrix']
        self.nav_pos_vel_mode = {0:'INVALID', 1:'DEAD_RECKON', 2:'STAND_ALONE', 3:'PRECISE_POINT_POSITIONING', 4:'CODE_DIFF', 5:'RTK_FLOAT', 6:'RTK_FIXED', 7:'USER_AIDING'}
        self.nav_att_mode = {0:'INVALID', 1:'COARSE', 2:'FINE'}
        self.setting_folder = os.path.join(os.getcwd(), r'setting')  # use to store some configuration files.
        self.connection_file = os.path.join(self.setting_folder, 'connection.json')
        self.rover_properties = utility.load_configuration(os.path.join(self.setting_folder, 'rover.json'))
        if not self.rover_properties:
            os._exit(1)
        print('Rover driver start at:{0}'.format(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')))

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
        if self.app:
            self.app.on_reinit()
        self.msgs.clear()

    def set_app(self, app):
        self.app = app

    def add_client(self, client):
        self.web_clients_lock.acquire()
        self.web_clients.append(client)
        self.web_clients_lock.release()

    def remove_client(self, client):
        self.web_clients_lock.acquire()
        self.web_clients.remove(client)
        self.web_clients_lock.release()

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
                print('Serial Exception! Please check the serial port connector is stable or not.')
                self.exit_lock.acquire()
                self.exit_thread = True  # Notice thread paser to exit.
                self.exit_lock.release()
                return  # exit thread receiver

            if len(serial_data):
                # print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:') + ' '.join('0X{0:x}'.format(serial_data[i]) for i in range(len(serial_data))))
                self.data_lock.acquire()
                for d in serial_data:
                    self.data_queue.put(d)
                self.data_lock.release()
            else:
                time.sleep(0.001)

    def parser(self):
        ''' get serial data from data_queue and parse data into one whole frame.
            return when occur SerialException in thread receiver.
        '''
        HEADER_05 = [0XAF, 0X20, 0X05]
        HEADER_06 = [0XAF, 0X20, 0X06]
        HEADER_07 = [0XAF, 0X20, 0X07]
        PAYLOAD_LEN_IDX = 4
        MSG_SUB_ID_IDX = 3
        MAX_FRAME_LIMIT = 500  # assume max len of frame is smaller than MAX_FRAME_LIMIT.

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
                time.sleep(0.001)
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
                        result = utility.check_sum(frame[PAYLOAD_LEN_IDX + 2:PAYLOAD_LEN_IDX + payload_len + 2])
                        if result[0] == frame[-2] and result[1] == frame[-1]:
                            # find a whole frame
                            self.parse_frame(frame)
                        else:
                            print("Checksum error!")
                    else:
                        pass

                    if payload_len > MAX_FRAME_LIMIT or len(frame) > MAX_FRAME_LIMIT:
                        find_header = False
                        payload_len = 0

                else:  # if hasn't found header [0XAF, 0X20, 0X05] or [0XAF, 0X20, 0X07].
                    sync_pattern.append(data)
                    if operator.eq(list(sync_pattern), HEADER_05):
                        frame = HEADER_05[:]  # header_tp.copy()
                        find_header = True
                    elif operator.eq(list(sync_pattern), HEADER_06):
                        frame = HEADER_06[:]
                        find_header = True
                    elif operator.eq(list(sync_pattern), HEADER_07):
                        frame = HEADER_07[:]
                        find_header = True
                    else:
                        pass

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

    def start_collection(self):
        ''' start two threads: receiver and parser.
            returns False when user trigger KeyboardInterrupt to stop this program.
            otherwise returns True.
        '''
        self.open_serial_port(self.port, self.baud, 0.1)
        if not self.ser:
            return True

        funcs = [self.receiver, self.parser]
        for func in funcs:
            t = threading.Thread(target=func, args=())
            t.start()
            print("Thread[{0}({1})] started.".format(t.name, t.ident))
            self.threads.append(t)

        # query user configurations setup
        self.write(self.cmds['queryInternalLeverArm'])
        time.sleep(0.1)
        self.write(self.cmds['queryIMURotationMatrix'])

        if self.handle_KeyboardInterrupt():
            return False

        for t in self.threads:
            t.join()
            print("Thread[{0}({1})] stoped.".format(t.name, t.ident))

        self.close_serial_port()
        return True

    def find_device(self):
        ''' Finds active ports and then autobauds units
        '''
        try:
            if self.try_last_port():
                pass
            else:
                while not self.autobaud(self.find_ports()):
                    time.sleep(0.1)

            if self.app:
                self.app.on_find_active_rover()
            return True
        except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
            print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))

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

    def find_header(self, data):
        rev = True if (data.find(b'\xAF\x20\x05') > -1 or data.find(b'\xAF\x20\x06') > -1 or data.find(b'\xAF\x20\x07') > -1) else False
        return rev



    def get_packet_type(self):
        return self.packet_type

    def set_packet_type(self, packet_type):  # consider add lock when other thread invoke this function.
        self.packet_type = packet_type

    def parse_frame(self, frame):
        '''Parses packet payload using rover.json as reference
        '''
        PAYLOAD_LEN_IDX = 4
        payload_len = 256 * frame[PAYLOAD_LEN_IDX + 1] + frame[PAYLOAD_LEN_IDX]
        payload = frame[6:payload_len+6]   # extract the payload
        header = ''.join(["%02X" % x for x in frame[0:PAYLOAD_LEN_IDX]]).strip()
        data = []

        # Find the packet properties from Rover's JSON description
        output_packet = next((x for x in self.rover_properties['userMessages']['outputPackets'] if x['header'] == header), None)
        input_packet = next((x for x in self.rover_properties['userMessages']['inputPackets'] if x['header'] == header), None)

        if output_packet:
            var_num = None
            is_var_len_frame = False
            try:
                var_num = output_packet['var_num']
                is_var_len_frame = True
            except KeyError:  # if there is no 'var_num' key.
                pass

            if var_num is not None:  # means this is a variable length frame, such as "Satellite Signal Strength" and "SV Visibility"
                # time_start = time.time()
                data = self.unpack_output_packet_var_len(output_packet, payload)
                # time_end = time.time()
                # print('[{0}]:unpack_output_packet_var_len cost:{1}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), time_end-time_start))
                pass
            else:
                data = self.unpack_output_packet(output_packet, payload, payload_len)
                self.change_scale(output_packet, data)
            if data:
                if self.app:
                    self.app.on_message(output_packet['name'], data, is_var_len_frame)

                self.web_clients_lock.acquire()
                for client in self.web_clients:
                    client.on_driver_message(output_packet['name'], data, is_var_len_frame)
                self.web_clients_lock.release()

                self.packet_handler(output_packet['name'], data, is_var_len_frame)
        elif header.startswith('AF2006'): # some messages start with 'AF2006' are different decoding rule with message list in rover.json, so need to handle specially.
            self.msg_typeid_06_handler(frame)
        elif input_packet:
            # data = self.unpack_input_packet(input_packet['responsePayload'], payload)
            pass
        else:
            pass
        return data
    
    def msg_typeid_06_handler(self, frame):
        '''

        '''
        PAYLOAD_SUB_ID_IDX = 3
        PAYLOAD_LEN_IDX = 4
        PAYLOAD_TOPIC_IDX = 6
        payload_len = 256 * frame[PAYLOAD_LEN_IDX + 1] + frame[PAYLOAD_LEN_IDX]
        payload = frame[6:payload_len+6]   # extract the payload
        # header = ''.join(["%02X" % x for x in frame[0:PAYLOAD_LEN_IDX]]).strip()
        sub_id = frame[PAYLOAD_SUB_ID_IDX]
        topic_tp = frame[PAYLOAD_TOPIC_IDX]
        data = []

        # ref msg8.4, rover report the 'IMU rotation matrix' and 'GNSS antenna lever arm' messages.
        if sub_id == 0X0C and topic_tp == 0X0A: 
            msg_060C_topic_0A = Msg_060C_topic_0A(frame)
            # msg_060C_topic_0A.pack_msg_060D()
            self.msgs['Msg_060C_topic_0A'] = msg_060C_topic_0A
            imu_rotation_matrix = msg_060C_topic_0A.get_imu_rotation_matrix()
            # json_msg = json.dumps({ 'messageType' : 'event',  'data' : {'packetType' : 'IMURotationMatrix', 'packet' : imu_rotation_matrix }})
            '''
            todo:
            1. update new msg_060C_topic_0A in self.msgs
            2. get_imu_rotation_matrix and send to web
            3. get_GNSS_lever_arm_center and send to web
            '''
            if self.app:
                self.app.on_message("IMURotationMatrix", imu_rotation_matrix, False)
                # wrt_imu_center,lever_arm = msg_060C_topic_0A.get_GNSS_lever_arm_center()
            self.web_clients_lock.acquire()
            for client in self.web_clients:
                client.on_driver_message("IMURotationMatrix", imu_rotation_matrix, False)
            self.web_clients_lock.release()
        elif sub_id == 0X0C and topic_tp == 0X0C: # Firmware version
            len_fmt = '{0}B'.format(payload_len)
            b = struct.pack(len_fmt, *payload)
            data = collections.OrderedDict()
            data['Firmware Version'] = b.decode() # bytes to string
            if self.app:
                self.app.on_message( "FV", data, False)

            self.web_clients_lock.acquire()
            for client in self.web_clients:
                client.on_driver_message( "FV", data, False)
            self.web_clients_lock.release()   
        elif sub_id == 0X0C and topic_tp == 0X04: # handle internal lever-arm message.
            pass
        else:
            pass

        # from pprint import pprint
        # pprint (vars(msg_060C_topic_0A))
        # print('********************')
        
    def handle_string_filed(self, value, payload, string_len):
        '''
            handle_string_filed is used to parse one type of message which Payload is only one string field, 
            such as message 5.18(Engine Version Message) and 6.1(Text Message).
        '''
        pack_fmt = string_len*'c'
        len_fmt = '{0}B'.format(string_len)
        b = struct.pack(len_fmt, *payload)
        data = struct.unpack(pack_fmt, b)
        out = [(value['name'], b.decode())] # bytes to string
        data = collections.OrderedDict(out)
        return data

    def unpack_output_packet(self, output_message, payload, payload_len):
        length = 0
        pack_fmt = '<'
        for value in output_message['payload']:
            if value['type'] == 'string':
                return self.handle_string_filed(value, payload, payload_len)
            elif value['type'] == 'float':
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

    def unpack_output_packet_var_len(self, output_message, payload):
        length = 0
        pack_fmt = '<'
        var_num_type = output_message['var_num']['type']
        var_num_idx = output_message['var_num']['idx']
        var_num_field_idx = output_message['var_num']['field_idx']
        p = []

        if var_num_type == 'uint8':
            pack_fmt += 'B'
            length = 1
            p = payload[var_num_idx: var_num_idx+length]
        elif var_num_type == 'uint16':
            pack_fmt += 'H'
            length = 2
            p = payload[var_num_idx: var_num_idx+length]
        elif var_num_type == 'uint32':
            pack_fmt += 'I'
            length = 4
            p = payload[var_num_idx: var_num_idx+length]
        elif var_num_type == 'uint64':
            pack_fmt += 'Q'
            length = 8
            p = payload[var_num_idx: var_num_idx+length]

        len_fmt = '{0}B'.format(length)
        b = struct.pack(len_fmt, *p)
        var_num = struct.unpack(pack_fmt, b)  # eg. var_num is N_SV in "Satellite Signal Strength" frame
        if var_num[0] == 0:
            return []

        idx = 0
        var_len_one_gropu = 0
        var_pack_fmt = None

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

            idx += 1
            if var_num_field_idx == idx:
                var_len_one_gropu = length
                var_pack_fmt = pack_fmt

        idx = 0
        field_names = []
        for idx, value in enumerate(output_message['payload']):
            field_names.append(value['name'])

        var_len_one_gropu = length - var_len_one_gropu  #eg. for "Satellite Signal Strength", var_len_one_gropu is 10. (SV system [1 Byte] + SVID [1 Byte] + L1CN0 [foat 4 Bytes] + L2CN0 [foat 4 Bytes])
        length += var_len_one_gropu * (var_num[0]-1)

        var_pack_fmt = pack_fmt[len(var_pack_fmt):len(pack_fmt)]  #eg. for "Satellite Signal Strength", var_pack_fmt is "BBff".
        var_fileld_num = len(var_pack_fmt)  #eg. for "Satellite Signal Strength", var_fileld_num is 4, the number of variable filelds.
        const_fileld_num = len(pack_fmt) - var_fileld_num - 1 #eg. for "Satellite Signal Strength", const_fileld_num is 4, the number of const filelds.
        pack_fmt += var_pack_fmt * (var_num[0]-1)

        # const_fileld_names = field_names[0:var_fileld_num+1]  #eg. for "Satellite Signal Strength", const_fileld_names is ['System time','GPS time','Receiver_ID','Antenna_ID','N_SV']
        var_fileld_names = field_names[var_fileld_num+1:len(pack_fmt)]  #eg. for "Satellite Signal Strength", var_fileld_names is ['SV_system','SVID','L1CN0','L2CN0']
        field_names += var_fileld_names * (var_num[0]-1)

        len_fmt = '{0}B'.format(length)
        b = struct.pack(len_fmt, *payload)
        payload_data = struct.unpack(pack_fmt, b)

        data = []
        info = collections.OrderedDict()
        for idx, value in enumerate(field_names):
            if idx < const_fileld_num:
                info[value] = payload_data[idx]
                if idx == const_fileld_num-1:
                    data.append(info.copy())
                    info.clear()
            else:
                info[value] = payload_data[idx]
                if len(info) == var_fileld_num:
                    data.append(info.copy())
                    info.clear()

        # print (json.dumps(data))
        # print("***************")
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

    def packet_handler(self, packet_type, packet, is_var_len):
        '''
        Driver has to construct some packets based on existing packets,
        such as 'NAV' (navigation information) packet which based on 'KFN','CNM' and 'GH'.
        '''
        TP_KFN = 'KFN' # msg tpyt is KFN
        TP_CNM = 'CNM'
        TP_GH = 'GH'
        TP_NAV = 'NAV'
        TP_SA = 'SA'
        TP_NCA = 'NCA'
        TP_SS = 'SS'

        # self.msgs is used to restore the newest packets.
        if TP_KFN == packet_type:
            self.msgs[TP_KFN] = packet
        elif TP_CNM == packet_type:
            self.msgs[TP_CNM] = packet
        elif TP_GH == packet_type:
            self.msgs[TP_GH] = packet
        elif TP_SA == packet_type:
            self.msgs[TP_SA] = packet
        elif TP_NCA == packet_type:
            self.msgs[TP_NCA] = packet
        else:
            pass

        # make sure have received 'KFN', 'CNM' and 'GH' already before construct 'NAV'
        if TP_KFN in self.msgs and TP_CNM in self.msgs and TP_GH in self.msgs and TP_CNM == packet_type:
            r2d = 180/math.pi
            pos_rms_n = self.msgs[TP_CNM]['Position RMS-N']
            pos_rms_e = self.msgs[TP_CNM]['Position RMS-E']
            pos_rms_d = self.msgs[TP_CNM]['Position RMS-D']
            vel_rms_n = self.msgs[TP_CNM]['Velocity RMS-N']
            vel_rms_e = self.msgs[TP_CNM]['Velocity RMS-E']
            vel_rms_d = self.msgs[TP_CNM]['Velocity RMS-D']
            att_rms_n = self.msgs[TP_CNM]['Attitude RMS-N']
            att_rms_e = self.msgs[TP_CNM]['Attitude RMS-E']
            att_rms_d = self.msgs[TP_CNM]['Attitude RMS-D']
            q0 = self.msgs[TP_CNM]['Attitude quaternion-Scalar']
            q1 = self.msgs[TP_CNM]['Attitude quaternion-X']
            q2 = self.msgs[TP_CNM]['Attitude quaternion-Y']
            q3 = self.msgs[TP_CNM]['Attitude quaternion-Z']
            euler_angle = utility.cal_attitude(q0, q1, q2, q3)

            nav = collections.OrderedDict()
            nav['System time'] = self.msgs[TP_CNM]['System time']
            nav['GPS week number'] = self.msgs[TP_CNM]['GPS week number']
            nav['GPS time'] = self.msgs[TP_KFN]['GPS time']
            nav['Position mode'] = self.nav_pos_vel_mode[self.msgs[TP_KFN]['Position mode']]
            nav['Latitude'] = self.msgs[TP_CNM]['Latitude']
            nav['Longitude'] = self.msgs[TP_CNM]['Longitude']
            nav['Ellipsoidal height'] = self.msgs[TP_CNM]['Ellipsoidal height']
            nav['MSL height'] = self.msgs[TP_CNM]['Ellipsoidal height'] - self.msgs[TP_GH]['Geoid height']
            nav['Position RMS-N'] = pos_rms_n
            nav['Position RMS-E'] = pos_rms_e
            nav['Position RMS-D'] = pos_rms_d
            nav['Position RMS'] = math.sqrt(math.pow(pos_rms_n,2)+math.pow(pos_rms_e,2)+math.pow(pos_rms_d,2))
            nav['Velocity mode'] = self.nav_pos_vel_mode[self.msgs[TP_KFN]['Velocity mode']]
            nav['Velocity_N'] = self.msgs[TP_CNM]['Velocity_N']
            nav['Velocity_E'] = self.msgs[TP_CNM]['Velocity_E']
            nav['Velocity_D'] = self.msgs[TP_CNM]['Velocity_D']
            nav['Velocity RMS-N'] = vel_rms_n
            nav['Velocity RMS-E'] = vel_rms_e
            nav['Velocity RMS-D'] = vel_rms_d
            nav['Velocity RMS'] = math.sqrt(math.pow(vel_rms_n,2)+math.pow(vel_rms_e,2)+math.pow(vel_rms_d,2))
            nav['Attitude status'] = self.nav_att_mode[self.msgs[TP_KFN]['Attitude status']]
            nav['Roll'] = euler_angle[0]*r2d
            nav['Pitch'] = euler_angle[1]*r2d
            nav['Heading'] = euler_angle[2]*r2d
            nav['Attitude RMS-N'] = att_rms_n
            nav['Attitude RMS-E'] = att_rms_e
            nav['Attitude RMS-D'] = att_rms_d
            nav['Attitude RMS'] = math.sqrt(math.pow(att_rms_n,2)+math.pow(att_rms_e,2)+math.pow(att_rms_d,2))

            if self.app:
                self.app.on_message(TP_NAV, nav, False)

            self.web_clients_lock.acquire()
            for client in self.web_clients:
                client.on_driver_message(TP_NAV, nav, False)
            self.web_clients_lock.release()                     
            # print (json.dumps(nav))
            # print("***************")
        # elif (TP_SA == packet_type and TP_NCA in self.msgs) \
        # or (TP_NCA == packet_type and TP_SA in self.msgs):
        
        elif TP_SA == packet_type:
            data = collections.OrderedDict()
            data['IMU Status'] = 1 if packet['IMU message count'] > 0 else 0
            data['GNSS Status'] = 1 if packet['GNSS message count'] > 0 else 0
            data['PPS Status'] = 1 if packet['PPS count'] > 0 else 0
            data['NTRIP Status'] = 0
            self.msgs[TP_SS] = data
            if self.app:
                self.app.on_message(TP_SS, data, False)

            self.web_clients_lock.acquire()
            for client in self.web_clients:
                client.on_driver_message(TP_SS, data, False)
            self.web_clients_lock.release()                    
        elif TP_NCA == packet_type:
            data = collections.OrderedDict()
            data['IMU Status'] = 0
            data['GNSS Status'] = 0
            data['PPS Status'] = 0
            data['NTRIP Status'] = 1 if packet['NTRIP message size'] > 0 else 0
            self.msgs[TP_SS] = data
            if self.app:
                self.app.on_message(TP_SS, data, False)
                
            self.web_clients_lock.acquire()
            for client in self.web_clients:
                client.on_driver_message(TP_SS, data, False)
            self.web_clients_lock.release()                    
        else:
            pass

    def handle_cmd_msg(self, message):
        '''
        Prase command message from web clint, 
        and send corresponding hex format command message to rover.
        '''
        CMD_TP_EVENT = 'event'
        CMD_TP_QUERY = 'query'
        CMD_TP_SET = 'set'
        CMD_PRODUCT_ID = 'ProductID'
        CMD_ENGINE_VERSION = 'EngineVersion'
        CMD_FIRMWARE_VERSION = 'FirmwareVersion'
        CMD_IMU_ROTATION_MATRIX = 'IMURotationMatrix'
        CMD_GNSS_ANTENNA_LEVER_ARM = 'GNSSAntennaLeverArm'

        if message['messageType'] == CMD_TP_QUERY:
            if message['data']['packetType'] == CMD_PRODUCT_ID:
                self.write(self.cmds['queryProductId'])
            if message['data']['packetType'] == CMD_ENGINE_VERSION:
                self.write(self.cmds['queryEngineVersion'])
            if message['data']['packetType'] == CMD_FIRMWARE_VERSION:
                self.write(self.cmds['queryFirmwareVersion'])
            if message['data']['packetType'] == CMD_IMU_ROTATION_MATRIX:
                self.write(self.cmds['queryIMURotationMatrix'])
            if message['data']['packetType'] == CMD_GNSS_ANTENNA_LEVER_ARM:
                self.write(self.cmds['queryGNSSAntennaLeverArm'])
        elif message['messageType'] == CMD_TP_SET:
            if message['data']['packetType'] == CMD_IMU_ROTATION_MATRIX:
                matrix = list(message['data']['packet']['C{0}'.format(i)] for i in range(9))
                if 'Msg_060C_topic_0A' in self.msgs:
                    msg_060C_topic_0A = self.msgs['Msg_060C_topic_0A']
                    msg_060C_topic_0A.set_imu_rotation_matrix(matrix)
                    msg_060C_topic_0A.pack_msg_060D()
                    self.write(msg_060C_topic_0A.msg_060D_cmd)
                else:
                    self.write(self.cmds['queryIMURotationMatrix'])

            elif message['data']['packetType'] == CMD_GNSS_ANTENNA_LEVER_ARM:
                packet = message['data']['packet']
                antenna_num = packet[0]['Antenna_num']
                lever_arm_wrt_imu_center = True if (packet[0]['LeverArm_WRT'] == 'IMU Center') else False
                lever_arms = {}
                for i in range(1,antenna_num+1):
                    lever_arms[packet[i]['Antenna_ID']] = [packet[i]['LeverArm_X'],packet[i]['LeverArm_Y'],packet[i]['LeverArm_Z']]
                # ensure lever-arm is sorted by Antenna_ID.
                lever_arm = [lever_arms[k] for k in sorted(lever_arms.keys())] 
                if 'Msg_060C_topic_0A' in self.msgs['Msg_060C_topic_0A']:      
                    msg_060C_topic_0A = self.msgs['Msg_060C_topic_0A']
                    msg_060C_topic_0A.set_GNSS_lever_arm_center(lever_arm, lever_arm_wrt_imu_center)
                    msg_060C_topic_0A.pack_msg_060D()
                    self.write(msg_060C_topic_0A.msg_060D_cmd)
                else:
                    self.write(self.cmds['queryGNSSAntennaLeverArm'])

        pass

    def write(self,n):
            try:
                self.ser.write(n)
            except Exception as e:
                print(e)
                print('serial exception write')
                self.exit_lock.acquire()
                self.exit_thread = True  # Notice thread paser and receiver to exit.
                self.exit_lock.release()


if __name__ == '__main__':
    frame = bytearray(b'\xAF\x20\x06\x0C\xB2\x00\x0A\x0A\x01\x1C\x04\x3C\x00\x1B\x04\x00\x00\x00\x00\x00\x00\xF0\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xF0\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xF0\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x0B\x00\x00\x24\x00\x00\x00\x5F\xFD\xFF\xFF\x33\xF3\xFF\xFF\x24\x00\x00\x00\x5F\xFD\xFF\xFF\x01\x00\x01\x00\x01\x82\x02\x82\x03\x82\x04\x82\x05\x00\x06\x82\x07\x00\x08\x00\x09\x82\x0A\x80\x0B\x00\x0C\x00\x0D\x82\x0E\x00\x0F\x00\x10\x82\x11\x00\x12\x82\x13\x82\x14\x00\x15\x00\x16\x82\x17\x00\x18\x00\x19\x00\x1A\x00\x1B\x00\x1C\x00\xF9\x38')
    driver = RoverDriver()
    driver.msg_typeid_06_handler(frame)

    # message = '''{"messageType":"set","data":{"packetType":"IMURotationMatrix","packet":{"C0":1,"C1":0,"C2":0,"C3":0,"C4":1,"C5":0,"C6":0,"C7":0,"C8":1}}}'''
    # message = '''{"messageType":"set","data":{"packetType":"GNSSAntennaLeverArm","packet":[{"Antenna_num":2,"LeverArm_WRT":"IMU Center"},{"Antenna_ID":2,"LeverArm_X":0.4,"LeverArm_Y":0.5,"LeverArm_Z":-0.6},{"Antenna_ID":1,"LeverArm_X":0.1,"LeverArm_Y":0.2,"LeverArm_Z":-0.3}]}}'''
    # message = json.loads(message)
    # driver.handle_cmd_msg(message)

    # todo:
    # 1. set/get_GNSS_lever_arm_center 
    # 2. msg_typeid_06_handler 
    pass

