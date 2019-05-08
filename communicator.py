# -*- coding: utf-8 -*
"""
Created on 2019-05-07
@author: Ocean
"""
import sys
import os
import socket
import select
import time
import datetime
import json
import glob
import serial
import serial.tools.list_ports

class Communicator():
    '''
    '''
    def __init__(self):
        self.setting_folder = os.path.join(os.getcwd(), r'setting')  # use to store some configuration files.
        self.connection_file = os.path.join(self.setting_folder, 'connection.json')
        self.read_size = 0
        pass

    def find_device(self):
        pass

    def open(self):
        pass

    def close(self):
        pass


class SerialPort(Communicator):
    def __init__(self):
        Communicator.__init__(self)
        self.ser = None  # the active UART
        self.port = None
        self.baud = None
        self.read_size = 100
        pass

    def find_device(self):
        ''' Finds active ports and then autobauds units
        '''
        try:
            if self.try_last_port():
                pass
            else:
                while not self.autobaud(self.find_ports()):
                    time.sleep(0.1)
            return True
        except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
            print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
            return False

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
        # port_list = list(serial.tools.list_ports.comports())
        # ports = [ p.device for p in port_list]
        if sys.platform.startswith('win'):
            port_list = list(serial.tools.list_ports.comports())
            ports = [ p.device for p in port_list]
            # ports = ['COM%s' % (i + 1) for i in range(256)]
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

    def open_serial_port(self, port=None, baud=115200, timeout=0.1):
        ''' open serial port
            returns: true when successful
        '''
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            return True
        except Exception as e:
            print('serial port open exception, port:{0} baud:{1} timeout:{2}'.format(port, baud, timeout) )
            self.ser = None
            return False

    def close_serial_port(self):
        '''close serial port
        '''
        if self.ser is not None:
            if self.ser.isOpen():
                self.ser.close()

    def find_header(self, data):
        '''
        sync.

        return: 
                True: Successful.
                False: Failed.
        '''
        rev = True if (data.find(b'\xAF\x20\x05') > -1  \
                    or data.find(b'\xAF\x20\x06') > -1  \
                    or data.find(b'\xAF\x20\x07') > -1) else False
        return rev

    def write(self,data):
        '''
        write the bytes data to the port

        return:
                length of data sent via serial port.
                False: Exception when sending data, eg. serial port hasn't been opened.
        '''
        try:
            return self.ser.write(data)
        except Exception as e:
            # print(e)
            raise

    def read(self,size):
        '''
        read size bytes from the serial port. 
        parameters: size – number of bytes to read.
        returns: bytes read from the port.
        return type: bytes
        '''
        try:
            return self.ser.read(size)
        except serial.SerialException:
            print('Serial Exception! Please check the serial port connector is stable or not.')
            raise   
        except Exception as e:
            # print(e)
            raise

    def open(self):
        return self.open_serial_port(self.port, self.baud, timeout=0.1)

    def close(self):
        return self.close_serial_port()


class TCPIP(Communicator):
    def __init__(self, host ='10.0.4.71', port=8888):#'127.0.0.1'  '192.168.31.223'
        Communicator.__init__(self)
        self.host = host
        self.port = port
        self.sock = None
        self.read_size = 1024
        pass

    def find_device(self):
        try:
            while (True):
                try:
                    print(datetime.datetime.now().strftime('[%Y_%m_%d %H_%M_%S]:') + 'connecting {0}:{1} ...'.format(self.host,self.port))
                    self.open()
                    print(datetime.datetime.now().strftime('[%Y_%m_%d %H_%M_%S]:') + 'connect {0}:{1} successfully.'.format(self.host,self.port))
                    # self.close()
                    return True
                except socket.error :
                    time.sleep(1)
                except Exception as e:
                    print(e)
        except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
            print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
            return False

    def write(self,data):
        '''
        write the bytes data to host.

        return:
                length of data sent via TCPIP.
                False: Exception when sending data, eg. host has closed.
        '''
        try:
            return self.sock.send(data)
        except socket.error :
            print ("socket error,do reconnect.")
            raise
        except Exception as e:
            print(e)
            raise

    def read(self,size):
        '''
        read size bytes via TCPIP. 
        parameters: size – number of bytes to read.
        returns: bytes read from the port.
        return type: bytes
        '''
        try:
            flag = True
            to_read, to_write, in_error = select.select([self.sock], [self.sock], [], 0.001) 
            if self.sock not in to_read: 
                flag = False

            data = self.sock.recv(size)
            if not flag and len(data) == 0:
                raise socket.error ('Can not connect to server[{0}:{1}]'.format(self.host, self.port))#server closed.
            else:
                return data
        except socket.error as e:
            print(e)
            print ("socket error,do reconnect.")
            raise
        except Exception as e:
            print(e)
            raise
        except :
            raise

    def open(self):
        if self.sock:
            return True

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.host, self.port))
            return True
        except socket.error :
            self.sock = None
            raise
        except socket.timeout as e:
            print(e)
        except Exception as e:
            self.sock = None
            raise

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
