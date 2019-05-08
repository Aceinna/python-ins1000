# -*- coding: utf-8 -*
import sys
import os
import datetime
import time
import threading
import json
import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.web
import rover_application_base
import driver_ins1000
from file_storage import RoverLogApp, FileLoger, FileUploader
import utility


class WSHandler(tornado.websocket.WebSocketHandler):
    clients = [] # reserve all web clients handlers.
    
    def __init__(self, *args, **kwargs):
        tornado.websocket.WebSocketHandler.__init__(self, *args, **kwargs)

        self.newest_packets = {}
        self.all_packets = []
        self.msgs_send2web = []
        self.data_lock = threading.Lock() # lock of all_packets and msgs_send2web.
        self.start_stream = False
        self.file_uploader = FileUploader()
        self.file_loger = FileLoger()
        self.file_loger_lock = threading.Lock() # lock of file_loger and file_uploader.
        self.ii = 0

        rover_properties = utility.load_configuration(os.path.join('setting', 'rover.json'))
        if not rover_properties:
            os._exit(1)
        for packet in rover_properties['userMessages']['outputPackets']:
            if 1 == packet['send2web']:
                self.msgs_send2web.append(packet['name'])

    def open(self):
        # if len(self.clients) > 0:
        #     self.close()
        #     return
        self.clients.append(self)
        driver.add_client(self)
        self.callback = tornado.ioloop.PeriodicCallback(self.send_data, callback_rate)
        self.callback.start()

    def send_data(self):
        self.data_lock.acquire()
        for p in self.all_packets:
            tp = p['data']['packetType']
            if not self.start_stream \
                and (tp == 'SSS' or tp == 'GSVM' or tp == 'SS' \
                    or tp == 'KNF' or tp == 'TSM' or tp == 'GH' ):
                continue
            json_msg = json.dumps(p)
            self.write_message(json_msg)
            # print(json_msg)
            # print('************')
        self.all_packets = []

        for i, (k, v) in enumerate(self.newest_packets.items()):
            if not self.start_stream and (k == 'NAV' or k == 'CNM'):
                continue
            json_msg = json.dumps(v)
            self.write_message(json_msg)
            # print(json_msg)
            # print('#############')
        self.newest_packets.clear()
        self.data_lock.release()

    def on_message(self, message):
        #driver report DeviceStatus msg if driver hasn't connected with device.
        if driver.connection_status == 0:
            json_msg = json.dumps({ 'messageType' : 'queryResponse', 'data' : {'packetType' : 'DeviceStatus', 'packet' : {'returnStatus' : 1} }})
            self.write_message(json_msg)
            # return
        try:
            message = json.loads(message)
            if message['messageType'] == 'operation' and message['data']['packetType'] == 'StartLog':
                status = self.start_log(message['data']['packet']['fileName'])
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StartLog','packet':{'returnStatus':status}}})
                self.write_message(json_msg)
                self.file_uploader.set_user_id(message['data']['packet']['userID'])
                self.file_uploader.set_user_access_token(message['data']['packet']['accessToken'])
            elif message['messageType'] == 'operation' and message['data']['packetType'] == 'StopLog':
                status = self.stop_log()
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StopLog','packet':{'returnStatus':status}}})
                self.write_message(json_msg)
            elif message['messageType'] == 'operation' and message['data']['packetType'] == 'StartStream':
                self.start_stream = True
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StartStream','packet':{'returnStatus':0}}})
                self.write_message(json_msg)
            elif message['messageType'] == 'operation' and message['data']['packetType'] == 'StopStream':
                self.start_stream = False
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StopStream','packet':{'returnStatus':0}}})
                self.write_message(json_msg)
            elif message['messageType'] == 'query' and message['data']['packetType'] == 'DeviceStatus':
                status = 0 if driver.connection_status == 1 else 1
                json_msg = json.dumps({ 'messageType' : 'queryResponse',  'data' : {'packetType' : 'DeviceStatus', 'packet' : {'returnStatus' : status} }})
                self.write_message(json_msg)
            else:
                driver.handle_cmd_msg(message)
        except Exception as e:
                print(e)

    def on_close(self):
        try:
            self.callback.stop()
        except Exception as e:
            pass

        if self in self.clients:
            self.clients.remove(self) # remove the closed web client.
            driver.remove_client(self)

        self.stop_log()
        return False

    def check_origin(self, origin):
        return True

    def on_driver_message(self, packet, ori_packet=None, is_var_len_frame=False):
        packet_type = packet['data']['packetType']
        # if packet_type == 'CNM':
        #     self.ii = self.ii + 1
        #     print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%S'), self.ii))
        self.data_lock.acquire()
        if packet_type in self.msgs_send2web:
            if packet_type == 'NAV': 
                self.newest_packets[packet_type] = packet
            else:
                self.all_packets.append(packet)
        self.data_lock.release()
        if ori_packet:
            self.update_log(ori_packet, packet_type, is_var_len_frame)

    def start_log(self, file_name):
        self.file_loger_lock.acquire()
        status = self.file_loger.start_user_log(file_name)
        self.file_loger_lock.release()
        return status

    def stop_log(self):
        self.file_loger_lock.acquire()
        # invoke get_log_file_names before invoking stop_user_log as stop_user_log will remove log_file_names.
        log_files = self.file_loger.get_log_file_names()
        status = self.file_loger.stop_user_log()
        if status == 0:
            self.file_uploader.upload(log_files)
        self.file_loger_lock.release()
        return status

    def update_log(self, packet, packet_type, is_var_len_frame):
        self.file_loger_lock.acquire()
        self.file_loger.update(packet, packet_type, is_var_len_frame)
        self.file_loger_lock.release()

class DataReceiver(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
        '''Init
        '''
        self.file_loger = FileLoger()
        self.file_loger.start_user_log()
        self.ii = 0
        pass

    def on_reinit(self):
        pass

    def on_find_active_rover(self):
        pass

    def on_message(self, *args):
        packet_type = args[0]
        packet = args[1]
        is_var_len_frame = args[2]
        self.file_loger.update(packet, packet_type, is_var_len_frame)
        # print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%S'), packet_type))
        if packet_type == 'NAV':
            self.ii = self.ii + 1
            if self.ii % 1000 == 0:
                print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%S'), self.ii))

    def on_exit(self):
        self.file_loger.stop_user_log()
        pass

def driver_thread(driver):
    while True:
        try:
            driver.reinit()
            if not driver.find_device():
                break # exit when user trigger KeyboardInterrupt.
        except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
            print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
            break
        if not driver.start_collection():
            break

def start_websocket_server():
    try:
        if sys.version_info[0] > 2:
            import asyncio
            asyncio.set_event_loop(asyncio.new_event_loop()) # bug fix:"There is no current event loop in thread 'Thread-n'." ref:https://www.wanghaiqing.com/article/daa4cde1-b6b7-4f8c-92a5-99f0dce80d6c/
        application = tornado.web.Application([(r'/', WSHandler)])
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(8000)
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
        print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
        os._exit(1)
    except Exception as e:
        print(e)


if __name__ == '__main__':
    '''main'''
    callback_rate = 1000

    try:
        driver = driver_ins1000.RoverDriver()
        data_receiver = DataReceiver()
        driver.set_app(data_receiver)
        threading.Thread(target=driver_thread, args=(driver,)).start()
        threading.Thread(target=start_websocket_server, args=()).start()
        while (True):
            time.sleep(1)
    except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
        print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
        os._exit(1)
    except Exception as e:
        print(e)
