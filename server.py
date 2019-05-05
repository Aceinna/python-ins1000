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
import file_storage
import utility


class WSHandler(tornado.websocket.WebSocketHandler):
    clients = [] # reserve all web clients handlers.
    start_stream = False
    def __init__(self, *args, **kwargs):
        tornado.websocket.WebSocketHandler.__init__(self, *args, **kwargs)

        self.newest_packets = {}
        self.all_packets = []
        self.msgs_send2web = []

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
        data_lock.acquire()
        for p in self.all_packets:
            tp = p['data']['packetType']
            # if not self.start_stream \
            #     and (tp == 'SSS' or tp == 'GSVM' or tp == 'SS' \
            #         or tp == 'KNF' or tp == 'TSM' or tp == 'GH' ):
            if not self.start_stream:
                continue
            json_msg = json.dumps(p)
            self.write_message(json_msg)
            # print(json_msg)
            # print('************')
        self.all_packets = []

        if self.start_stream:
            for i, (k, v) in enumerate(self.newest_packets.items()):
                json_msg = json.dumps(v)
                self.write_message(json_msg)
                # print(json_msg)
                # print('#############')
            self.newest_packets.clear()
        data_lock.release()

    def on_message(self, message):
        # if driver.connection_status == 0:
        #     json_msg = json.dumps({ 'messageType' : 'event',  'data' : {'packetType' : 'ConnectionStatus', 'packet' : {'ConnectionStatus' : 0} }})
        #     self.write_message(json_msg)
        #     return
        data_lock.acquire()
        try:
            message = json.loads(message)
            if message['messageType'] == 'operation' and message['data']['packetType'] == 'StartLog':
                rev = rover_log.start_user_log(message['data']['packet']['userID'],message['data']['packet']['fileName'])
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StartLog','packet':{'returnStatus':rev}}})
                self.write_message(json_msg)
            elif message['messageType'] == 'operation' and message['data']['packetType'] == 'StopLog':
                rev = rover_log.stop_user_log(message['data']['packet']['accessToken'], message['data']['packet']['sasToken'])
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StopLog','packet':{'returnStatus':rev}}})
                # print(json_msg)
                # print('************') 
                self.write_message(json_msg)
            elif message['messageType'] == 'operation' and message['data']['packetType'] == 'StartStream':
                self.start_stream = True
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StartStream','packet':{'returnStatus':0}}})
                self.write_message(json_msg)
            elif message['messageType'] == 'operation' and message['data']['packetType'] == 'StopStream':
                self.start_stream = False
                json_msg = json.dumps({'messageType':'operationResponse','data':{'packetType':'StartStream','packet':{'returnStatus':0}}})
                self.write_message(json_msg)
            else:
                driver.handle_cmd_msg(message)
            # if message['messageType'] != 'serverStatus' and 'data' in message:
            #     file_storage.RoverLogApp(message['data'])
        except Exception as e:
                print(e)
        data_lock.release()

    def on_close(self):
        try:
            self.callback.stop()
        except Exception as e:
            pass
        if self in self.clients:
            self.clients.remove(self) # remove the closed web client.
            driver.remove_client(self)
        return False

    def check_origin(self, origin):
        return True

    def on_driver_message(self, packet):
        data_lock.acquire()
        # print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), packet))
        packet_type = packet['data']['packetType']

        if packet_type in self.msgs_send2web:
            if packet_type == 'NAV': 
                self.newest_packets[packet_type] = packet
            else: 
                # p = {packet_type:packet}
                self.all_packets.append(packet)
        data_lock.release()
    

class DataReceiver(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
        '''Init
        '''
        pass

    def on_reinit(self):
        pass

    def on_find_active_rover(self):
        pass

    def on_message(self, *args):
        data_lock.acquire()
        packet_type = args[0]
        data = args[1]
        is_var_len_frame = args[2]

        # user could configure which msgs can be saved to file or not in rover.json.
        if packet_type in rover_log.msgs_need_to_log:
            if is_var_len_frame:
                rover_log.log_var_len(data, packet_type)
                # print (json.dumps(data))
            else:
                rover_log.log(data, packet_type)
        data_lock.release()

    def on_exit(self):
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
    data_lock = threading.Lock()

    try:
        driver = driver_ins1000.RoverDriver()
        data_receiver = DataReceiver()
        driver.set_app(data_receiver)
        threading.Thread(target=driver_thread, args=(driver,)).start()
        rover_log = file_storage.RoverLogApp()  # log data.
        threading.Thread(target=start_websocket_server, args=()).start()
        while (True):
            time.sleep(1)
    except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
        print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
        os._exit(1)
    except Exception as e:
        print(e)
