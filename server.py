import sys
import os
import datetime
import time
import threading
import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.web
import json
import rover_application_base
import driver_ins1000
import file_storage


class WSHandler(tornado.websocket.WebSocketHandler):
    users=[]
    def open(self):
        self.users.append(self)
        data_receiver.b_have_client = True
        self.callback = tornado.ioloop.PeriodicCallback(self.send_data, callback_rate)
        self.callback.start()

    def send_data(self):
        data_lock.acquire()
        if data_receiver.latest_packets is not None:
            for key in data_receiver.latest_packets:
                json_msg = json.dumps({ 'messageType' : 'event',  'data' : {'packetType' : key, 'newOutput' : data_receiver.latest_packets[key] }})
                self.write_message(json_msg)
            data_receiver.latest_packets.clear()
            for key in data_receiver.all_GSVM_packets:
                json_msg = json.dumps({ 'messageType' : 'event',  'data' : {'packetType' : 'GSVM', 'newOutput' : key }})
                self.write_message(json_msg)
            data_receiver.all_GSVM_packets = []
        data_lock.release()
        print(len(self.users) )
    def on_message(self, message):
        message = json.loads(message)
        if message['messageType'] != 'serverStatus':
            file_storage.RoverLogApp(message['data'])

    def on_close(self):
        self.callback.stop()
        data_lock.acquire()
        self.users.remove(self) # remove the closed web client from users.
        if len(self.users) == 0: # clear all packets if no web client connect with server.
            data_receiver.b_have_client = False
            data_receiver.latest_packets.clear()
            data_receiver.all_GSVM_packets = []
        data_lock.release()
        print(len(self.users) )
        return False

    def check_origin(self, origin):
        return True

class DataReceiver(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
        '''Init
        '''
        self.latest_packets = {}
        self.all_GSVM_packets = []
        self.b_have_client = False
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
        # print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), packet_type))
        if self.b_have_client:
            if packet_type == 'GSVM':
                self.all_GSVM_packets.append(data)
            else:
                self.latest_packets[packet_type] = data

        if is_var_len_frame:
            rover_log.log_var_len(data, packet_type)
        else:
            rover_log.log(data, packet_type)
        data_lock.release()

    def on_exit(self):
        pass


def driver_thread(driver):
    while True:
        try:
            driver.reinit()
            driver.find_device()
        except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
            print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
            break
        if not driver.start_collection():
            break


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

        application = tornado.web.Application([(r'/', WSHandler)])
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(8000)
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:  # response for KeyboardInterrupt such as Ctrl+C
        os.remove("tempLogFiles.json")
        print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
        os._exit(1)
    except Exception as e:
            print(e)
