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


callback_rate = 100
data_lock = threading.Lock()  # used to protect data_receiver.data.


class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        self.callback = tornado.ioloop.PeriodicCallback(self.send_data, callback_rate)
        self.callback.start()

    def send_data(self):
        # time_start = time.time()
        # data_lock.acquire()
        # print(driver.data_queue.qsize())
        if data_receiver.data is not None:
            self.write_message(json.dumps({ 'messageType' : 'event',  'data' : { 'newOutput' : data_receiver.data }}))
            # print(json.dumps({ 'messageType' : 'event',  'data' : { 'newOutput' : data_receiver.data }}))
        # data_lock.release()
        # time_end = time.time()
        # print('[{0}]:send_data cost:{1}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), time_end-time_start))

    def on_message(self, message):
        return True

    def on_close(self):
        self.callback.stop()
        return False

    def check_origin(self, origin):
        return True

class DataReceiver(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
        '''Init
        '''
        self.data = None
        pass

    def on_reinit(self):
        pass

    def on_find_active_rover(self):
        pass

    def on_message(self, *args):
        data_lock.acquire()
        packet_type = args[0]
        self.data = args[1]
        is_var_len_frame = args[2]
        print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), packet_type))
        # print(self.data)
        if is_var_len_frame:
            rover_log.log_var_len(self.data, packet_type)
        else:
            rover_log.log(self.data, packet_type)
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
        print('User stop this program by KeyboardInterrupt! File:[{0}], Line:[{1}]'.format(__file__, sys._getframe().f_lineno))
        os._exit(1)
    except Exception as e:
            print(e)
