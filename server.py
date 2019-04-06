# -*- coding: utf-8 -*
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
import utility


class WSHandler(tornado.websocket.WebSocketHandler):
    # def __init__(self, *args, **kwargs):
    #     tornado.websocket.WebSocketHandler.__init__(self, *args, **kwargs)
    #     self.users = []
    users = [] # reserve all web clients handlers.

    def open(self):
        self.users.append(self)
        data_receiver.b_have_client = True
        self.callback = tornado.ioloop.PeriodicCallback(self.send_data, callback_rate)
        self.callback.start()

    def send_data(self):
        data_lock.acquire()
        if data_receiver.latest_packets is not None:
            for key in data_receiver.latest_packets:
                json_msg = json.dumps({ 'messageType' : 'event',  'data' : {'packetType' : key, 'packet' : data_receiver.latest_packets[key] }})
                self.write_message(json_msg)
                # if key == 'GH':
                #     print(json_msg)
                #     print('************')
            data_receiver.latest_packets.clear()

            for key in data_receiver.all_GSVM_packets:
                json_msg = json.dumps({ 'messageType' : 'event',  'data' : {'packetType' : 'GSVM', 'packet' : key }})
                self.write_message(json_msg)
                # print(json_msg)
                # print('************')
            data_receiver.all_GSVM_packets = []
            # print(json.dumps({ 'messageType' : 'event',  'data' : { 'data' : data_receiver.data }}))
        data_lock.release()

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

        return False

    def check_origin(self, origin):
        return True

class DataReceiver(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
        '''Init
        '''
        self.latest_packets = {}
        self.all_GSVM_packets = []
        self.msgs_send2web = []
        self.b_have_client = False

        self.rover_properties = utility.load_configuration(os.path.join('setting', 'rover.json'))
        if not self.rover_properties:
            os._exit(1)

        self.output_packets = self.rover_properties['userMessages']['outputPackets']
        for packet in self.output_packets:
            if 1 == packet['send2web']:
                self.msgs_send2web.append(packet['name'])
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
        '''
            1. Message which need to send to Web client every x seconds by timer.
                1.1 SA: Sensor Activity.
                1.2 NCA: NTRIP Client Activity.
            2. Message which need to send all to Web client.
                2.1 SSS: Satellite Signal Strength
                2.2 GSVM: Repackaged GSV Message
            3. Message which need to be re-packet then send to Web client.
                3.1 Construct new 'NAV' packet based on KFN/CNM/GH then send to web client.
            4. Message which need to send to Web client at once when driver receives.
                4.1 PID: Product ID Message.
                4.2 EV: Engine Version Message.
            5. Message which needn't to send to Web client.
                5.1 TSM: PPS info.
        '''
        if self.b_have_client and packet_type in self.msgs_send2web:
            if packet_type == 'GSVM':
                self.all_GSVM_packets.append(data)
            else:
                self.latest_packets[packet_type] = data

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


if __name__ == '__main__':
    '''main'''
    callback_rate = 150
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
