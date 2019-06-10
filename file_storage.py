# -*- coding: utf-8 -*
import sys
import os
import time
import datetime
import json
import rover_application_base
import driver_ins1000
import requests
import threading
from azure.storage.blob import AppendBlobService
from azure.storage.blob import ContentSettings
from azure.storage.blob import BlockBlobService
import utility


class RoverLogApp(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
        '''Initialize
        '''
        self.file_loger = FileLoger()
        self.file_loger.start_user_log()
        self.ii = 0

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
        # if packet_type == 'NAV':
        #     self.ii = self.ii + 1
        #     if self.ii % 1000 == 0:
        #         print('[{0}]:{1}'.format(datetime.datetime.now().strftime('%S'), self.ii))

    def on_exit(self):
        self.file_loger.stop_user_log()
        pass


class FileLoger():
    def __init__(self):
        '''Initialize and create a CSV file
        '''
        start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.rover_properties = utility.load_configuration(os.path.join('setting', 'rover.json'))
        if not self.rover_properties:
            os._exit(1)
        if not os.path.exists('data/'):
            os.mkdir('data/')
        self.output_packets = self.rover_properties['userMessages']['outputPackets']
        self.log_file_rows = {}
        self.log_file_names = {}
        self.log_files_obj = {}
        self.log_files = {}
        self.user_file_name = '' # the prefix of log file name.
        self.msgs_need_to_log = []
        self.ws = False
        # azure app.
        self.user_id = ''
        self.file_name = ''
        self.sas_token = '' 
        self.db_user_access_token = ''
        self.host_url = self.rover_properties['userConfiguration']['hostURL']

        #
        self.threads = []  # thread of receiver and paser
        self.exit_thread = False  # flag of exit threads
        self.exit_lock = threading.Lock()  # lock of exit_thread
        self.data_dict = {}  # data container
        self.data_lock = threading.Lock()  # lock of data_queue

    def start_user_log(self, file_name='', ws=False):
        '''
        start log.
        return:
                0: OK
                1: exception that has started logging already.
                2: other exception.
        '''
        try:
            if len(self.log_file_rows) > 0:
                return 1 # has started logging already.

            self.ws = ws
            self.exit_thread = False
            self.user_file_name = file_name
            start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            for packet in self.output_packets:
                if 1 == packet['save2file']:
                    self.msgs_need_to_log.append(packet['name'])
                    self.log_file_rows[packet['name']] = 0
                    if self.user_file_name == '':
                        self.log_file_names[packet['name']] = packet['name'] +'-' + start_time + '.csv'
                    else:
                        self.log_file_names[packet['name']] = self.user_file_name + '_' + packet['name'] +'-' + start_time + '.csv'
                    self.log_files[packet['name']] = self.log_file_names[packet['name']]
                    self.log_files_obj[packet['name']] = open('data/' + self.log_file_names[packet['name']], 'w')    

            if self.ws:
                self.get_sas_token()
                self.data_dict.clear()
                for i, (k, v) in enumerate(self.log_files.items()): # k:pack type  v:log file name
                    self.data_dict[v]=''
                    threading.Thread(target=self.upload_azure, args=(k,v)).start()
            return 0
        except Exception as e:
            print('Exception! File:[{0}], Line:[{1}]. Exception:{2}'.format(__file__, sys._getframe().f_lineno, e))
            return 2

    def stop_user_log(self):
        '''
        stop log.
        return:
                0: OK
                1: exception that driver hasn't started logging files yet.
                2: other exception.
        '''
        rev = 0
        try:
            if len(self.log_file_rows) == 0:
                return 1 # driver hasn't started logging files yet.
            for i, (k, v) in enumerate(self.log_files_obj.items()):
                v.close()
            self.log_file_rows.clear()
            self.log_file_names.clear()
            self.log_files_obj.clear()
            rev = 0
        except Exception as e:
            print(e)
            rev = 2

        if self.ws:
            time.sleep(1)
            self.exit_lock.acquire()
            self.exit_thread = True
            self.exit_lock.release()
            self.ws = False
            
        return rev

    # def upload_callback(self, current, total):
    #     print('({}, {})'.format(current, total))

    def upload_azure(self, packet_type, log_file_name):
        if self.db_user_access_token == '' or self.sas_token == '':
            print("Error: Can not upload log to azure since token is empty! Please check the network.")
            
        print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:'), log_file_name, ' start.')

        accountName = 'navview'
        countainerName = 'data-1000'
        fileName = log_file_name
        bcreate_blob_ok = False

        error_connection = 'ConnectionError'
        error_authorization = 'AuthenticationFailed'
        ii=0
        while True:
            # get data from data_dict.
            self.data_lock.acquire()
            text = self.data_dict[log_file_name]
            self.data_dict[log_file_name] = ''
            self.data_lock.release()

            # check if user stop logging data.
            self.exit_lock.acquire()
            if self.exit_thread:
                # check for internet and text
                if text == '' or (not self.internet_on()):
                    self.exit_lock.release()
                    break
                else:
                    pass
            self.exit_lock.release()

            #let CPU have a break.
            if text == '' : 
                time.sleep(1)
                continue

            #create blob on azure
            if not bcreate_blob_ok:
                try:
                    self.append_blob_service = AppendBlobService(account_name=accountName,
                                                                sas_token=self.sas_token,
                                                                protocol='http')
                    self.append_blob_service.create_blob(container_name=countainerName, blob_name=fileName,
                                                        content_settings=ContentSettings(content_type='text/plain'))
                    bcreate_blob_ok = True
                    threading.Thread(target=self.save_to_db_task, args=(packet_type, log_file_name)).start()
                except Exception as e:
                    # print('Exception when create_blob:', type(e), e)
                    if error_connection in str(e):
                        pass
                    elif error_authorization in str(e):
                        self.get_sas_token()
                        self.append_blob_service = AppendBlobService(account_name=accountName,
                                                                    sas_token=self.sas_token,
                                                                    protocol='http')
                    print('Retry to create_blob again...')
                    continue

            # append blob on azure
            try:
                # self.append_blob_service.append_blob_from_text(countainerName, fileName, text, progress_callback=self.upload_callback)
                self.append_blob_service.append_blob_from_text(countainerName, fileName, text)
            except Exception as e:
                # print('Exception when append_blob:', type(e), e)
                if error_connection in str(e):
                    pass
                elif error_authorization in str(e):
                    self.get_sas_token()
                    self.append_blob_service = AppendBlobService(account_name=accountName,
                                                                sas_token=self.sas_token,
                                                                protocol='http')
                    # if append blob failed, do not drop 'text', but push 'text' to data_dict and re-append next time.
                    self.data_lock.acquire()
                    self.data_dict[log_file_name] = text + self.data_dict[log_file_name]
                    self.data_lock.release()

        if bcreate_blob_ok:
            # if not self.save_to_ans_platform(packet_type, log_file_name):
            #     print('save_to_ans_platform failed.')
            print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:') , log_file_name, ' done.')

    def save_to_db_task(self, packet_type, file_name):
        if not self.save_to_ans_platform(packet_type, file_name):
            print('save_to_ans_platform failed.')

    def update(self, packet, packet_type, is_var_len_frame):
        if len(self.log_file_rows) == 0: #if hasn't started logging.
            return

        if packet_type in self.msgs_need_to_log:
            if is_var_len_frame:
                self.log_var_len(packet, packet_type)
            else:
                self.log(packet, packet_type)

    def get_log_file_names(self):
        return self.log_file_names.copy()

    def log(self, data, packet_type):
        ''' Parse the data, read in from the unit, and generate a data file using
            the json properties file to create a header and specify the precision
            of the data in the resulting data file.
        '''
        output_packet = next((x for x in self.output_packets if x['name'] == packet_type), None)

        '''Write row of CSV file based on data received.  Uses dictionary keys for column titles
        '''
        if self.log_file_rows[packet_type] == 0:
            # Loop through each item in the data dictionary and create a header from the json
            #   properties that correspond to the items in the dictionary
            labels = ''
            # for key in data:
            for i, (k, v) in enumerate(data.items()):
                '''dataStr = output_packet['payload'][i]['name'] + \
                          ' [' + \
                          output_packet['payload'][i]['unit'] + \
                          ']'''
                dataStr = output_packet['payload'][i]['name']
                unitStr = output_packet['payload'][i]['unit']
                if unitStr == '':
                    labels = labels + '{0:s},'.format(dataStr)
                else:
                    labels = labels + '{0:s} ({1:s}),'.format(dataStr, unitStr)

            # Remove the comma at the end of the string and append a new-line character
            labels = labels[:-1]
            header = labels + '\n'
        else:
            header = ''

        self.log_file_rows[packet_type] += 1

        # Loop through the items in the data dictionary and append to an output string
        #   (with precision based on the data type defined in the json properties file)
        str = ''
        for i, (k, v) in enumerate(data.items()):
            outputPcktType = output_packet['payload'][i]['type']

            if outputPcktType == 'uint32' or outputPcktType == 'int32' or \
               outputPcktType == 'uint16' or outputPcktType == 'int16' or \
               outputPcktType == 'uint64' or outputPcktType == 'int64':
                # integers and unsigned integers
                str += '{0:d},'.format(v)
            elif outputPcktType == 'double':
                # double
                str += '{0:0.8f},'.format(v)# 15.12
            elif outputPcktType == 'float':
                str += '{0:0.4f},'.format(v) # 12.8
            elif outputPcktType == 'uint8':
                # byte
                str += '{0:d},'.format(v)
            elif outputPcktType == 'uchar' or outputPcktType == 'char' or outputPcktType == 'string':
                # character
                str += '{:},'.format(v)
            else:
                # unknown
                str += '{0:3.5f},'.format(v)
        # 
        str = header + str[:-1] + '\n'

        self.log_files_obj[packet_type].write(str)
        self.log_files_obj[packet_type].flush()

        if self.ws:
            self.data_lock.acquire()
            self.data_dict[self.log_files[packet_type]] = self.data_dict[self.log_files[packet_type]] + str
            self.data_lock.release()

    def log_var_len(self, data, packet_type):
        ''' Parse the data, read in from the unit, and generate a data file using
            the json properties file to create a header and specify the precision
            of the data in the resulting data file.
        '''
        output_packet = next((x for x in self.output_packets if x['name'] == packet_type), None)

        '''Write row of CSV file based on data received.  Uses dictionary keys for column titles
        '''
        if self.log_file_rows[packet_type] == 0:

            # Loop through each item in the data dictionary and create a header from the json
            #   properties that correspond to the items in the dictionary
            labels = ''
            for value in output_packet['payload']:
                dataStr = value['name']
                unitStr = value['unit']
                if unitStr == '':
                    labels = labels + '{0:s},'.format(dataStr)
                else:
                    labels = labels + '{0:s} ({1:s}),'.format(dataStr, unitStr)
            # Remove the comma at the end of the string and append a new-line character
            labels = labels[:-1]
            header = labels + '\n'
        else:
            header = ''

        self.log_file_rows[packet_type] += 1

        # Loop through the items in the data dictionary and append to an output string
        #   (with precision based on the data type defined in the json properties file)
        str = ''
        const_str = ''
        var_str = ''
        var_fileld_tpyes = []
        var_fileld_num = len(output_packet['payload']) - output_packet['var_num']['field_idx']
        const_fileld_num = len(output_packet['payload']) - var_fileld_num

        for idx, value in enumerate(output_packet['payload']):
            if idx >= const_fileld_num:
                var_fileld_tpyes.append(value['type'])

        for idx, key in enumerate(data):
            if idx == 0: # handle const filelds which are all in the first item of data.
                for i, (k, v) in enumerate(key.items()):
                    outputPcktType = output_packet['payload'][i]['type']

                    if outputPcktType == 'uint32' or outputPcktType == 'int32' or \
                    outputPcktType == 'uint16' or outputPcktType == 'int16' or \
                    outputPcktType == 'uint64' or outputPcktType == 'int64':
                        # integers and unsigned integers
                        const_str += '{0:d},'.format(v)
                    elif outputPcktType == 'double':
                        # double
                        const_str += '{0:0.12f},'.format(v) # 15.12
                    elif outputPcktType == 'float':
                        const_str += '{0:0.4f},'.format(v) # 12.8
                    elif outputPcktType == 'uint8':
                        # byte
                        const_str += '{0:d},'.format(v)
                    elif outputPcktType == 'uchar' or outputPcktType == 'char' or outputPcktType == 'string':
                        # character
                        const_str += '{:},'.format(v)
                    else:
                        # unknown
                        const_str += '{0:3.5f},'.format(v)
            else:
                for i, (k, v) in enumerate(key.items()):
                    outputPcktType = var_fileld_tpyes[i]
                    if outputPcktType == 'uint32' or outputPcktType == 'int32' or \
                    outputPcktType == 'uint16' or outputPcktType == 'int16' or \
                    outputPcktType == 'uint64' or outputPcktType == 'int64':
                        # integers and unsigned integers
                        var_str += '{0:d},'.format(v)
                    elif outputPcktType == 'double':
                        # double
                        var_str += '{0:15.12f},'.format(v)# 15.12
                    elif outputPcktType == 'float':
                        var_str += '{0:12.4f},'.format(v) # 12.8
                    elif outputPcktType == 'uint8':
                        # byte
                        var_str += '{0:d},'.format(v)
                    elif outputPcktType == 'uchar' or outputPcktType == 'char':
                        # character
                        var_str += '{:},'.format(v)
                    else:
                        # unknown
                        var_str += '{0:3.5f},'.format(v)

                str = const_str + var_str
                str = header + str[:-1] + '\n'

                self.log_files_obj[packet_type].write(str)
                self.log_files_obj[packet_type].flush()

                if self.ws:
                    self.data_lock.acquire()
                    self.data_dict[self.log_files[packet_type]] = self.data_dict[self.log_files[packet_type]] + str
                    self.data_lock.release()

                header = ''
                str = ''
                var_str = ''

    def set_user_id(self, user_id):
        self.user_id = user_id
        if not isinstance(self.user_id, str):
            self.user_id = str(self.user_id)

    def set_user_access_token(self, access_token):
        self.db_user_access_token = access_token

    def upload(self, log_file_names):
        t = threading.Thread(target=self.upload_to_azure_task, args=(log_file_names, ))
        t.start()

    def upload_to_azure_task(self, log_files_dict):
        self.get_sas_token()
        if self.db_user_access_token != '' and self.sas_token != '':
            print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:') , 'Start.')
            for i, (k, v) in enumerate(log_files_dict.items()): # k: packet type; v: log file name
                print('upload:', v)
                self.upload_to_azure(k, v)
            # self.db_user_access_token = ''
            # self.sas_token = ''    
            print(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S:') , 'Done.')

    def get_sas_token(self):
        try:
            url = self.host_url + "token/storagesas"
            headers = {'Content-type': 'application/json', 'Authorization': self.db_user_access_token}
            response = requests.post(url, headers=headers)
            rev = response.json()
            if 'token' in rev:
                self.sas_token = rev['token']
            else:
                self.sas_token = ''
                print('Error: Get sas token failed!')
        except Exception as e:
            print('Exception when get_sas_token:', e)

    def upload_to_azure(self, packet_type, file_name):
        ''' Upload CSV's to Azure container.
        '''
        f = open("data/" + file_name, "r")
        text = f.read() #.decode("utf-8")

        try:
            self.azure_storage('navview', self.sas_token, 'data', file_name, text)
        except Exception as e:
            print('azure_storage exception:', e)
            return
            # Try again!
            # self.azure_storage('navview', self.sas_token, 'data', file_name, text)
            pass

        ''' Trigger Database upload
        '''
        rev = self.save_to_ans_platform(packet_type, file_name)
        if not rev:
            print('save_to_ans_platform failed.')

    def azure_storage(self, accountName, sasToken, countainerName,fileName,text):
        if 0:
            self.append_blob_service = AppendBlobService(account_name=accountName,
                                                        sas_token=sasToken,
                                                        protocol='http')
            self.append_blob_service.create_blob(container_name=countainerName, blob_name=fileName,
                                                content_settings=ContentSettings(content_type='text/plain'))
            self.append_blob_service.append_blob_from_text(countainerName, fileName, text)
        else:
            self.block_blob_service = BlockBlobService(account_name=accountName,
                                                        sas_token=sasToken,
                                                        protocol='http')
            self.block_blob_service.create_blob_from_text(  container_name= countainerName,
                                                        blob_name= fileName,
                                                        text=text,
                                                        content_settings=ContentSettings(content_type='text/plain'))

    def save_to_ans_platform(self, packet_type, file_name):
        ''' Upload CSV related information to the database.
        '''
        try:
            data = {"type": 'INS', "model": 'INS1000', "fileName": file_name, "url": file_name, "userId": self.user_id, 
                    "logInfo": { "pn": '11', "sn": '', "packetType":packet_type,"insProperties":json.dumps(self.rover_properties)}}

            url = self.host_url + "api/recordLogs/post"
            data_json = json.dumps(data)
            headers = {'Content-type': 'application/json', 'Authorization': self.db_user_access_token}
            response = requests.post(url, data=data_json, headers=headers)
            return True if 'success' in response.json() else False
        except Exception as e:
            print('Exception when update db:', e)

    def internet_on(self):
        try:
            url = 'https://navview.blob.core.windows.net/'
            if sys.version_info[0] > 2:
                import urllib.request
                response = urllib.request.urlopen(url, timeout=1)
            else:
                import urllib2
                response = urllib2.urlopen(url, timeout=1)
            # print(response.read())
            return True
        except urllib2.URLError as err: 
            return False


def main():
    '''main'''
    driver = driver_ins1000.RoverDriver()
    rover_log = RoverLogApp()
    driver.set_app(rover_log)
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
    main()
    # os._exit(1)
    