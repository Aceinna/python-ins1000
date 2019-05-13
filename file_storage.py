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
        rover_properties = utility.load_configuration(os.path.join('setting', 'rover.json'))
        if not rover_properties:
            os._exit(1)
        if not os.path.exists('data/'):
            os.mkdir('data/')
        self.output_packets = rover_properties['userMessages']['outputPackets']
        self.log_file_rows = {}
        self.log_file_names = {}
        self.log_files = {}
        self.user_file_name = '' # the prefix of log file name.
        self.msgs_need_to_log = []

    def start_user_log(self, file_name=''):
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
                    self.log_files[packet['name']] = open('data/' + self.log_file_names[packet['name']], 'w')        
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
        try:
            if len(self.log_file_rows) == 0:
                return 1 # driver hasn't started logging files yet.
            for i, (k, v) in enumerate(self.log_files.items()):
                v.close()
            self.log_file_rows.clear()
            self.log_file_names.clear()
            self.log_files.clear()
            return 0
        except Exception as e:
            print(e)
            return 2

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
                str += '{0:15.8f},'.format(v)# 15.12
            elif outputPcktType == 'float':
                str += '{0:12.4f},'.format(v) # 12.8
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
        str = str[:-1]
        str = str + '\n'

        if self.log_file_rows[packet_type] == 1:
            self.log_files[packet_type].write(header+str)
        else:
            self.log_files[packet_type].write(str)
        self.log_files[packet_type].flush()

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
                        const_str += '{0:15.12f},'.format(v) # 15.12
                    elif outputPcktType == 'float':
                        const_str += '{0:12.4f},'.format(v) # 12.8
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
                str = str[:-1]
                str = str + '\n'
                    
                if self.log_file_rows[packet_type] == 1:
                    self.log_files[packet_type].write(header+str)
                else:
                    self.log_files[packet_type].write(str)
                self.log_files[packet_type].flush()

                header = ''
                str = ''
                var_str = ''
                

class FileUploader():
    def __init__(self):
        # azure app.
        self.user_id = ''
        self.file_name = ''
        self.sas_token = '' 
        self.db_user_access_token = ''
        self.rover_properties = utility.load_configuration(os.path.join('setting', 'rover.json'))
        if not self.rover_properties:
            os._exit(1)
        self.host_url = self.rover_properties['userConfiguration']['hostURL']

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
