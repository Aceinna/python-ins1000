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
import utility


class RoverLogApp(rover_application_base.RoverApplicationBase):
    def __init__(self, user=False):
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
        self.log_files = {}
        self.msgs_need_to_log = []
        self.user_log_file_rows = {}
        self.user_log_file_names = {}
        self.user_log_files = {}
        # azure app.
        self.user_id = ''
        self.file_name = ''
        self.blob_user_access_token = '' # Reserved.
        self.db_user_access_token = ''
        self.host_address = self.rover_properties['userConfiguration']['hostAddress']

        for packet in self.output_packets:
            if 1 == packet['save2file']:
                self.msgs_need_to_log.append(packet['name'])
            else:
                continue
            self.log_file_rows[packet['name']] = 0
            self.log_file_names[packet['name']] = packet['name'] +'-' + start_time + '.csv'
            self.log_files[packet['name']] = open('data/' + self.log_file_names[packet['name']], 'w')

    def on_reinit(self):
        print ("RoverLogApp.on_reinit()")
        pass
        # Is it necessary to create a new file to log when replug serial connector?

        # start_time = datetime.datetime.now().strftime('%Y%m%d_%H_%M_%S')
        # try:
        #     for packet in self.output_packets:
        #         self.log_file_rows[packet['name']] = 0
        #         self.log_file_names[packet['name']] = packet['name'] +'-' + start_time + '.csv'
        #         self.log_files[packet['name']] = open('data/' + self.log_file_names[packet['name']], 'w')# just log Compact Navigation Message
        # except:
        #     pass

    def on_find_active_rover(self):
        print ("RoverLogApp.on_find_active_rover()")

    def on_message(self, *args):
        packet_type = args[0]
        self.data = args[1]
        is_var_len_frame = args[2]
        if packet_type in self.msgs_need_to_log:
            if is_var_len_frame:
                self.log_var_len(self.data, packet_type)
            else:
                self.log(self.data, packet_type)

    def on_exit(self):
        pass

    def start_user_log(self, user_id, file_name):
        self.user_id = user_id
        self.file_name = file_name
        try:
            if len(self.user_log_file_rows) > 0:
                return 1

            if not isinstance(self.user_id, str):
                self.user_id = str(self.user_id)
            start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            for packet in self.output_packets:
                if packet['name'] in self.msgs_need_to_log:
                    self.user_log_file_rows[packet['name']] = 0
                    self.user_log_file_names[packet['name']] = self.file_name + '_' + packet['name'] +'-' + start_time + '.csv'
                    self.user_log_files[packet['name']] = open('data/' + self.user_log_file_names[packet['name']], 'w')
            return 0
        except Exception as e:
            print(e)
            return 2

    def stop_user_log(self, db_user_access_token):
        try:
            if len(self.user_log_file_rows) == 0:
                return 1 # driver hasn't started logging files yet.

            for i, (k, v) in enumerate(self.user_log_files.items()):
                v.close()

            # start a thread to upload logs to cloud here if necessary.
            self.db_user_access_token = db_user_access_token
            t = threading.Thread(target=self.upload_to_azure_task, args=(self.user_log_file_names.copy(), ))
            t.start()
            print("upload_to_azure_task[{0}({1})] started.".format(t.name, t.ident)) # for debug 

            self.user_log_file_rows.clear()
            self.user_log_file_names.clear()
            self.user_log_files.clear()
            return 0
        except Exception as e:
            print(e)
            return 2

    def upload_to_azure_task(self, log_files_dict):
        for i, (k, v) in enumerate(log_files_dict.items()): # k: packet type; v: log file name
            self.uploadtoAzure(k, v)

    def log(self, data, packet_type):
        ''' Parse the data, read in from the unit, and generate a data file using
            the json properties file to create a header and specify the precision
            of the data in the resulting data file.
        '''
        output_packet = next((x for x in self.output_packets if x['name'] == packet_type), None)

        '''Write row of CSV file based on data received.  Uses dictionary keys for column titles
        '''
        if self.log_file_rows[packet_type] == 0 or (len(self.user_log_file_rows) > 0 and self.user_log_file_rows[packet_type] == 0):
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
        if len(self.user_log_file_rows) > 0:
            self.user_log_file_rows[packet_type] += 1

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

        if len(self.user_log_file_rows) > 0:
            if self.user_log_file_rows[packet_type] == 1:
                self.user_log_files[packet_type].write(header+str)
            else:
                self.user_log_files[packet_type].write(str)
            self.user_log_files[packet_type].flush()

    def log_var_len(self, data, packet_type):
        ''' Parse the data, read in from the unit, and generate a data file using
            the json properties file to create a header and specify the precision
            of the data in the resulting data file.
        '''
        output_packet = next((x for x in self.output_packets if x['name'] == packet_type), None)

        '''Write row of CSV file based on data received.  Uses dictionary keys for column titles
        '''
        if self.log_file_rows[packet_type] == 0 or (len(self.user_log_file_rows) > 0 and self.user_log_file_rows[packet_type] == 0):

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
        if len(self.user_log_file_rows) > 0:
            self.user_log_file_rows[packet_type] += 1

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

                if len(self.user_log_file_rows) > 0:
                    if self.user_log_file_rows[packet_type] == 1:
                        self.user_log_files[packet_type].write(header+str)
                    else:
                        self.user_log_files[packet_type].write(str)
                    self.user_log_files[packet_type].flush()

                header = ''
                str = ''
                var_str = ''


    ''' Upload CSV's to Azure container.
    '''
    def uploadtoAzure(self, packet_type, file_name):
        f = open("data/" + file_name, "r")
        text = f.read()
        account_key = '+roYuNmQbtLvq2Tn227ELmb6s1hzavh0qVQwhLORkUpM0DN7gxFc4j+DF/rEla1EsTN2goHEA1J92moOM/lfxg=='

        try:
            # self.azureStorage('navview',account_key,'data', file_name, text)
            self.azureStorage('navview', account_key, 'data-1000', file_name, text)
        except:
            # Try again!
            # self.azureStorage('navview', account_key, 'data', file_name, text)
            self.azureStorage('navview', account_key, 'data-1000', file_name, text)

        ''' Trigger Database upload
        '''
        self.savetoAnsPlatform(packet_type, file_name)

    def azureStorage(self, accountName, accountkey, countainerName,fileName,text):
        self.append_blob_service = AppendBlobService(account_name=accountName,
                                                     account_key=accountkey,
                                                     protocol='http')
        self.append_blob_service.create_blob(container_name=countainerName, blob_name=fileName,
                                             content_settings=ContentSettings(content_type='text/plain'))
        self.append_blob_service.append_blob_from_text(countainerName, fileName, text)


    ''' Upload CSV related information to the database.
    '''
    def savetoAnsPlatform(self, packet_type, file_name):
        if self.db_user_access_token == '':
            self.db_user_access_token = 'dNKBMAJitNN1oQEFezCxXKJevj5Vgo8EQhoUJY9kB2xxZkkVHrefBabI7S5BAnJj' # for debug.

        data = {"type": 'INS', "model": 'INS1000', "fileName": file_name, "url": file_name, "userId": self.user_id, 
                "logInfo": { "pn": '11', "sn": '', "packetType":packet_type,"insProperties":json.dumps(self.rover_properties)}}

        url = "http://" + self.host_address + ":3000/api/recordLogs/post"
        data_json = json.dumps(data)
        headers = {'Content-type': 'application/json', 'Authorization': self.db_user_access_token}
        response = requests.post(url, data=data_json, headers=headers)
        response = response.json()

    def close(self):
        pass

    def test_azure(self):
        threading.Thread(target=self.upload_to_azure_task(self.user_log_file_names.copy())).start()


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
    