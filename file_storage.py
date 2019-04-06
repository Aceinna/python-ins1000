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
        if user and list(user.keys())[0] == 'startLog':
            self.username = user['startLog']['username']
            self.userId = user['startLog']['id']
            self.userFilename = user['startLog']['fileName']
            self.userAccessToken = user['startLog']['access_token']

        self.start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.rover_properties = utility.load_configuration(os.path.join('setting', 'rover.json'))
        if not self.rover_properties:
            os._exit(1)
        if not os.path.exists('data/'):
            os.mkdir('data/')
        self.output_packets = self.rover_properties['userMessages']['outputPackets']
        self.first_row = {}
        self.log_file_names = {}
        self.log_files = {}
        self.msgs_need_to_log = []

        if user:
            with open('tempLogFiles.json', 'r') as outfile:
                data = json.load(outfile)
                self.log_file_names = data

        if not user:
            with open('tempLogFiles.json', 'w') as outfile:
                json.dump({}, outfile)
            try:
                for packet in self.output_packets:
                    if 1 == packet['save2file']:
                        self.msgs_need_to_log.append(packet['name'])
                    else:
                        continue
                    self.first_row[packet['name']] = 0
                    self.log_file_names[packet['name']] = packet['name'] +'-' + self.start_time + '.csv'
                    self.log_files[packet['name']] = open('data/' + self.log_file_names[packet['name']], 'w')

                    entry = {packet['name']:self.log_file_names[packet['name']]}
                    with open('tempLogFiles.json') as f:
                        data = json.load(f)
                    data.update(entry)
                    with open('tempLogFiles.json','w') as f:
                        json.dump(data, f)
            except:
                pass

        if user and list(user.keys())[0] == 'startLog':
            self.savetoAnsPlatform()

        if user and list(user.keys())[0] == 'stopLog':
            time.sleep(10)
            self.close()
                # os.remove("tempLogFiles.json")

    def on_reinit(self):
        print ("RoverLogApp.on_reinit()")
        pass
        # Is it necessary to create a new file to log when replug serial connector?

        # self.start_time = datetime.datetime.now().strftime('%Y%m%d_%H_%M_%S')
        # try:
        #     for packet in self.output_packets:
        #         self.first_row[packet['name']] = 0
        #         self.log_file_names[packet['name']] = packet['name'] +'-' + self.start_time + '.csv'
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

    def log(self, data, packet_type):
        ''' Parse the data, read in from the unit, and generate a data file using
            the json properties file to create a header and specify the precision
            of the data in the resulting data file.
        '''
        if not self.rover_properties:
            return

        output_packet = next((x for x in self.rover_properties['userMessages']['outputPackets'] if x['name'] == packet_type), None)

        '''Write row of CSV file based on data received.  Uses dictionary keys for column titles
        '''
        if not self.first_row[packet_type]:
            self.first_row[packet_type] = 1

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
                    labels = labels + '{0:s}({1:s}),'.format(dataStr, unitStr)

            # Remove the comma at the end of the string and append a new-line character
            labels = labels[:-1]
            header = labels + '\n'
        else:
            self.first_row[packet_type] += 1
            header = ''


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
                str += '{0:15.12f},'.format(v)
            elif outputPcktType == 'float':
                str += '{0:12.8f},'.format(v)
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
        self.log_files[packet_type].write(header+str)
        self.log_files[packet_type].flush()

    def log_var_len(self, data, packet_type):
        ''' Parse the data, read in from the unit, and generate a data file using
            the json properties file to create a header and specify the precision
            of the data in the resulting data file.
        '''
        if not self.rover_properties:
            return

        output_packet = next((x for x in self.rover_properties['userMessages']['outputPackets'] if x['name'] == packet_type), None)

        '''Write row of CSV file based on data received.  Uses dictionary keys for column titles
        '''
        if not self.first_row[packet_type]:
            self.first_row[packet_type] = 1

            # Loop through each item in the data dictionary and create a header from the json
            #   properties that correspond to the items in the dictionary
            labels = ''
            for value in output_packet['payload']:
                dataStr = value['name']
                unitStr = value['unit']
                if unitStr == '':
                    labels = labels + '{0:s},'.format(dataStr)
                else:
                    labels = labels + '{0:s}({1:s}),'.format(dataStr, unitStr)
            # Remove the comma at the end of the string and append a new-line character
            labels = labels[:-1]
            header = labels + '\n'
        else:
            self.first_row[packet_type] += 1
            header = ''

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
                        const_str += '{0:15.12f},'.format(v)
                    elif outputPcktType == 'float':
                        const_str += '{0:12.8f},'.format(v)
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
                        var_str += '{0:15.12f},'.format(v)
                    elif outputPcktType == 'float':
                        # print(3) #key + str(2))
                        var_str += '{0:12.8f},'.format(v)
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
                self.log_files[packet_type].write(header+str)
                header = ''
                str = ''
                var_str = ''
                self.log_files[packet_type].flush()

    ''' Upload CSV's to Azure container.
    '''
    def uploadtoAzure(self,fileDisplayName):

        # f = open("data/" + self.user['fileName'], "r")
        f = open("data/" + fileDisplayName, "r")
        text = f.read()
        account_key = '+roYuNmQbtLvq2Tn227ELmb6s1hzavh0qVQwhLORkUpM0DN7gxFc4j+DF/rEla1EsTN2goHEA1J92moOM/lfxg=='

        try:
            self.azureStorage('navview',account_key,'data', fileDisplayName, text)
        except:
            # Try again!
            self.azureStorage('navview', account_key, 'data', fileDisplayName, text)

        ''' Trigger Database upload
        '''
        # self.savetoAnsPlatform()

    def azureStorage(self, accountName, accountkey, countainerName,fileName,text):
        self.append_blob_service = AppendBlobService(account_name=accountName,
                                                     account_key=accountkey,
                                                     protocol='http')
        self.append_blob_service.create_blob(container_name=countainerName, blob_name=fileName,
                                             content_settings=ContentSettings(content_type='text/plain'))
        self.append_blob_service.append_blob_from_text(countainerName, fileName, text)


    ''' Upload CSV related information to the database.
    '''
    def savetoAnsPlatform(self):
        for files in self.log_file_names:
            fileDisplayName = files + "-" + self.userFilename + ".csv"

            data = {"pn": '1.0.0', "sn": 'rtk', "fileName": fileDisplayName, "url": self.log_file_names[files],
                    "imuProperties": json.dumps(self.rover_properties),
                    "sampleRate": '100', "packetType": files, "userId": self.userId}

            url = "https://api.aceinna.com/api/datafiles/replaceOrCreate"
            data_json = json.dumps(data)
            headers = {'Content-type': 'application/json', 'Authorization': self.userAccessToken}
            response = requests.post(url, data=data_json, headers=headers)
            response = response.json()



    # def close(self,fileName,storedFile):
    #     time.sleep(0.1)
    #     # if self.ws:
    #     storedFile.close()
    #     threading.Thread(target=self.uploadtoAzure(fileName)).start()
    #     # else:
    #     #     self.file.close()
    #     # print('close')
    #     # try:
    #     #     for packet in self.output_packets:
    #     #         self.log_files[packet['name']].close()
    #     #         threading.Thread(target=self.write_to_azurelog_files[packet['name']]).start()
    #     # except:
    #     #     pass

    def close(self):
        for files in self.log_file_names:
            self.uploadtoAzure(self.log_file_names[files])


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
