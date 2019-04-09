# -*- coding: utf-8 -*
import struct


class Msg_060C0A(object):
    def __init__(self, frame):
        ''' initialization
        '''
        self.frame = frame
        self.payload_len = 0
        self.topic = 0
        self.indicators = 0
        self.flags = 0
        self.ICD_num = 0
        self.min_vel = 0
        self.max_unaided_time = 0
        self.max_output_rate = 0
        self.imu_rotation_matrix = []
        self.output_position_offset = []
        self.smooth_mode = 0
        self.GNSS_lever_arm = []
        self.GNSS_lever_arm_uncertainty = []
        self.ICD_configuration = []
        # DMI configurations
        self.DMI_id = 0
        self.DMI_scale_factor = 0
        self.DMI_lever_arm= []
        self.DMI_lever_arm_uncertainty = 0
        self.DMI_options = 0
        # get from self.indicators
        self.extended_version_flag = 0
        self.antenna_num = 0
        self.DMI_exists = 0
        # used to packet the '8.4 User Configuration Setup' message.
        self.msg_060D_frame = ''
        
        self.unpack_msg_060C0A()
        pass

    def unpack_msg_060C0A(self):
        '''
        parse AF20060C0A message which topic is 0A.
        '''
        PAYLOAD_SUB_ID_IDX = 3
        PAYLOAD_LEN_IDX = 4
        PAYLOAD_TOPIC_IDX = 6
        payload_len = 256 * self.frame[PAYLOAD_LEN_IDX + 1] + self.frame[PAYLOAD_LEN_IDX]
        payload = self.frame[6:payload_len+6]   # extract the payload

        i = 0
        self.topic = payload[i]
        i+=1

        self.indicators = payload[i]
        # antenna_num is the 1st and 2nd bits in 'Aiding sensor indicators' filed.
        self.antenna_num = self.indicators & 3
        # extended_version_flag is the 3rd bit in 'Aiding sensor indicators' filed.
        self.extended_version_flag = (self.indicators & 8 > 0) 
        # DMI_exists flag is the 4 bits in 'Aiding sensor indicators' filed.
        self.DMI_exists = self.indicators & 4
        i+=1

        self.flags = payload[i]
        # if flags_leverarm_from_imu_center is False, means leverarm is w.r.t housing mark        
        self.flags_leverarm_from_imu_center = (self.flags & 4 == 0)   
        i+=1

        self.ICD_num = payload[i]
        i+=1

        self.min_vel = payload[i]
        i+=1

        self.max_unaided_time = payload[i+1]*256+payload[i]
        i+=2

        scaling = 1.0e-1
        self.max_output_rate = int((payload[i+1]*256+payload[i])*scaling)
        i+=2

        tmp = struct.pack('72B', *payload[i:i+72]) # 9 double
        self.imu_rotation_matrix = list(struct.unpack('<9d', tmp))
        i+=72

        tmp = struct.pack('12B', *payload[i:i+12]) # 3 int
        scaling = 1.0e-4
        self.output_position_offset = list(p*scaling for p in struct.unpack('<3i', tmp))
        i+=12

        if self.extended_version_flag:
            self.smooth_mode = payload[i]
            i+=1

        fmt = '{0}B'.format(4*self.antenna_num*3) #int32[antenna_num][3]
        tmp = struct.pack(fmt, *payload[i:i+4*self.antenna_num*3]) 
        fmt = '<{0}i'.format(self.antenna_num*3) # int32
        scaling = 1.0e-4
        self.GNSS_lever_arm = list(p*scaling for p in struct.unpack(fmt, tmp))
        i += 4*self.antenna_num*3

        if self.extended_version_flag:
            fmt = '{0}B'.format(2*self.antenna_num)
            tmp = struct.pack(fmt, *payload[i:i+2*self.antenna_num]) 
            fmt = '<{0}H'.format(self.antenna_num) # uint16 (unsigned short)
            scaling = 1.0e-2
            self.GNSS_lever_arm_uncertainty = list(p*scaling for p in struct.unpack(fmt, tmp))
            i += 2*self.antenna_num

        if self.ICD_num > 0:
            self.ICD_configuration = list(p for p in payload[i:i+2*self.ICD_num])
            i += 2*self.ICD_num

        # Note: haven't verify below code snippet since have no virtual hex data with DMI info.
        # DMI configuration block. 
        if self.DMI_exists > 0:
            
            self.DMI_id = payload[i]
            i+=1

            tmp = struct.pack('8B', *payload[i:i+8]) # 1 double
            self.DMI_scale_factor = struct.unpack('<d', tmp)[0]
            i+=8

            tmp = struct.pack('12B', *payload[i:i+12]) # int32_t[3]
            scaling = 1.0e-4
            self.DMI_lever_arm = list(p*scaling for p in struct.unpack('<3i', tmp))
            i+=12

            if self.extended_version_flag:
                tmp = struct.pack('2B', *payload[i:i+2]) # uint16_t
                scaling = 1.0e-2
                self.DMI_lever_arm_uncertainty = struct.unpack('<H', tmp)[0]*scaling
                i+=2
                self.DMI_options = payload[i]

    def pack_msg_060D(self):
        '''
        pack 060D message. ref 8.4. User Configuration Setup.
        '''
        self.msg_060D_frame = b'\XAF\X20\X06\X0D'


        pass




