# -*- coding: utf-8 -*
"""
Created on 2019-04-09
@author: Ocean
"""

import json
import math

def load_configuration(json_file):
    '''
    load properties from json_file
    returns: json properties when load successfully.
             None when load failed.
    '''
    try:
        with open(json_file) as json_data:
            return json.load(json_data)
    # except (ValueError, KeyError, TypeError) as error:
    except Exception as e:
        print(e)
        return None

def check_sum(data):
    ''' Calculate the checksum code
    '''
    checksum_a = checksum_b = 0
    for i in data:
        checksum_a += i
        checksum_b += checksum_a

    checksum_a %= 256
    checksum_b %= 256
    return checksum_a, checksum_b

def cal_attitude(q0, q1, q2, q3):
    '''
    (q0, q1, q2, q3) are (Scalar, X, Y, Z) respective.
    Note that the quaternion in ROS is defined as (X, Y, Z, scalar). 

    return: (roll, pitch, heading), unit: radians
    '''
    q0_2 = q0 * q0
    q1_2 = q1 * q1
    q2_2 = q2 * q2
    q3_2 = q3 * q3

    c11 = q0_2 + q1_2 - q2_2 - q3_2
    c21 = 2.0 * (q1 * q2 + q0 * q3)
    c31 = 2.0 * (q1 * q3 - q0 * q2)
    c32 = 2.0 * (q2 * q3 + q0 * q1)
    c33 = q0_2 - q1_2 - q2_2 + q3_2

    roll = 0.0
    pitch = 0.0
    heading = 0.0

    if abs(c31) < 0.9999:
        pitch = math.atan(-1*c31 / math.sqrt(c32 * c32 + c33 * c33))
        roll = math.atan2(c32, c33)
        heading = math.atan2(c21, c11)
    return (roll, pitch, heading)