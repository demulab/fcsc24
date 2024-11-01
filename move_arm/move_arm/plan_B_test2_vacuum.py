#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
"""
import sys
import math
import time
import datetime
import random
import traceback
import threading

"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""
try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.1.240')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

variables = {}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}


# Register error/warn changed callback
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] > 1 or (arm.version_number[0] == 1 and arm.version_number[1] > 1):
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(connect_changed_callback)

# Linear Motion
if not params['quit']:
    params['speed'] = 70
if not params['quit']:
    params['acc'] = 10000
for i in range(int(1)):
    if params['quit']:
        break
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[208.5, 0.1, 331.8, 180.0, 0.0, 0.0], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_suction_cup(True, wait=False, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_suction_cup, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[368.8, -0.6, 331.8, -117.1, -4.3, -4.5], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[368.8, -0.6, 98.5, -117.1, -4.3, -4.5], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[370.5, 1.1, 96.8, -116.4, -0.5, -1.4], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[371.9, 12.4, 97.5, -116.3, -4.6, -2.4], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[371.4, 18.1, 93.3, -119.9, -2.8, -3.8], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[371.4, 18.1, 473.5, -119.9, -2.8, -3.8], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[550.3, 18.1, 473.5, -116.9, -12.4, -9.7], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[532.6, 18.1, 353.7, -111.8, -7.7, -10.7], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[542.8, 18.1, 353.7, -113.5, 1.9, 0.5], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_suction_cup(False, wait=False, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_suction_cup, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_position(*[208.5, 0.1, 331.8, 180.0, 0.0, 0.0], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
        if code != 0:
            params['quit'] = True
            pprint('set_position, code={}'.format(code))

# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
