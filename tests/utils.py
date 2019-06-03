# -*- coding: utf-8 -*-
import os
import itertools

from mock import MagicMock

from parameterized import param


def __get_mqtt_params(use_ca=False, cafile=False, username=False, password=False):
    params = {
        'mqtt': {
            'host': 'testhost',
            'port': 1883,
            'use_ca': False,
        },
    }
    if use_ca:
        params['mqtt']['use_ca'] = True
    if cafile:
        params['mqtt']['cafile'] = '/path/to/ca.crt'
        os.path.isfile = MagicMock(return_value=True)
    if username:
        params['mqtt']['username'] = 'username'
    if password:
        params['mqtt']['password'] = 'password'

    return params


def get_cmd_params(use_ca=False, cafile=False, username=False, password=False):
    params = __get_mqtt_params(use_ca=use_ca, cafile=cafile, username=username, password=password)
    params['topics'] = {
        'mqtt': {
            'cmd': '/robot/turtlebot3/cmd',
            'result': '/robot/turtlebot3/cmdexe',
        },
        'ros': '/turtlebot3_bridge/cmd',
    }
    return params


def get_attrs_params(use_ca=False, cafile=False, username=False, password=False):
    params = __get_mqtt_params(use_ca=use_ca, cafile=cafile, username=username, password=password)
    params['topics'] = {
        'mqtt': '/robot/turtlebot3/attrs',
        'ros': {
            'pos': '/turtlebot3_bridge/attrs',
            'r_mode': '/r_mode',
            'battery_state': '/battery_state',
        },
    }
    params['thresholds'] = {
        'send_delta_millisec': 1000,
    }
    params['timezone'] = 'Asia/Tokyo'
    return params


def expand_ca_params():
    use_ca = [False, True]
    cafile = [False, True]
    username = [False, True]
    password = [False, True]

    return [param(use_ca=p[0], cafile=p[1], username=p[2], password=p[3])
            for p in itertools.product(use_ca, cafile, username, password)]
