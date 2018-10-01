# -*- coding: utf-8 -*-
import datetime
from threading import Lock

import pytz

import rospy
from sensor_msgs.msg import BatteryState

from fiware_ros_turtlebot3_msgs.msg import r_pos

from fiware_ros_turtlebot3_bridge.base import MQTTBase

from fiware_ros_turtlebot3_bridge.logging import getLogger
logger = getLogger(__name__)

POS_PAYLOAD_FMT = '{timestamp}|x|{x}|y|{y}|z|{z}|theta|{theta}'
BATTERY_STATE_PAYLOAD_FMT = '{timestamp}|voltage|{voltage}|current|{current}|charge|{charge}|capacity|{capacity}|' \
                            'design_capacity|{design_capacity}|percentage|{percentage}'


class AttrsBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(AttrsBridge, self).__init__(self.__params)
        rospy.Subscriber(self.__params['topics']['ros']['pos'], r_pos, self._on_receive_pos, queue_size=10)
        rospy.Subscriber(self.__params['topics']['ros']['battery_state'],
                         BatteryState, self._on_receive_battery_state, queue_size=10)
        self.__attrs_topic = self.__params['topics']['mqtt']

        self.__tz = pytz.timezone(self.__params['timezone'])
        self.__send_delta_ms = self.__params['thresholds']['battery_state']['send_delta_millisec']
        self.__prev_ms = datetime.datetime.now(self.__tz)
        self.__lock = Lock()

    def start(self):
        logger.infof('AttrsBridge start')
        rospy.spin()
        logger.infof('AttrsBridge finish')

    def _on_receive_pos(self, pos):
        timestamp = datetime.datetime.now(self.__tz).strftime('%Y-%m-%dT%H:%M:%S.%f%z')
        msg = POS_PAYLOAD_FMT.format(timestamp=timestamp, x=pos.x, y=pos.y, z=pos.z, theta=pos.theta)
        self.client.publish(self.__attrs_topic, msg)

    def _on_receive_battery_state(self, battery_state):
        now = datetime.datetime.now(self.__tz)
        if now >= self.__prev_ms + datetime.timedelta(milliseconds=self.__send_delta_ms) and self.__lock.acquire(False):
            self.__prev_ms = now

            timestamp = now.strftime('%Y-%m-%dT%H:%M:%S.%f%z')
            msg = BATTERY_STATE_PAYLOAD_FMT.format(timestamp=timestamp,
                                                   voltage=battery_state.voltage,
                                                   current=battery_state.current,
                                                   charge=battery_state.charge,
                                                   capacity=battery_state.capacity,
                                                   design_capacity=battery_state.design_capacity,
                                                   percentage=battery_state.percentage)
            self.client.publish(self.__attrs_topic, msg)

            self.__lock.release()
