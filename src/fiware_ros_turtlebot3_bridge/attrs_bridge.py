# -*- coding: utf-8 -*-
import datetime

import pytz

import rospy

from fiware_ros_turtlebot3_bridge.msg import r_pos
from fiware_ros_turtlebot3_bridge.base import MQTTBase

from fiware_ros_turtlebot3_bridge.logging import getLogger
logger = getLogger(__name__)

PAYLOAD_FMT = '{timestamp}|x|{x}|y|{y}|z|{z}|theta|{theta}'


class AttrsBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(AttrsBridge, self).__init__(self.__params)
        rospy.Subscriber(self.__params['topics']['ros'], r_pos, self._on_receive, queue_size=10)
        self.__attrs_topic = self.__params['topics']['mqtt']

    def start(self):
        logger.infof('AttrsBridge start')
        rospy.spin()
        logger.infof('AttrsBridge finish')

    def _on_receive(self, msg):
        logger.debugf('received message from ros : {}', str(msg).replace('\n', ' '))

        timestamp = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%dT%H:%M:%S.%f%z')
        msg = PAYLOAD_FMT.format(timestamp=timestamp, x=msg.x, y=msg.y, z=msg.z, theta=msg.theta)
        self.client.publish(self.__attrs_topic, msg)
