# -*- coding: utf-8 -*-
import re

import rospy
from std_msgs.msg import String

from fiware_ros_bridge.base import MQTTBase

from fiware_ros_bridge.logging import getLogger
logger = getLogger(__name__)

CMD_RE = re.compile(r'^(?P<device_id>.+)@(?P<command>[^|]+)\|(?P<value>.+)$')
RESULT_FMT = '{device_id}@{command}|{result}'

CMD_DEFINITIONS = {
    'circle': 'circle',
    'square': 'square',
    'triangle': 'triangle',
    'cross': 'stop',
    'up': 'up',
    'down': 'down',
    'left': 'left',
    'right': 'right',
    'return': 'return'
}


class CmdBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(CmdBridge, self).__init__(self.__params)
        self.__cmd_pub = rospy.Publisher(self.__params['topics']['ros'], String, queue_size=10)

    def start(self):
        logger.infof('CmdBridge start')
        rospy.spin()
        logger.infof('CmdBridge finish')

    def _on_connect(self, client, userdata, flags, response_code):
        super(CmdBridge, self)._on_connect(client, userdata, flags, response_code)
        client.subscribe(self.__params['topics']['mqtt']['cmd'])

    def _on_message(self, client, userdata, msg):
        payload = str(msg.payload)
        logger.infof('received message from mqtt: {}', payload)

        matcher = CMD_RE.match(payload)
        if matcher:
            device_id = matcher.group('device_id')
            command = matcher.group('command')
            value = matcher.group('value')

            if value in CMD_DEFINITIONS:
                cmd = String(data=CMD_DEFINITIONS[value])
                self.__cmd_pub.publish(cmd)

                result = 'cmd {} executed successfully'.format(value)

            else:
                result = 'unknown cmd {} did not executed'.format(value)

            client.publish(self.__params['topics']['mqtt']['result'],
                           RESULT_FMT.format(device_id=device_id, command=command, result=result))
        else:
            logger.errorf('invalid format, payload={}', payload)
