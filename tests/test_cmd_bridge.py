#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
import ssl

import rosunit

from mock import patch, MagicMock

from parameterized import parameterized, param

from std_msgs.msg import String

from fiware_ros_bridge.cmd_bridge import CmdBridge

from . import utils


class TestCmdBridge(unittest.TestCase):

    @patch('fiware_ros_bridge.cmd_bridge.rospy')
    def test_init(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_cmd_params()

        CmdBridge()
        mocked_rospy.Publisher.assert_called_once_with('/turtlebot3_bridge/cmd', String, queue_size=10)

    @parameterized.expand(utils.expand_ca_params)
    @patch('fiware_ros_bridge.base.mqtt')
    @patch('fiware_ros_bridge.base.rospy')
    @patch('fiware_ros_bridge.cmd_bridge.rospy')
    def test_connect(self, mocked_rospy, mocked_base_rospy, mocked_mqtt, use_ca, cafile, username, password):
        mocked_rospy.get_param.return_value = utils.get_cmd_params(use_ca, cafile, username, password)
        mocked_mqtt_client = mocked_mqtt.Client.return_value

        bridge = CmdBridge().connect()
        mocked_mqtt.Client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)

        if use_ca and cafile:
            mocked_mqtt_client.tls_set.assert_called_once_with('/path/to/ca.crt', tls_version=ssl.PROTOCOL_TLSv1_2)
        else:
            mocked_mqtt_client.tls_set.assert_not_called()
        if username and password:
            mocked_mqtt_client.username_pw_set.assert_called_once_with('username', 'password')
        else:
            mocked_mqtt_client.username_pw_set.assert_not_called()

        mocked_mqtt_client.connect.assert_called_once_with('testhost', port=1883, keepalive=60)
        mocked_mqtt_client.loop_start.assert_called_once_with()

        mocked_base_rospy.on_shutdown.assert_called_once_with(bridge._on_shutdown)

    @patch('fiware_ros_bridge.cmd_bridge.rospy')
    def test_start(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_cmd_params()

        CmdBridge().start()
        mocked_rospy.spin.assert_called_once_with()

    @patch('fiware_ros_bridge.cmd_bridge.rospy')
    def test__on_connect(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_cmd_params()
        mocked_client = MagicMock()

        CmdBridge()._on_connect(mocked_client, None, None, 0)
        mocked_client.subscribe.assert_called_once_with('/robot/turtlebot3/cmd')

    @parameterized.expand([
        param(mcmd='circle', rcmd='circle'),
        param(mcmd='square', rcmd='square'),
        param(mcmd='triangle', rcmd='triangle'),
        param(mcmd='cross', rcmd='stop'),
        param(mcmd='up', rcmd='up'),
        param(mcmd='down', rcmd='down'),
        param(mcmd='left', rcmd='left'),
        param(mcmd='right', rcmd='right'),
        param(mcmd='invalid', rcmd=None),
    ])
    @patch('fiware_ros_bridge.cmd_bridge.rospy')
    def test__on_message(self, mocked_rospy, mcmd, rcmd):
        mocked_rospy.get_param.return_value = utils.get_cmd_params()
        mocked_client = MagicMock()

        msg = type('msg', (object,), {'payload': 'deviceid@move|{}'.format(mcmd)})
        CmdBridge()._on_message(mocked_client, None, msg)

        if mcmd in ('circle', 'square', 'triangle', 'cross', 'up', 'down', 'left', 'right'):
            mocked_rospy.Publisher.return_value.publish.assert_called_once_with(String(data=rcmd))
            mocked_client.publish.assert_called_once_with('/robot/turtlebot3/cmdexe',
                                                          'deviceid@move|cmd {} executed successfully'.format(mcmd))
        else:
            mocked_rospy.Publisher.return_value.publish.assert_not_called()
            mocked_client.publish.assert_called_once_with('/robot/turtlebot3/cmdexe',
                                                          'deviceid@move|unknown cmd {} did not executed'.format(mcmd))


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_bridge', 'test_cmd_bridge', TestCmdBridge)
