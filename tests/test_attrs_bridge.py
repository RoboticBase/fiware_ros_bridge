# -*- coding: utf-8 -*-
import unittest
import ssl

import rosunit

from mock import patch

from parameterized import parameterized

import freezegun

from fiware_ros_turtlebot3_bridge.msg import r_pos
from fiware_ros_turtlebot3_bridge.attrs_bridge import AttrsBridge

from . import utils


class TestAttrsBridge(unittest.TestCase):

    @patch('fiware_ros_turtlebot3_bridge.attrs_bridge.rospy')
    def test_init(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()

        bridge = AttrsBridge()
        mocked_rospy.Subscriber.assert_called_once_with('/turtlebot3_bridge/attrs', r_pos, bridge._on_receive, queue_size=10)

    @parameterized.expand(utils.expand_ca_params)
    @patch('fiware_ros_turtlebot3_bridge.base.mqtt')
    @patch('fiware_ros_turtlebot3_bridge.base.rospy')
    @patch('fiware_ros_turtlebot3_bridge.attrs_bridge.rospy')
    def test_connect(self, mocked_rospy, mocked_base_rospy, mocked_mqtt, use_ca, cafile, username, password):
        mocked_rospy.get_param.return_value = utils.get_attrs_params(use_ca, cafile, username, password)
        mocked_mqtt_client = mocked_mqtt.Client.return_value

        bridge = AttrsBridge().connect()
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

    @patch('fiware_ros_turtlebot3_bridge.attrs_bridge.rospy')
    def test_start(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()

        AttrsBridge().start()
        mocked_rospy.spin.assert_called_once_with()

    @freezegun.freeze_time('2018-01-02T03:04:05+09:00')
    @patch('fiware_ros_turtlebot3_bridge.base.mqtt')
    @patch('fiware_ros_turtlebot3_bridge.attrs_bridge.rospy')
    def test__on_receive(self, mocked_rospy, mocked_mqtt):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()
        mocked_mqtt_client = mocked_mqtt.Client.return_value

        msg = r_pos()
        msg.x = 0.1
        msg.y = 0.2
        msg.z = 0.3
        msg.theta = 0.4

        AttrsBridge().connect()._on_receive(msg)
        payload = '2018-01-02T03:04:05.000000+0900|x|0.1|y|0.2|z|0.3|theta|0.4'
        mocked_mqtt_client.publish.assert_called_once_with('/robot/turtlebot3/attrs', payload)


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlebot3_bridge', 'test_atrs_bridge', TestAttrsBridge)
