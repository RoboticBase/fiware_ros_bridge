# -*- coding: utf-8 -*-
import unittest
import ssl

import rosunit

from mock import patch, call

from parameterized import parameterized

import freezegun

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

from fiware_ros_msgs.msg import r_pos

from fiware_ros_bridge.attrs_bridge import AttrsBridge

from . import utils


class TestAttrsBridge(unittest.TestCase):

    @patch('fiware_ros_bridge.attrs_bridge.rospy')
    def test_init(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()

        bridge = AttrsBridge()
        mocked_rospy.Subscriber.assert_called()
        called_args = mocked_rospy.Subscriber.call_args_list
        assert len(called_args) == 3
        assert called_args[0] == call('/turtlebot3_bridge/attrs',
                                      r_pos, bridge._on_receive_pos, queue_size=10)
        assert called_args[1] == call('/battery_state',
                                      BatteryState, bridge._on_receive_battery_state, queue_size=10)
        assert called_args[2] == call('/r_mode',
                                      String, bridge._on_receive_r_mode, queue_size=10)

    @parameterized.expand(utils.expand_ca_params)
    @patch('fiware_ros_bridge.base.mqtt')
    @patch('fiware_ros_bridge.base.rospy')
    @patch('fiware_ros_bridge.attrs_bridge.rospy')
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

    @patch('fiware_ros_bridge.attrs_bridge.rospy')
    def test_start(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()

        AttrsBridge().start()
        mocked_rospy.spin.assert_called_once_with()

    @freezegun.freeze_time('2018-01-02T03:04:05+09:00')
    @patch('fiware_ros_bridge.base.mqtt')
    @patch('fiware_ros_bridge.attrs_bridge.rospy')
    def test__on_receive_pos(self, mocked_rospy, mocked_mqtt):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()
        mocked_mqtt_client = mocked_mqtt.Client.return_value

        msg = r_pos()
        msg.x = 0.1
        msg.y = 0.2
        msg.z = 0.3
        msg.theta = 0.4

        with freezegun.freeze_time('2018-01-02T03:04:05.000000+0900'):
            bridge = AttrsBridge().connect()
        with freezegun.freeze_time('2018-01-02T03:04:06.000000+0900'):
            bridge._on_receive_pos(msg)
        payload = '2018-01-02T03:04:06.000000+0900|x|0.1|y|0.2|z|0.3|theta|0.4|r_mode|standby'
        mocked_mqtt_client.publish.assert_called_once_with('/robot/turtlebot3/attrs', payload)

    @patch('fiware_ros_bridge.base.mqtt')
    @patch('fiware_ros_bridge.attrs_bridge.rospy')
    def test__on_receive_battery_state_under_threshold(self, mocked_rospy, mocked_mqtt):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()
        mocked_mqtt_client = mocked_mqtt.Client.return_value

        with freezegun.freeze_time('2018-01-02T03:04:05.123456+09:00'):
            bridge = AttrsBridge().connect()

        with freezegun.freeze_time('2018-01-02T03:04:05.123457+09:00'):
            bridge._on_receive_battery_state(BatteryState())

        mocked_mqtt_client.publish.assert_not_called()

    @patch('fiware_ros_bridge.base.mqtt')
    @patch('fiware_ros_bridge.attrs_bridge.rospy')
    def test__on_receive_battery_state_over_threshold(self, mocked_rospy, mocked_mqtt):
        mocked_rospy.get_param.return_value = utils.get_attrs_params()
        mocked_mqtt_client = mocked_mqtt.Client.return_value

        with freezegun.freeze_time('2018-01-02T03:04:05.123456+09:00'):
            bridge = AttrsBridge().connect()

        battery = BatteryState()
        battery.voltage = 0.1
        battery.current = 0.2
        battery.charge = 0.3
        battery.capacity = 0.4
        battery.design_capacity = 0.5
        battery.percentage = 0.6

        with freezegun.freeze_time('2018-01-02T03:04:06.123457+09:00'):
            bridge._on_receive_battery_state(battery)

        payload = '2018-01-02T03:04:06.123457+0900|voltage|0.1|current|0.2|charge|0.3|capacity|0.4|' \
                  'design_capacity|0.5|percentage|0.6'
        mocked_mqtt_client.publish.assert_called_once_with('/robot/turtlebot3/attrs', payload)


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_bridge', 'test_atrs_bridge', TestAttrsBridge)
