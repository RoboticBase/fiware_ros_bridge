#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from fiware_ros_bridge.cmd_bridge import CmdBridge

NODE_NAME = 'robot_cmd'


def main():
    try:
        rospy.init_node(NODE_NAME)
        CmdBridge().connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
