# fiware_ros_bridge
This ros package acts as a bridge between [FIWARE](https://www.fiware.org) and [ROS](http://wiki.ros.org/) through MQTT.


## Description
### `robot_cmd`
This ROS node receives a command from [FIWARE orion context broker](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) through MQTT.

When receiving a command, this node publish a ROS message to a ROS topic.

### `robot_attrs`
This ROS node subscribes a ROS topic.

When receiving a ROS message from the topic, this node publishes the received messae to FIWARE orion context broker through MQTT.

## Requirement

**ROS kinetic**

## Prepare
### Install libraries

```bash
$ cd ~/ros_ws/src
$ git clone https://github.com/RoboticBase/fiware_ros_bridge.git
$ pip install -r fiware_ros_bridge/requirements/common.txt
```

### scp cert file (if you need)
If your MQTT Broker has TLS encryption, you have to set the root certification file.

```bash
$ scp /path/to/your/cert_file.crt ${user}@${robot}:${ROS_WS}/src/fiware_ros_bridge/secrets/ca.crt
```

### configure parameters to connect MQTT broker

```bash
$ cp src/fiware_ros_bridge/config/mqtt.yaml.template src/fiware_ros_bridge/config/mqtt.yaml
$ vi src/fiware_ros_bridge/config/mqtt.yaml
```

## How to Run

```bash
$ roslaunch fiware_ros_bridge fiware_ros_bridge.launch
```

**Confirm that the log messages like below are shown.**
```text
[INFO] [1531975302.196302]: [fiware_ros_bridge.base:CmdBridge._on_connect] connected to mqtt broker, status=0
[INFO] [1531975302.204255]: [fiware_ros_bridge.base:AttrsBridge._on_connect] connected to mqtt broker, status=0
```

If `status` is not 0, mqtt.yaml and/or ca.crt is invalid.

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 TIS Inc.
