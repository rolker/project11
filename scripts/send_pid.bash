#!/bin/bash

rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/project11/crab_angle/control_effort', destination_topic: '/udp/project11/crab_angle/control_effort', queue_size: 0, period: 0.0}"
rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/project11/crab_angle/setpoint', destination_topic: '/udp/project11/crab_angle/setpoint', queue_size: 0, period: 0.0}"
rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/project11/crab_angle/state', destination_topic: '/udp/project11/crab_angle/state', queue_size: 0, period: 0.0}"
rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/project11/crab_angle/pid_debug', destination_topic: '/udp/project11/crab_angle/pid_debug', queue_size: 0, period: 0.0}"

