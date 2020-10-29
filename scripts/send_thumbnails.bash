#!/bin/bash

rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/pano_1/thumbnail/image/compressed', destination_topic: '/pano_1/thumbnail/image/compressed', queue_size: 0, period: 2.0}"
rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/pano_2/thumbnail/image/compressed', destination_topic: '/pano_2/thumbnail/image/compressed', queue_size: 0, period: 2.0}"
rosservice call /vehicle/udp_bridge_vehicle/remote_advertise "{remote_host: '192.168.100.199', remote_port: 4201, source_topic: '/pano_6/thumbnail/image/compressed', destination_topic: '/pano_6/thumbnail/image/compressed', queue_size: 0, period: 2.0}"

