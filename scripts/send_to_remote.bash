#!/bin/bash

rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/contact', destination_topic: '/udp/contact', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/heading', destination_topic: '/udp/heading', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/heartbeat', destination_topic: '/udp/heartbeat', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/position', destination_topic: '/udp/position', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/project11/display', destination_topic: '/udp/project11/display', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/project11/mission_manager/status', destination_topic: '/udp/project11/mission_manager/status', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/sog', destination_topic: '/udp/sog', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/posmv/orientation', destination_topic: '/udp/posmv/orientation', queue_size: 0, period: 0.25}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/posmv/position', destination_topic: '/udp/posmv/position', queue_size: 0, period: 0.25}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/pano_1/thumbnail/image/compressed', destination_topic: '/pano_1/thumbnail/image/compressed', queue_size: 0, period: 0.5}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/pano_2/thumbnail/image/compressed', destination_topic: '/pano_2/thumbnail/image/compressed', queue_size: 0, period: 0.5}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/pano_6/thumbnail/image/compressed', destination_topic: '/pano_6/thumbnail/image/compressed', queue_size: 0, period: 0.5}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/project11/crab_angle/pid_debug',destination_topic: '/udp/project11/crab_angle/pid_debug', queue_size: 0, period: 0.0}"
rosservice call /operator/udp_bridge_operator/remote_advertise "{remote_host: 'snowpetrelz', remote_port: 4200, source_topic: '/udp/radar',destination_topic: '/udp/radar', queue_size: 0, period: 0.0}"

