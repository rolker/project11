#!/bin/env python3

import rospy
from project11_msgs.msg import PlatformList, Platform, NavSource
rospy.init_node("platform_sender")

namespace = rospy.get_namespace()
print('namespace', namespace)
if len(namespace) > 1 and namespace[0] == '/':
  namespace = namespace[1:]
#namespace = namespace.split('/')[0]
if len(namespace) > 0 and namespace[-1] == '/':
  namespace = namespace[:-1]
print('trimmed namespace',namespace)

platform_pub = rospy.Publisher("/project11/platforms", PlatformList, queue_size=5)

def timerCallback(event):
  try:
    platforms = rospy.get_param("project11/platforms")
    pl = PlatformList()
    for name in platforms:
      platform = platforms[name]
      p = Platform()
      p.name = name
      p.platform_namespace = namespace
      try:
        p.robot_description = platform['robot_description']
      except KeyError:
        pass
      try:
        nav_sources = platform['nav_sources']
        for ns_name in nav_sources:
          nav_source = nav_sources[ns_name]
          ns = NavSource()
          ns.name = ns_name
          try:
            ns.position_topic = nav_source['position_topic']
          except KeyError:
            pass
          try:
            ns.orientation_topic = nav_source['orientation_topic']
          except KeyError:
            pass
          try:
            ns.velocity_topic = nav_source['velocity_topic']
          except KeyError:
            pass
          p.nav_sources.append(ns)
      except KeyError:
        pass

      try:
        p.width = platform['width']
      except KeyError:
        pass
      try:
        p.length = platform['length']
      except KeyError:
        pass
      try:
        p.reference_x = platform['reference_x']
      except KeyError:
        pass
      try:
        p.reference_y = platform['reference_y']
      except KeyError:
        pass
      try:
        p.color.r = platform['color']['red']
      except KeyError:
        pass
      try:
        p.color.g = platform['color']['green']
      except KeyError:
        pass
      try:
        p.color.b = platform['color']['blue']
      except KeyError:
        pass

      p.color.a = 1.0
      try:
        p.color.a = platform['color']['alpha']
      except KeyError:
        pass

      pl.platforms.append(p)
    platform_pub.publish(pl)
  
  except:
    pass

timer = rospy.Timer(rospy.Duration(secs=1.0), timerCallback)

rospy.spin()
