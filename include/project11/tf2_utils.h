#ifndef PROJECT11_TF2_UTILS_H
#define PROJECT11_TF2_UTILS_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Point.h"
#include "project11/utils.h"
#include <cmath>

namespace project11
{
  class Transformations
  {
  public:
      Transformations():m_tf_listener(m_tf_buffer)
      {
      }
      
      bool canTransform(std::string map_frame="map")
      {
        return m_tf_buffer.canTransform("earth", map_frame, ros::Time(0), ros::Duration(0.5));
      }

      geometry_msgs::Point wgs84_to_map(geographic_msgs::GeoPoint const &position, std::string map_frame="map")
      {
        geometry_msgs::Point ret;
        try
        {
          geometry_msgs::TransformStamped t = m_tf_buffer.lookupTransform(map_frame,"earth",ros::Time(0));
          LatLongDegrees p_ll;
          fromMsg(position, p_ll);
          ECEF p_ecef = p_ll;
          geometry_msgs::Point in;
          toMsg(p_ecef, in);
          tf2::doTransform(in,ret,t);
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN_STREAM("Transformations: " << ex.what());
        }
        return ret;
      }
      
      geographic_msgs::GeoPoint map_to_wgs84(geometry_msgs::Point const &point, std::string map_frame="map")
      {
          geographic_msgs::GeoPoint ret;
          try
          {
            geometry_msgs::TransformStamped t = m_tf_buffer.lookupTransform("earth",map_frame,ros::Time(0));
            geometry_msgs::Point out;
            tf2::doTransform(point,out,t);
            ECEF out_ecef;
            fromMsg(out,out_ecef);
            LatLongDegrees latlon = out_ecef;
            toMsg(latlon, ret);
          }
          catch (tf2::TransformException &ex)
          {
            ROS_WARN_STREAM("Transformations: " << ex.what());
          }
          return ret;
      }
      
      const tf2_ros::Buffer& operator()() const
      {
        return m_tf_buffer;
      }
      
    private:
      tf2_ros::Buffer m_tf_buffer;
      tf2_ros::TransformListener m_tf_listener;
    };
}

#endif
