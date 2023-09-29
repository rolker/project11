#!/usr/bin/env python3

import rospy
import tf2_ros

from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

import project11.wgs84
import project11.geodesic

from tf2_geometry_msgs import do_transform_pose
from tf2_geometry_msgs import do_transform_point
from tf.transformations import quaternion_about_axis
from tf.transformations import euler_from_quaternion

import math
import copy

def distanceBearingDegrees(start_lat, start_lon, dest_lat, dest_lon):
    """ Returns distance and bearing of a geodesic line.

    Uses project11.geodesic library to determine bearing (degrees, NED)
    from start lat/lon to destination lat/lon.

    TODO: Write Args documentation.

    Returns:
        A float, bearing from start lat/lon to dest lat/lon, degrees, NED.
    """
    start_lat_rad = math.radians(start_lat)
    start_lon_rad = math.radians(start_lon)

    dest_lat_rad = math.radians(dest_lat)
    dest_lon_rad = math.radians(dest_lon)
    
    path_azimuth, path_distance = project11.geodesic.inverse( start_lon_rad, start_lat_rad, dest_lon_rad, dest_lat_rad)
    return path_distance, math.degrees(path_azimuth)

def headingToYaw(heading):
    """ Convert heading (degrees, NED) to yaw (degrees, ENU).

    Args:
        heading: float heading: degrees, NED
        
    Returns:
        Float, yaw: degrees, ENU
    """
    return 90.0-heading

def yawToHeading(yaw):
    """ Convert yaw (degrees, ENU) to heading (degrees, NED).

    Args:
        yaw: float  yaw: degrees, ENU
        
    Returns:
        Float, heading: degrees, NED
    """
    return 90.0-yaw

def yawRadiansToQuaternionMsg(yaw):
    """ Convert yaw (radians, ENU) to a quaternion msg.

    Args:
        yaw: float  yaw: degrees, ENU
        
    Returns:
        geometry_msgs/Quaternion
    """
    ret = Quaternion() 
    if yaw is not None:
        quat = quaternion_about_axis(yaw, (0,0,1))
        ret.x = quat[0]
        ret.y = quat[1]
        ret.z = quat[2]
        ret.w = quat[3]
    return ret

def headingToQuaternionMsg(heading):
    """ Convert heading (degrees, NED) to a quaternion msg.

    Args:
        heading: float heading: degrees, NED
        
    Returns:
        geometry_msgs/Quaternion
    """
    ret = Quaternion() 
    if heading is not None:
        yaw = math.radians(headingToYaw(heading))
    else:
        yaw = None
    return yawRadiansToQuaternionMsg(yaw)

def transformPointToGeoPoint(point, transform):
    """Transforms a Point or PointStamped to a GeoPointStamped using given transform to earth frame"""
    ps = point
    if not hasattr(point, 'header'):
        ps = PointStamped()
        ps.point = point
    ecef = do_transform_point(point, transform)
    latlon = project11.wgs84.fromECEFtoLatLong(ecef.point.x, ecef.point.y, ecef.point.z)
    gp = GeoPointStamped()
    gp.position.latitude = math.degrees(latlon[0])
    gp.position.longitude = math.degrees(latlon[1])
    gp.position.altitude = latlon[2]
    gp.header = copy.deepcopy(ps.header)
    gp.header.frame_id = 'wgs84'
    return gp

def transformPoseToGeoPose(pose, transform):
    """Transforms a Pose to a GeoPose using given transform to earth frame"""
    gp = GeoPose()
    gp.position = transformPointToGeoPoint(pose.position, transform)
    gp.orientation = copy.deepcopy(pose.pose.orientation)
    return gp

class EarthTransforms(object):
    """Help transform coordinates between Lat/Lon and map space"""
    def __init__(self, tfBuffer = None, map_frame = None):

        if tfBuffer is None:
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
        else:
            self.tfBuffer = tfBuffer

        if map_frame is None:
            self.map_frame = rospy.get_param("~map_frame", "map")
        else:
            self.map_frame = map_frame

        

    def geoToPose(self, lat, lon, heading=None, frame_id=None, time=rospy.Time(), timeout=rospy.Duration(0)):
        """ Returns a PoseStamped in given frame from given geographic position.
        
        Args:
          lat: A float, latitude, degrees
          lon: A float, longitude, degrees
          heading: degrees, NED.
          frame_id: A string, target TF frame.

        """

        if frame_id is None:
            frame_id = self.map_frame

        try:
            earth_to_frame = self.tfBuffer.lookup_transform(frame_id, "earth", time, timeout)
        except Exception as e:
            rospy.logerr("mission_manager: Cannot lookup transform from {} to <earth>".format(frame_id))
            rospy.logerr(e)
            return None

        ecef = project11.wgs84.toECEFfromDegrees(lat, lon)
        ecef_pose = PoseStamped()
        ecef_pose.pose.position.x = ecef[0]
        ecef_pose.pose.position.y = ecef[1]
        ecef_pose.pose.position.z = ecef[2]
        ret = do_transform_pose(ecef_pose, earth_to_frame)

        if heading is not None:
            ret.pose.orientation = headingToQuaternionMsg(heading)

        return ret

    def mapToEarthTransform(self, map_frame = None, timestamp=rospy.Time()):
        if map_frame is None:
            map_frame = self.map_frame

        try:
            return self.tfBuffer.lookup_transform("earth", map_frame, timestamp)
        except Exception as e:
            rospy.logerr("Cannot lookup transform from <earth> to {}".format(map_frame))
            rospy.logerr(e)

    def poseListToGeoPoseList(self, poses, map_frame = None):
        map_to_earth = self.mapToEarthTransform(map_frame)
        if map_to_earth is not None:
            ret = []
            for p in poses:
                ret.append(transformPoseToGeoPose(p, map_to_earth))
            return ret

    def pointListToGeoPointList(self, points, map_frame = None):
        """ Transforms a list of Point or PointStamped messages to GeoPointStamped

        The frame_id of a PointStamped will be used if not empty.
        The map_frame if not None, or the default map_frame will be used for Point
        messages and PointStamped messages with an empty frame_id.
        """
        transforms = {}
        ret = []
        default_time = rospy.Time()
        for p in points:
            if hasattr(p, 'header'):
                transform_key = (p.header.frame_id, p.header.stamp)
            else:
                transform_key = ('', default_time)
            if not transform_key in transforms:
                frame_id = transform_key[0]
                if frame_id == '':
                    frame_id = map_frame
                transforms[transform_key] = self.mapToEarthTransform(frame_id, transform_key[1])
            if transforms[transform_key] is not None:
                ret.append(transformPointToGeoPoint(p, transforms[transform_key]))
            else:
                ret.append(None)
        return ret

    def pointToGeoPoint(self, point):
        try:
            map_to_earth = self.tfBuffer.lookup_transform("earth", point.header.frame_id, rospy.Time())
        except Exception as e:
            rospy.logerr("Cannot lookup transform from <earth> to {}".format(point.header.frame_id))
            rospy.logerr(e)
            return None
        ecef = do_transform_point(point, map_to_earth)
        latlon = project11.wgs84.fromECEFtoLatLong(ecef.point.x, ecef.point.y, ecef.point.z)
        gp = GeoPointStamped()
        gp.position.latitude = math.degrees(latlon[0])
        gp.position.longitude = math.degrees(latlon[1])
        gp.position.altitude = latlon[2]
        gp.header = copy.deepcopy(point.header)
        gp.header.frame_id = 'wgs84'
        return gp

        



class RobotNavigation(EarthTransforms):
    """Keeps track of robot's navigation state"""

    def __init__(self, tfBuffer = None):
        EarthTransforms.__init__(self, tfBuffer)
        self.odometry = None

        self.odom_sub = rospy.Subscriber('project11/odom', Odometry, self.odometryCallback, queue_size = 1)


    def odometryCallback(self, msg):
        """ Stores navigation Odometry as class attribute.

        Args:
          msg: A nav_msgs/Odometry message.
        """
        self.odometry = msg
        if self.map_frame is None:
            self.map_frame = msg.header.frame_id

    def positionLatLon(self):
        """ Return last known position lat/lon of the vehicle.

        Position is determined as...
        1. Use the frame_id value in the odometry message to lookup the 
        tf transfrom from the "earth" frame to the frame_id.  
        2. Transform odometry.pose to ECEF frame
        2. The wgs84.py module from project11 is used to transfrom 
        ECEF -> lat/lon

        Returns:
          A tuple of length three with 
          Lat/Long in radians and altitude in meters.
        """
        if self.odometry is None:
            rospy.logwarn_throttle(2, "There is no current odomentry, "
                          "so can't determine position!")
            return None
        
        try:
            odom_to_earth = self.tfBuffer.lookup_transform("earth", self.odometry.header.frame_id, rospy.Time())
        except Exception as e:
            rospy.logerr("Cannot lookup transform from <earth>"
                         " to odometry frame_id")
            rospy.logerr(e)
            return None
        # Function from tf2_geoemetry_msgs
        ecef = do_transform_pose(self.odometry.pose, odom_to_earth).pose.position
        return project11.wgs84.fromECEFtoLatLong(ecef.x, ecef.y, ecef.z)

    def heading(self):
        """Uses current odometry message to return heading in degrees NED.

        TODO: This should not be a method of the object.  Should be a general
              purpose function, probably in project11 module.  
              Not specific to this program.

        Returns:
          A float for heading, degrees, NED.
        """
        
        if self.odometry is not None:
            o = self.odometry.pose.pose.orientation
            q = (o.x, o.y, o.z, o.w)
            return 90.0-math.degrees(euler_from_quaternion(q)[2])

    def distanceBearingTo(self, lat, lon):
        """ Returns distance from current vehicle position to given location.

        Uses position() function and lat/lon arguments to report 
        distance in meters.

        Args:
          lat: A float, latitude, degrees
          lon: A float, longitude, degrees
        
        Returns: 
          Tuple containing a float, distance in meters from current position (lat, lon)  to lat, lon)
          and a float, bearing to target location, degrees, NED.

        """
        p_rad = self.positionLatLon()
        if p_rad is None:
            return None, None
        current_lat_rad = p_rad[0]
        current_lon_rad = p_rad[1]
        target_lat_rad = math.radians(lat)
        target_lon_rad = math.radians(lon)
        azimuth, distance = project11.geodesic.inverse(current_lon_rad, current_lat_rad, target_lon_rad, target_lat_rad)
        return distance, math.degrees(azimuth)

