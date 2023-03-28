#ifndef PROJECT11_PID_H
#define PROJECT11_PID_H

#include <ros/ros.h>
#include <project11_msgs/PIDParameters.h>
#include <project11_msgs/PIDDebug.h>

namespace project11
{

class PID
{
public:  

  void configure(ros::NodeHandle nh)
  {
    Kp_ = nh.param("Kp", Kp_);
    Ki_ = nh.param("Ki", Ki_);
    Kd_ = nh.param("Kd", Kd_);
    windup_limit_ = nh.param("windup_limit", windup_limit_);
    upper_limit_ = nh.param("upper_limit", upper_limit_);
    lower_limit_ = nh.param("lower_limit", lower_limit_);

    max_dt_ = ros::Duration(nh.param("max_dt", max_dt_.toSec()));


    set_parameters_sub_ = nh.subscribe("set_parameters", 10, &PID::parameters_callback, this);
    debug_pub_ = nh.advertise<project11_msgs::PIDDebug>("debug", 10);
  }

  double setPoint(double set_point)
  {
    set_point_ = set_point;
    return set_point_;
  }

  double setPoint() const
  {
    return set_point_;
  }

  void reset()
  {
    integral_ = 0.0;
    last_error_ = 0.0;
    last_timestamp_= ros::Time();
  }

  double update(double process_variable, ros::Time timestamp=ros::Time())
  {
    if(timestamp.is_zero())
      timestamp = ros::Time::now();

    double error = set_point_ - process_variable;
    double derivative = 0.0;
    ros::Duration dt;

    if(last_timestamp_.isValid())
    {
      if(timestamp > last_timestamp_)
      {
        dt = timestamp - last_timestamp_;
        if(dt > max_dt_)
        {
          // it's been too long, reset.
          reset();
        }
        else
        {
          integral_ += error*dt.toSec();
          integral_ = std::min(integral_, windup_limit_);
          integral_ = std::max(integral_, -windup_limit_);
          
          derivative = (error - last_error_)/dt.toSec();
        }
      }
    }
    last_timestamp_ = timestamp;
    last_error_ = error;

    double control_effort = Kp_*error + Ki_*integral_ + Kd_*derivative;

    control_effort = std::min(control_effort, upper_limit_);
    control_effort = std::max(control_effort, lower_limit_);

    project11_msgs::PIDDebug debug;
    debug.header.stamp = timestamp;
    debug.parameters.Kp = Kp_;
    debug.parameters.Ki = Ki_;
    debug.parameters.Kd = Kd_;
    debug.parameters.windup_limit = windup_limit_;
    debug.parameters.upper_limit = upper_limit_;
    debug.parameters.lower_limit = lower_limit_;
    debug.set_point = set_point_;
    debug.process_variable = process_variable;
    debug.error = error;
    debug.dt = dt.toSec();
    debug.integral = integral_;
    debug.derivative = derivative;
    debug.control_effort = control_effort;
    debug.p = Kp_*error;
    debug.i = Ki_*integral_;
    debug.d = Kd_*derivative;

    if(debug_pub_)
      debug_pub_.publish(debug);

    return control_effort;
  }

  double Kp() const
  {
    return Kp_;
  }

  double Ki() const
  {
    return Ki_;
  }

  double Kd() const
  {
    return Kd_;
  }

private:
  void parameters_callback(const project11_msgs::PIDParameters::ConstPtr &msg)
  {
    Kp_ = msg->Kp;
    Ki_ = msg->Ki;
    Kd_ = msg->Kd;
    windup_limit_ = msg->windup_limit;
    upper_limit_ = msg->windup_limit;
    lower_limit_ = msg->lower_limit;
  }

  ros::Subscriber set_parameters_sub_;
  ros::Publisher debug_pub_;

  double set_point_ = 0.0;
  
  ros::Time last_timestamp_;
  double last_error_;

  double integral_ = 0.0;

  double Kp_ = 1.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  double windup_limit_ = 1000.0;
  double upper_limit_ = 1000.0;
  double lower_limit_ = -1000.0;

  ros::Duration max_dt_ = ros::Duration(5.0);

};

} // namespace project11

#endif