#ifndef PROJECT11_PID_H
#define PROJECT11_PID_H

#include <rclcpp/rclcpp.hpp>
#include <project11_msgs/msg/pid_parameters.hpp>
#include <project11_msgs/msg/pid_debug.hpp>

namespace project11
{

class PID
{
public:  

  PID(rclcpp::Node::SharedPtr node, std::string prefix = "pid")
  {
    node_ = node;
    update_parameters_callback_ = node->add_post_set_parameters_callback(std::bind(&PID::updateParameters, this, std::placeholders::_1));

    parameter_prefix_ = prefix;
    if(!parameter_prefix_.empty())
      parameter_prefix_ += ".";

    node->declare_parameter<double>(parameter_prefix_+"kp", Kp_);
    node->declare_parameter<double>(parameter_prefix_+"ki", Ki_);
    node->declare_parameter<double>(parameter_prefix_+"kd", Kd_);
    node->declare_parameter<double>(parameter_prefix_+"windup_limit", windup_limit_);
    node->declare_parameter<double>(parameter_prefix_+"upper_limit", upper_limit_);
    node->declare_parameter<double>(parameter_prefix_+"lower_limit", lower_limit_);

    node->declare_parameter<double>(parameter_prefix_+"max_dt", max_dt_.seconds());
    node->declare_parameter<bool>(parameter_prefix_+"publish_debug", false);

    topic_prefix_ = prefix;
    if(!topic_prefix_.empty())
      topic_prefix_ += "/";
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
    last_timestamp_= rclcpp::Time();
  }

  double update(double process_variable, rclcpp::Time timestamp=rclcpp::Time())
  {
    if(timestamp.nanoseconds() == 0)
      timestamp = node_->get_clock()->now();

    double error = set_point_ - process_variable;
    double derivative = 0.0;
    auto dt = rclcpp::Duration::from_seconds(0.0);

    if(last_timestamp_.nanoseconds() != 0)
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
          integral_ += error*dt.seconds();
          integral_ = std::min(integral_, windup_limit_);
          integral_ = std::max(integral_, -windup_limit_);
          
          derivative = (error - last_error_)/dt.seconds();
        }
      }
    }
    last_timestamp_ = timestamp;
    last_error_ = error;

    double control_effort = Kp_*error + Ki_*integral_ + Kd_*derivative;

    control_effort = std::min(control_effort, upper_limit_);
    control_effort = std::max(control_effort, lower_limit_);

    if(debug_publisher_)
    {
      project11_msgs::msg::PIDDebug debug;
      debug.header.stamp = timestamp;
      debug.parameters.kp = Kp_;
      debug.parameters.ki = Ki_;
      debug.parameters.kd = Kd_;
      debug.parameters.windup_limit = windup_limit_;
      debug.parameters.upper_limit = upper_limit_;
      debug.parameters.lower_limit = lower_limit_;
      debug.set_point = set_point_;
      debug.process_variable = process_variable;
      debug.error = error;
      debug.dt = dt.seconds();
      debug.integral = integral_;
      debug.derivative = derivative;
      debug.control_effort = control_effort;
      debug.p = Kp_*error;
      debug.i = Ki_*integral_;
      debug.d = Kd_*derivative;
      debug_publisher_->publish(debug);
    }

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
  void updateParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    for(const auto& param: parameters)
    {
      if(param.get_name() == parameter_prefix_+"kp")
        Kp_ = param.as_double();
      if(param.get_name() == parameter_prefix_+"ki")
        Ki_ = param.as_double();
      if(param.get_name() == parameter_prefix_+"kd")
        Kd_ = param.as_double();
      if(param.get_name() == parameter_prefix_+"windup_limit")
        windup_limit_ = param.as_double();
      if(param.get_name() == parameter_prefix_+"upper_limit")
        upper_limit_ = param.as_double();
      if(param.get_name() == parameter_prefix_+"lower_limit")
        lower_limit_ = param.as_double();
      if(param.get_name() == parameter_prefix_+"max_dt")
        max_dt_ = rclcpp::Duration::from_seconds(param.as_double());
      if(param.get_name() == parameter_prefix_+"publish_debug")
      {
        if(param.as_bool())
        {
          if(!debug_publisher_)
            debug_publisher_ = node_->create_publisher<project11_msgs::msg::PIDDebug>(topic_prefix_+"debug", 10);
        }
        else
          debug_publisher_.reset();
      }
    }
  }

  double set_point_ = 0.0;
  
  rclcpp::Time last_timestamp_;
  double last_error_;

  double integral_ = 0.0;

  double Kp_ = 1.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  double windup_limit_ = 1000.0;
  double upper_limit_ = 1000.0;
  double lower_limit_ = -1000.0;

  rclcpp::Duration max_dt_ = rclcpp::Duration::from_seconds(5.0);

  /// Prepended to parameter names
  std::string parameter_prefix_;

  /// Prepended to topic names
  std::string topic_prefix_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr update_parameters_callback_;
  rclcpp::Publisher<project11_msgs::msg::PIDDebug>::SharedPtr debug_publisher_;

};

} // namespace project11

#endif