//
// Low Pass Filter for ROS2
//

#pragma once

#include <rclcpp/rclcpp.hpp>

class LowPassFilter {
public:
  explicit LowPassFilter(double cutoff_freq = 100.0);
  
  void input(double input_value, const rclcpp::Time& stamp);
  double output() const;
  
  void setCutoffFreq(double cutoff_freq);
  double getCutoffFreq() const;
  
  void reset();

private:
  double cutoff_freq_;
  double output_;
  double previous_input_;
  rclcpp::Time previous_time_;
  bool is_first_input_;
  
  double alpha_;
  
  void updateAlpha(double dt);
}; 