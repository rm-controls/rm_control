//
// Created by yezi on 22-5-21.
//

#pragma once
#include <vector>
#include <ros/ros.h>

namespace rm_common
{
class LinearInterp
{
public:
  LinearInterp() = default;
  void init(XmlRpc::XmlRpcValue& config)
  {
    ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < config.size(); i++)
    {
      ROS_ASSERT(config[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(config[i].size() == 2);
      ROS_ASSERT(config[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 config[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(config[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 config[i][1].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (!input_vector_.empty())
        if ((double)config[i][0] < input_vector_.back())
        {
          ROS_ERROR("Please sort the point's abscissa from smallest to largest. %lf < %lf", (double)config[i][0],
                    input_vector_.back());
          return;
        }
      input_vector_.push_back(config[i][0]);
      output_vector_.push_back(config[i][1]);
    }
  }
  double output(double input)
  {
    if (input >= input_vector_.back())
      return output_vector_.back();
    else if (input <= input_vector_.front())
      return output_vector_.front();
    for (size_t i = 0; i < input_vector_.size(); i++)
    {
      if (input >= input_vector_[i] && input <= input_vector_[i + 1])
        return output_vector_[i] +
               ((output_vector_[i + 1] - output_vector_[i]) / (input_vector_[i + 1] - input_vector_[i])) *
                   (input - input_vector_[i]);
    }
    ROS_ERROR("The point's abscissa aren't sorted from smallest to largest.");
    return 0;
  }

private:
  std::vector<double> input_vector_;
  std::vector<double> output_vector_;
};
}  // namespace rm_common
