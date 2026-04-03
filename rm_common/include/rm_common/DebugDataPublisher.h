//
// Created by wk on 2026/3/2.
//
#pragma once
#include <ros/ros.h>
#include <rm_msgs/DebugData.h>
#include <unordered_map>
#include <string>

/**
 * @brief Debug data publisher. Supports adding/updating named variables and batch publishing.
 */
class DebugDataPublisher
{
public:
  /**
   * @brief Construct and initialize the publisher.
   * @param nh         ROS NodeHandle used for advertise.
   * @param topic      Topic name, defaults to "debug_data".
   * @param queue_size Publish queue size, defaults to 10.
   */
  explicit DebugDataPublisher(ros::NodeHandle& nh, const std::string& topic = "debug_data", int queue_size = 10)
  {
    debug_data_pub_ = nh.advertise<rm_msgs::DebugData>(topic, queue_size);
  }

  /**
   * @brief Add or update a debug variable, stamped at current time.
   * @param name  Variable name. Duplicates update the existing entry.
   * @param value Variable value.
   */
  void add(const std::string& name, double value)
  {
    auto it = index_map_.find(name);
    if (it != index_map_.end())
    {
      // Update existing entry
      debug_data_.stamp[it->second] = ros::Time::now();
      debug_data_.value[it->second] = value;
    }
    else
    {
      // Insert new entry
      index_map_[name] = debug_data_.name.size();
      debug_data_.stamp.push_back(ros::Time::now());
      debug_data_.name.push_back(name);
      debug_data_.value.push_back(value);
    }
  }

  /**
   * @brief Add or update a debug variable with a custom timestamp.
   * @param name  Variable name. Duplicates update the existing entry.
   * @param value Variable value.
   * @param stamp Custom timestamp.
   */
  void add(const std::string& name, double value, const ros::Time& stamp)
  {
    auto it = index_map_.find(name);
    if (it != index_map_.end())
    {
      debug_data_.stamp[it->second] = stamp;
      debug_data_.value[it->second] = value;
    }
    else
    {
      index_map_[name] = debug_data_.name.size();
      debug_data_.stamp.push_back(stamp);
      debug_data_.name.push_back(name);
      debug_data_.value.push_back(value);
    }
  }

  /**
   * @brief Publish accumulated debug data and clear.
   */
  void publish()
  {
    if (!debug_data_.name.empty())
    {
      debug_data_pub_.publish(debug_data_);
    }
    clear();
  }

  /**
   * @brief Publish accumulated debug data without clearing. Useful for persistent variables.
   */
  void publishAndKeep()
  {
    if (!debug_data_.name.empty())
    {
      debug_data_pub_.publish(debug_data_);
    }
  }

  /**
   * @brief Clear all stored debug data.
   */
  void clear()
  {
    debug_data_.stamp.clear();
    debug_data_.name.clear();
    debug_data_.value.clear();
    index_map_.clear();
  }

private:
  ros::Publisher debug_data_pub_;                      ///< ROS publisher.
  rm_msgs::DebugData debug_data_;                      ///< Debug data to be published.
  std::unordered_map<std::string, size_t> index_map_;  ///< Name-to-index map for O(1) update.
};
