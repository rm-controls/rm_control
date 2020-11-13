//
// Created by qiayuan on 8/13/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_ORI_TOOL_H_
#define SRC_RM_COMMON_INCLUDE_ORI_TOOL_H_
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
void quatToRPY(const geometry_msgs::Quaternion &q,
               double &roll, double &pitch, double &yaw);

tf::Quaternion getAverageQuaternion(
    const std::vector<tf::Quaternion> &quaternions,
    const std::vector<double> &weights);

#endif //SRC_RM_COMMON_INCLUDE_ORI_TOOL_H_
