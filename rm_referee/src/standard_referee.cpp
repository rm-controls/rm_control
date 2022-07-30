//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/standard_referee.h"

namespace rm_referee
{
StandardReferee::StandardReferee(ros::NodeHandle& nh, Base& base) : HeroReferee(nh, base)
{
  StandardReferee::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &StandardReferee::manualDataCallBack, this);
}

void StandardReferee::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  HeroReferee::manualDataCallBack(data);
  flash_ui_->update("cover", ros::Time::now(), !base_.manual_to_referee_data_.cover_state);
}

}  // namespace rm_referee
