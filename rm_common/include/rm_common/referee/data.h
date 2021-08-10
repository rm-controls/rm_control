//
// Created by peter on 2021/7/17.
//

#ifndef RM_COMMON_REFEREE_DATA_H_
#define RM_COMMON_REFEREE_DATA_H_

#include "rm_common/referee/protocol.h"

namespace rm_common {
struct CapacityData {
  double chassis_power_;
  double limit_power_;
  double buffer_power_;
  double cap_power_;
  bool is_online_ = false;
};
struct RefereeData {
  GameStatus game_status_;
  GameResult game_result_;
  GameRobotHp game_robot_hp_;
  DartStatus dart_status_;
  IcraBuffDebuffZoneStatus icra_buff_debuff_zone_status;
  EventData event_data_;
  SupplyProjectileAction supply_projectile_action_;
  RefereeWarning referee_warning_;
  DartRemainingTime dart_remaining_time_;
  GameRobotStatus game_robot_status_;
  PowerHeatData power_heat_data_;
  GameRobotPos game_robot_pos_;
  Buff buff_;
  AerialRobotEnergy aerial_robot_energy_;
  RobotHurt robot_hurt_;
  ShootData shoot_data_;
  BulletRemaining bullet_remaining_;
  RfidStatus rfid_status_;
  DartClientCmd dart_client_cmd_;
  InteractiveData interactive_data;
  GraphConfig graphic_data_struct_;
  CapacityData capacity_data;
  std::string robot_color_;
  int robot_id_;
  bool is_online_ = false;
};
}

#endif //RM_COMMON_REFEREE_DATA_H_
