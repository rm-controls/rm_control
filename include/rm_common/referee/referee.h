//
// Created by peter on 2021/5/17.
//
#ifndef RM_COMMON_REFEREE_H_
#define RM_COMMON_REFEREE_H_

#include <cstdint>
#include <serial/serial.h>
#include <ros/ros.h>

#include <rm_msgs/Referee.h>
#include <rm_msgs/SuperCapacitor.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>

#include "rm_common/referee/protocol.h"

namespace rm_common {
class SuperCapacitor {
 public:
  void read(const std::vector<uint8_t> &rx_buffer);
  float getChassisPower() const { return parameters[0]; }
  float getLimitPower() const { return parameters[1]; }
  float getBufferPower() const { return parameters[2]; }
  float getCapPower() const { return parameters[3]; }
  bool is_online_ = false;
  ros::Time last_get_capacitor_data_ = ros::Time::now();
 private:
  void dtpReceivedCallBack(unsigned char receive_byte);
  void receiveCallBack(unsigned char package_id, unsigned char *data);
  static float int16ToFloat(unsigned short data0);

  float parameters[4] = {0};
  unsigned char receive_buffer_[1024] = {0};
  unsigned char ping_pong_buffer_[1024] = {0};
  unsigned int receive_buf_counter_ = 0;
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
  InteractiveData student_interactive_data_;
  GraphicDataStruct graphic_data_struct_;
};

class Referee {
 public:
  Referee() = default;
  void init();
  void read();
  void drawCircle(int center_x, int center_y, int radius, int picture_id,
                  GraphicColorType color, GraphicOperateType operate_type);
  void drawString(int x, int y, int picture_id, std::string data,
                  GraphicColorType color, GraphicOperateType operate_type);
  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);

  ros::Publisher referee_pub_;
  ros::Publisher super_capacitor_pub_;

  rm_msgs::Referee referee_pub_data_;
  rm_msgs::SuperCapacitor super_capacitor_pub_data_;
  RefereeData referee_data_{}, last_referee_data_;
  SuperCapacitor super_capacitor_;

  bool is_online_ = false;
  int robot_id_ = 0;
  int client_id_ = 0;
 private:
  int unpack(uint8_t *rx_data);
  void pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len);
  void getRobotId();
  void publishData();

  serial::Serial serial_;
  ros::Time last_get_referee_data_ = ros::Time::now();
  const std::string serial_port_ = "/dev/usbReferee";
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  static const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[k_unpack_buffer_length_];
};

// CRC verification
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char uc_crc_8);
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
uint16_t getCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc);
uint32_t verifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length);
void appendCRC16CheckSum(unsigned char *pch_message, unsigned int dw_length);
}

#endif //RM_COMMON_REFEREE_H_