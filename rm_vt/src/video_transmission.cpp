//
// Created by ch on 24-11-23.
//
#include "rm_vt/video_transmission.h"

namespace rm_vt
{
uint16_t VideoTransmission::keyboardMaskFromLegacyFrame(const rm_vt::KeyboardMouseData& data)
{
  uint16_t mask = 0;
  mask |= static_cast<uint16_t>(data.key_w) << 0;
  mask |= static_cast<uint16_t>(data.key_s) << 1;
  mask |= static_cast<uint16_t>(data.key_a) << 2;
  mask |= static_cast<uint16_t>(data.key_d) << 3;
  mask |= static_cast<uint16_t>(data.key_shift) << 4;
  mask |= static_cast<uint16_t>(data.key_ctrl) << 5;
  mask |= static_cast<uint16_t>(data.key_q) << 6;
  mask |= static_cast<uint16_t>(data.key_e) << 7;
  mask |= static_cast<uint16_t>(data.key_r) << 8;
  mask |= static_cast<uint16_t>(data.key_f) << 9;
  mask |= static_cast<uint16_t>(data.key_g) << 10;
  mask |= static_cast<uint16_t>(data.key_z) << 11;
  mask |= static_cast<uint16_t>(data.key_x) << 12;
  mask |= static_cast<uint16_t>(data.key_c) << 13;
  mask |= static_cast<uint16_t>(data.key_v) << 14;
  mask |= static_cast<uint16_t>(data.key_b) << 15;
  return mask;
}

uint16_t VideoTransmission::keyCodeToMask(uint8_t key_code)
{
  switch (key_code)
  {
    case 'W':
      return (1u << 0);
    case 'S':
      return (1u << 1);
    case 'A':
      return (1u << 2);
    case 'D':
      return (1u << 3);
    case 0:
      return 0;
    default:
      break;
  }
  switch (key_code)
  {
    case 'Q':
      return (1u << 6);
    case 'E':
      return (1u << 7);
    case 'R':
      return (1u << 8);
    case 'F':
      return (1u << 9);
    case 'G':
      return (1u << 10);
    case 'Z':
      return (1u << 11);
    case 'X':
      return (1u << 12);
    case 'C':
      return (1u << 13);
    case 'V':
      return (1u << 14);
    case 'B':
      return (1u << 15);
    default:
      break;
  }

  if (key_code == 0x10)
    return (1u << 4);
  if (key_code == 0x11)
    return (1u << 5);
  return 0;
}

uint16_t VideoTransmission::keyboardMaskFromKeyCodes(uint16_t key_value)
{
  const uint8_t key_1 = static_cast<uint8_t>(key_value & 0xFFu);
  const uint8_t key_2 = static_cast<uint8_t>((key_value >> 8) & 0xFFu);
  uint16_t new_mask = 0;
  new_mask |= keyCodeToMask(key_1);
  new_mask |= keyCodeToMask(key_2);
  return new_mask;
}

uint16_t VideoTransmission::updateKeyboardValueStateFromKeyCodes(uint16_t key_value)
{
  if (key_value == 0)
    return keyboard_value_state_;

  keyboard_value_state_ = keyboardMaskFromKeyCodes(key_value);
  return keyboard_value_state_;
}

void VideoTransmission::publishKeyboardMouseData(uint16_t keyboard_value, int16_t mouse_x, int16_t mouse_y,
                                                 int16_t mouse_z, bool left_button_down, bool right_button_down)
{
  rm_msgs::VTKeyboardMouseData keyboard_mouse_data;
  keyboard_mouse_data.mouse_x = mouse_x;
  keyboard_mouse_data.mouse_y = mouse_y;
  keyboard_mouse_data.mouse_z = mouse_z;
  keyboard_mouse_data.left_button_down = left_button_down;
  keyboard_mouse_data.right_button_down = right_button_down;
  keyboard_mouse_data.key_w = static_cast<bool>(keyboard_value & (1u << 0));
  keyboard_mouse_data.key_s = static_cast<bool>(keyboard_value & (1u << 1));
  keyboard_mouse_data.key_a = static_cast<bool>(keyboard_value & (1u << 2));
  keyboard_mouse_data.key_d = static_cast<bool>(keyboard_value & (1u << 3));
  keyboard_mouse_data.key_shift = static_cast<bool>(keyboard_value & (1u << 4));
  keyboard_mouse_data.key_ctrl = static_cast<bool>(keyboard_value & (1u << 5));
  keyboard_mouse_data.key_q = static_cast<bool>(keyboard_value & (1u << 6));
  keyboard_mouse_data.key_e = static_cast<bool>(keyboard_value & (1u << 7));
  keyboard_mouse_data.key_r = static_cast<bool>(keyboard_value & (1u << 8));
  keyboard_mouse_data.key_f = static_cast<bool>(keyboard_value & (1u << 9));
  keyboard_mouse_data.key_g = static_cast<bool>(keyboard_value & (1u << 10));
  keyboard_mouse_data.key_z = static_cast<bool>(keyboard_value & (1u << 11));
  keyboard_mouse_data.key_x = static_cast<bool>(keyboard_value & (1u << 12));
  keyboard_mouse_data.key_c = static_cast<bool>(keyboard_value & (1u << 13));
  keyboard_mouse_data.key_v = static_cast<bool>(keyboard_value & (1u << 14));
  keyboard_mouse_data.key_b = static_cast<bool>(keyboard_value & (1u << 15));
  vt_keyboard_mouse_pub_.publish(keyboard_mouse_data);
}

void VideoTransmission::read()
{
  if (base_.serial_.available())
  {
    rx_len_ = static_cast<int>(base_.serial_.available());
    base_.serial_.read(rx_buffer_, rx_len_);
  }
  else
    return;
  uint8_t temp_buffer[k_unpack_buffer_length_] = { 0 };
  int frame_len;
  if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
    base_.video_transmission_is_online_ = false;
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
      temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
    for (int k_i = 0; k_i < rx_len_; ++k_i)
      temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      unpack_buffer_[k_i] = temp_buffer[k_i];
  }
  else
  {
    const int offset = rx_len_ - k_unpack_buffer_length_;
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      unpack_buffer_[k_i] = rx_buffer_[offset + k_i];
  }
  for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i)
  {
    if (unpack_buffer_[k_i] == 0xA5)
    {
      frame_len = unpack(&unpack_buffer_[k_i], k_unpack_buffer_length_ - k_i);
      if (frame_len > 0)
        k_i += frame_len - 1;
    }
    if (unpack_buffer_[k_i] == 0xA9 && unpack_buffer_[k_i + 1] == 0x53)
    {
      frame_len = control_data_unpack(&unpack_buffer_[k_i], k_unpack_buffer_length_ - k_i);
      if (frame_len > 0)
        k_i += frame_len - 1;
    }
  }
  clearRxBuffer();
}

int VideoTransmission::unpack(uint8_t* rx_data, int rx_data_len)
{
  uint16_t cmd_id;
  int frame_len;
  rm_vt::FrameHeader frame_header;

  if (rx_data_len < k_header_length_)
    return -1;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_)))
  {
    const uint16_t max_payload_len = k_unpack_buffer_length_ - k_header_length_ - k_cmd_id_length_ - k_tail_length_;
    if (frame_header.data_length > max_payload_len)
    {
      ROS_INFO("discard possible wrong frames, data length: %d", frame_header.data_length);
      return -1;
    }
    frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (frame_len > rx_data_len || frame_len > k_unpack_buffer_length_)
      return -1;
    if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      auto ensure_payload = [&](size_t expected_len, const char* cmd_name) {
        if (frame_header.data_length < expected_len)
        {
          ROS_WARN("Drop cmd %s due to short payload: %u < %zu", cmd_name, frame_header.data_length, expected_len);
          return false;
        }
        return true;
      };
      auto copy_payload = [&](void* dst, size_t len, const char* cmd_name) {
        if (!ensure_payload(len, cmd_name))
          return false;
        memcpy(dst, rx_data + 7, len);
        return true;
      };
      switch (cmd_id)
      {
        case rm_vt::CUSTOM_CONTROLLER_CMD:
        {
          rm_vt::CustomControllerData custom_controller_ref;
          rm_msgs::CustomControllerData custom_controller_data;
          if (!copy_payload(&custom_controller_ref, sizeof(custom_controller_ref), "CUSTOM_CONTROLLER_CMD"))
            break;
          custom_controller_data.encoder_data[0] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder1_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder1_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[1] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder2_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder2_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[5] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder3_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder3_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[3] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder4_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder4_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[4] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder5_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder5_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[2] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder6_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder6_data[1]) /
                                                   18000.0;
          custom_controller_data.joystick_l_y_data = ((uint16_t)(custom_controller_ref.joystick_l_x_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_l_x_data[1]);
          custom_controller_data.joystick_l_x_data = ((uint16_t)(custom_controller_ref.joystick_l_y_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_l_y_data[1]);
          custom_controller_data.joystick_r_y_data = ((uint16_t)(custom_controller_ref.joystick_r_x_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_r_x_data[1]);
          custom_controller_data.joystick_r_x_data = ((uint16_t)(custom_controller_ref.joystick_r_y_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_r_y_data[1]);
          custom_controller_data.button_data[0] = static_cast<bool>(custom_controller_ref.button1_data);
          custom_controller_data.button_data[1] = static_cast<bool>(custom_controller_ref.button2_data);
          custom_controller_data.button_data[2] = static_cast<bool>(custom_controller_ref.button3_data);
          custom_controller_data.button_data[3] = static_cast<bool>(custom_controller_ref.button4_data);
          custom_controller_cmd_pub_.publish(custom_controller_data);
          break;
        }
        case rm_vt::KEYBOARD_MOUSE_CMD:
        {
          rm_vt::KeyboardMouseData2026 keyboard_mouse_ref;
          if (!copy_payload(&keyboard_mouse_ref, sizeof(keyboard_mouse_ref), "KEYBOARD_MOUSE_CMD"))
            break;

          const uint16_t keyboard_value = updateKeyboardValueStateFromKeyCodes(keyboard_mouse_ref.key_value);
          publishKeyboardMouseData(keyboard_value, static_cast<int16_t>(keyboard_mouse_ref.x_position),
                                   static_cast<int16_t>(keyboard_mouse_ref.y_position), 0,
                                   keyboard_mouse_ref.mouse_left == 1, keyboard_mouse_ref.mouse_right == 1);
          break;
        }
        case rm_vt::ROBOT_COMMAND_CMD:
        {
          rm_vt::KeyboardMouseData keyboard_mouse_ref;
          if (!copy_payload(&keyboard_mouse_ref, sizeof(keyboard_mouse_ref), "ROBOT_COMMAND_CMD"))
            break;
          const uint16_t keyboard_value = keyboardMaskFromLegacyFrame(keyboard_mouse_ref);
          keyboard_value_state_ = keyboard_value;
          publishKeyboardMouseData(keyboard_value, keyboard_mouse_ref.mouse_x, keyboard_mouse_ref.mouse_y,
                                   keyboard_mouse_ref.mouse_z, keyboard_mouse_ref.left_button_down != 0,
                                   keyboard_mouse_ref.right_button_down != 0);
          break;
        }
        case rm_vt::ROBOT_TO_CUSTOM_CMD:
        {
          rm_vt::RobotToCustomData payload;
          if (!copy_payload(&payload, sizeof(payload), "ROBOT_TO_CUSTOM_CMD"))
            break;
          std_msgs::UInt8MultiArray msg;
          msg.data.assign(payload.data, payload.data + sizeof(payload.data));
          robot_to_custom_pub_.publish(msg);
          break;
        }
        case rm_vt::ROBOT_TO_CUSTOM_CMD_2:
        {
          rm_vt::RobotToCustomData2 payload;
          if (!copy_payload(&payload, sizeof(payload), "ROBOT_TO_CUSTOM_CMD_2"))
            break;
          std_msgs::UInt8MultiArray msg;
          msg.data.assign(payload.data, payload.data + sizeof(payload.data));
          robot_to_custom2_pub_.publish(msg);
          break;
        }
        case rm_vt::CUSTOM_TO_ROBOT_CMD:
        {
          rm_vt::RobotToCustomData payload;
          if (!copy_payload(&payload, sizeof(payload), "CUSTOM_TO_ROBOT_CMD"))
            break;
          std_msgs::UInt8MultiArray msg;
          msg.data.assign(payload.data, payload.data + sizeof(payload.data));
          custom_to_robot_pub_.publish(msg);
          break;
        }
        default:
          ROS_WARN("Video transmission command ID %d not found.", cmd_id);
          break;
      }
      base_.video_transmission_is_online_ = true;
      last_get_data_time_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

int VideoTransmission::control_data_unpack(uint8_t* rx_data, int rx_data_len)
{
  const int frame_len = 21;
  if (rx_data_len < frame_len)
    return -1;

  if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
  {
    rm_vt::ControlData control_ref;
    rm_msgs::VTReceiverControlData control_data;
    memcpy(&control_ref, rx_data + 2, sizeof(rm_vt::ControlData));
    control_data.joystick_r_x = (control_ref.joystick_r_x - 1024.0) / 660.0;
    control_data.joystick_r_y = (control_ref.joystick_r_y - 1024.0) / 660.0;
    control_data.joystick_l_y = (control_ref.joystick_l_y - 1024.0) / 660.0;
    control_data.joystick_l_x = (control_ref.joystick_l_x - 1024.0) / 660.0;
    control_data.mode_switch = control_ref.mode_switch;
    control_data.pause_button = control_ref.pause_button;
    control_data.custom_button_l = control_ref.custom_button_l;
    control_data.custom_button_r = control_ref.custom_button_r;
    control_data.wheel = (control_ref.wheel - 1024.0) / 660.0;
    control_data.trigger = control_ref.trigger;
    control_data.mouse_x = control_ref.mouse_x;
    control_data.mouse_y = control_ref.mouse_y;
    control_data.mouse_wheel = control_ref.mouse_wheel;
    control_data.mouse_left_down = control_ref.mouse_left_down;
    control_data.mouse_right_down = control_ref.mouse_right_down;
    control_data.mouse_mid_down = control_ref.mouse_mid_down;
    control_data.key_w = control_ref.key_w;
    control_data.key_s = control_ref.key_s;
    control_data.key_a = control_ref.key_a;
    control_data.key_d = control_ref.key_d;
    control_data.key_shift = control_ref.key_shift;
    control_data.key_ctrl = control_ref.key_ctrl;
    control_data.key_q = control_ref.key_q;
    control_data.key_e = control_ref.key_e;
    control_data.key_r = control_ref.key_r;
    control_data.key_f = control_ref.key_f;
    control_data.key_g = control_ref.key_g;
    control_data.key_z = control_ref.key_z;
    control_data.key_x = control_ref.key_x;
    control_data.key_c = control_ref.key_c;
    control_data.key_v = control_ref.key_v;
    control_data.key_b = control_ref.key_b;
    vt_receiver_control_pub_.publish(control_data);

    base_.video_transmission_is_online_ = true;
    last_get_data_time_ = ros::Time::now();
    return frame_len;
  }
  return -1;
}
}  // namespace rm_vt
