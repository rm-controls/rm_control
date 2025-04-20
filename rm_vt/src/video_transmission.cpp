//
// Created by ch on 24-11-23.
//
#include "rm_vt/video_transmission.h"

namespace rm_vt
{
void VideoTransmission::read()
{
  if (base_.serial_.available())
  {
    rx_len_ = static_cast<int>(base_.serial_.available());
    base_.serial_.read(rx_buffer_, rx_len_);
  }
  else
    return;
  uint8_t temp_buffer[256] = { 0 };
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
  for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i)
  {
    if (unpack_buffer_[k_i] == 0xA5)
    {
      frame_len = unpack(&unpack_buffer_[k_i]);
      if (frame_len != -1)
        k_i += frame_len;
    }
    if (unpack_buffer_[k_i] == 0xA9 && unpack_buffer_[k_i + 1] == 0x53)
    {
      frame_len = control_data_unpack(&unpack_buffer_[k_i]);
      if (frame_len != -1)
        k_i += frame_len;
    }
  }
  clearRxBuffer();
}

int VideoTransmission::unpack(uint8_t* rx_data)
{
  uint16_t cmd_id;
  int frame_len;
  rm_vt::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_)))
  {
    if (frame_header.data_length > 256)  // temporary and inaccurate value
    {
      ROS_INFO("discard possible wrong frames, data length: %d", frame_header.data_length);
      return 0;
    }
    frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id)
      {
        case rm_vt::CUSTOM_CONTROLLER_CMD:
        {
          rm_vt::CustomControllerData custom_controller_ref;
          rm_msgs::CustomControllerData custom_controller_data;
          memcpy(&custom_controller_ref, rx_data + 7, sizeof(rm_vt::CustomControllerData));
          for (int i = 0; i < 6; i++)
            custom_controller_data.encoder_data[i] = custom_controller_ref.encoder_data[i] * 3.14 / 18000.0;
          custom_controller_data.joystick_l_y_data = custom_controller_ref.joystick_l_y_data;
          custom_controller_data.joystick_l_x_data = custom_controller_ref.joystick_l_x_data;
          custom_controller_data.joystick_r_y_data = custom_controller_ref.joystick_r_y_data;
          custom_controller_data.joystick_r_x_data = custom_controller_ref.joystick_r_x_data;
          custom_controller_data.button_data[0] = custom_controller_ref.button1_data;
          custom_controller_data.button_data[1] = custom_controller_ref.button2_data;
          custom_controller_data.button_data[2] = custom_controller_ref.button3_data;
          custom_controller_data.button_data[3] = custom_controller_ref.button4_data;
          custom_controller_cmd_pub_.publish(custom_controller_data);
          break;
        }
        case rm_vt::ROBOT_COMMAND_CMD:
        {
          rm_vt::KeyboardMouseData keyboard_mouse_ref;
          rm_msgs::VTKeyboardMouseData keyboard_mouse_data;
          memcpy(&keyboard_mouse_ref, rx_data + 7, sizeof(rm_vt::KeyboardMouseData));
          keyboard_mouse_data.mouse_x = keyboard_mouse_ref.mouse_x;
          keyboard_mouse_data.mouse_y = keyboard_mouse_ref.mouse_y;
          keyboard_mouse_data.mouse_z = keyboard_mouse_ref.mouse_z;
          keyboard_mouse_data.left_button_down = keyboard_mouse_ref.left_button_down;
          keyboard_mouse_data.right_button_down = keyboard_mouse_ref.right_button_down;
          keyboard_mouse_data.key_w = keyboard_mouse_ref.key_w;
          keyboard_mouse_data.key_s = keyboard_mouse_ref.key_s;
          keyboard_mouse_data.key_a = keyboard_mouse_ref.key_a;
          keyboard_mouse_data.key_d = keyboard_mouse_ref.key_d;
          keyboard_mouse_data.key_shift = keyboard_mouse_ref.key_shift;
          keyboard_mouse_data.key_ctrl = keyboard_mouse_ref.key_ctrl;
          keyboard_mouse_data.key_q = keyboard_mouse_ref.key_q;
          keyboard_mouse_data.key_e = keyboard_mouse_ref.key_e;
          keyboard_mouse_data.key_r = keyboard_mouse_ref.key_r;
          keyboard_mouse_data.key_f = keyboard_mouse_ref.key_f;
          keyboard_mouse_data.key_g = keyboard_mouse_ref.key_g;
          keyboard_mouse_data.key_z = keyboard_mouse_ref.key_z;
          keyboard_mouse_data.key_x = keyboard_mouse_ref.key_x;
          keyboard_mouse_data.key_c = keyboard_mouse_ref.key_c;
          keyboard_mouse_data.key_v = keyboard_mouse_ref.key_v;
          keyboard_mouse_data.key_b = keyboard_mouse_ref.key_b;
          vt_keyboard_mouse_pub_.publish(keyboard_mouse_data);
          break;
        }
        default:
          ROS_WARN("Referee command ID %d not found.", cmd_id);
          break;
      }
      base_.video_transmission_is_online_ = true;
      last_get_data_time_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

int VideoTransmission::control_data_unpack(uint8_t* rx_data)
{
  int frame_len = 21;
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
