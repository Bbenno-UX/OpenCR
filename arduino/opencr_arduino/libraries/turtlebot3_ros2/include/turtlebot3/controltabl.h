

///home/bachenbenno/.arduino15/packages/OpenCR/hardware/OpenCR/1.5.2/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp
#ifndef CONTROLTABL
#define CONTROLTABL
#define SERIAL_DXL_SLAVE Serial
#define ANZAHL_MAMUT_AKTOREN 4
#include "../../include/turtlebot3/turtlebot3.h"
#include <stddef.h>
#include <stdint.h>
const uint8_t ID_DXL_SLAVE = 200;
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
enum ControlTableItemAddr{
  ADDR_MODEL_INFORM    = 2,
  
  ADDR_MILLIS          = 10,

  ADDR_DEBUG_MODE      = 14,  
  ADDR_CONNECT_ROS2    = 15,
  ADDR_CONNECT_MANIP   = 16,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,

  ADDR_ILLUMINATION    = 30,
  ADDR_IR              = 34,
  ADDR_SORNA           = 38,

  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,

  ADDR_SOUND           = 50,

  ADDR_IMU_RECALIBRATION  = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X       = 72,
  ADDR_LINEAR_ACC_Y       = 76,
  ADDR_LINEAR_ACC_Z       = 80,
  ADDR_MAGNETIC_X         = 84,
  ADDR_MAGNETIC_Y         = 88,
  ADDR_MAGNETIC_Z         = 92,
  ADDR_ORIENTATION_W      = 96,
  ADDR_ORIENTATION_X      = 100,
  ADDR_ORIENTATION_Y      = 104,
  ADDR_ORIENTATION_Z      = 108,
  
  ADDR_PRESENT_CURRENT_L  = 120,
  ADDR_PRESENT_CURRENT_R  = 124,
  ADDR_PRESENT_VELOCITY_L = 128,
  ADDR_PRESENT_VELOCITY_R = 132,
  ADDR_PRESENT_POSITION_L = 136,
  ADDR_PRESENT_POSITION_R = 140,
  
  ADDR_MOTOR_CONNECT      = 148,
  ADDR_MOTOR_TORQUE       = 149,
  ADDR_CMD_VEL_LINEAR_X   = 150,
  ADDR_CMD_VEL_LINEAR_Y   = 154,
  ADDR_CMD_VEL_LINEAR_Z   = 158,
  ADDR_CMD_VEL_ANGULAR_X  = 162,
  ADDR_CMD_VEL_ANGULAR_Y  = 166,
  ADDR_CMD_VEL_ANGULAR_Z  = 170,
  ADDR_PROFILE_ACC_L      = 174,
  ADDR_PROFILE_ACC_R      = 178,

  ADDR_TORQUE_JOINT             = 199,

  ADDR_GOAL_POSITION_JOINT_1    = 200,
  ADDR_GOAL_POSITION_JOINT_2    = 204,
  ADDR_GOAL_POSITION_JOINT_3    = 208,
  ADDR_GOAL_POSITION_JOINT_4    = 212,
  ADDR_GOAL_POSITION_GRIPPER    = 216,
  ADDR_GOAL_POSITION_WR_JOINT   = 220,
  ADDR_GOAL_POSITION_WR_GRIPPER = 221,
  ADDR_GOAL_POSITION_RD         = 222,

  ADDR_PRESENT_POSITION_JOINT_1 = 224,
  ADDR_PRESENT_POSITION_JOINT_2 = 228,
  ADDR_PRESENT_POSITION_JOINT_3 = 232,
  ADDR_PRESENT_POSITION_JOINT_4 = 236,
  ADDR_PRESENT_POSITION_GRIPPER = 240,

  ADDR_PRESENT_VELOCITY_JOINT_1 = 244,
  ADDR_PRESENT_VELOCITY_JOINT_2 = 248,
  ADDR_PRESENT_VELOCITY_JOINT_3 = 252,
  ADDR_PRESENT_VELOCITY_JOINT_4 = 256,
  ADDR_PRESENT_VELOCITY_GRIPPER = 260,

  ADDR_PRESENT_CURRENT_JOINT_1  = 264,
  ADDR_PRESENT_CURRENT_JOINT_2  = 266,
  ADDR_PRESENT_CURRENT_JOINT_3  = 268,
  ADDR_PRESENT_CURRENT_JOINT_4  = 270,
  ADDR_PRESENT_CURRENT_GRIPPER  = 272,

  ADDR_PROFILE_ACC_JOINT_1      = 284,
  ADDR_PROFILE_ACC_JOINT_2      = 288,
  ADDR_PROFILE_ACC_JOINT_3      = 292,
  ADDR_PROFILE_ACC_JOINT_4      = 296,
  ADDR_PROFILE_ACC_GRIPPER      = 300,
  ADDR_PROFILE_ACC_WR_JOINT     = 304,
  ADDR_PROFILE_ACC_WR_GRIPPER   = 305,
  ADDR_PROFILE_ACC_RD           = 306,

  ADDR_PROFILE_VEL_JOINT_1      = 308,
  ADDR_PROFILE_VEL_JOINT_2      = 312,
  ADDR_PROFILE_VEL_JOINT_3      = 316,
  ADDR_PROFILE_VEL_JOINT_4      = 320,
  ADDR_PROFILE_VEL_GRIPPER      = 324,
  ADDR_PROFILE_VEL_WR_JOINT     = 328,
  ADDR_PROFILE_VEL_WR_GRIPPER   = 329,
  ADDR_PROFILE_VEL_RD           = 330,

  ADDR_GOAL_CURRENT_JOINT_1     = 332,
  ADDR_GOAL_CURRENT_JOINT_2     = 334,
  ADDR_GOAL_CURRENT_JOINT_3     = 336,
  ADDR_GOAL_CURRENT_JOINT_4     = 338,
  ADDR_GOAL_CURRENT_GRIPPER     = 340,  
  ADDR_GOAL_CURRENT_WR_JOINT    = 342,
  ADDR_GOAL_CURRENT_WR_GRIPPER  = 343,
  ADDR_GOAL_CURRENT_RD          = 344,
  ADDR_MAMUT_GOAL_POS_STEPPER_1       = 345,
  ADDR_MAMUT_GOAL_POS_STEPPER_2       = 347,
  ADDR_MAMUT_GOAL_POS_DYNXL_1         = 349,
  ADDR_MAMUT_GOAL_POS_DYNXL_2         = 351

};

typedef struct ControlItemVariables{
  uint32_t model_inform;

  uint32_t dev_time_millis;
  uint32_t dev_time_micros;

  int8_t device_status;
  uint8_t heart_beat;
  bool debug_mode;
  bool is_connect_ros2_node;
  bool is_connect_motors;
  bool is_connect_manipulator;

  bool user_led[4];
  bool push_button[2];
  bool bumper[2];

  uint16_t illumination;
  uint32_t ir_sensor;
  float sornar;

  uint32_t bat_voltage_x100;
  uint32_t bat_percent_x100;

  uint8_t buzzer_sound;

  bool imu_recalibration;
  float angular_vel[3];
  float linear_acc[3];
  float magnetic[3];
  float orientation[4];

  int32_t present_position[MortorLocation::MOTOR_NUM_MAX];
  int32_t present_velocity[MortorLocation::MOTOR_NUM_MAX];
  int32_t present_current[MortorLocation::MOTOR_NUM_MAX];

  bool motor_torque_enable_state;
  int32_t cmd_vel_linear[3];
  int32_t cmd_vel_angular[3];
  uint32_t profile_acceleration[MortorLocation::MOTOR_NUM_MAX];

  bool joint_torque_enable_state;
  joint_position_info_t joint_goal_position;  
  joint_position_info_t joint_present_position;
  joint_velocity_info_t joint_present_velocity;
  joint_current_info_t joint_present_current;
  joint_accel_info_t joint_profile_acc;
  joint_accel_info_t joint_profile_vel;
  joint_current_info_t joint_goal_current;

  bool joint_goal_position_wr_joint;
  bool joint_goal_position_wr_gripper;
  bool joint_goal_position_rd;

  bool joint_profile_acc_wr_joint;
  bool joint_profile_acc_wr_gripper;
  bool joint_profile_acc_rd;

  bool joint_profile_vel_wr_joint;
  bool joint_profile_vel_wr_gripper;
  bool joint_profile_vel_rd;

  bool joint_goal_current_wr_joint;
  bool joint_goal_current_wr_gripper;
  bool joint_goal_current_rd;
  int16_t stepper_goal_pos_1;
  int16_t stepper_goal_pos_2;
  int16_t dynxl_goal_pos_1;
  int16_t dynxl_goal_pos_2;

}ControlItemVariables;
#endif
