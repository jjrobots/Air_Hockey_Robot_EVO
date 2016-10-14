// JJROBOTS AHR AIR HOCKEY ROBOT EVO PROJECT

#include <stdint.h>

// Variable definitions

String MAC;  // MAC address of Wifi module

// Log and Timer variables
long loop_counter;
long timer_old;
long timer_packet_old;
long timer_value;
long log_counter;
int debug_counter;
uint32_t micros_old;

// kinematic variables
// position, speed and acceleration are in step units
volatile int16_t position_M1;  // This variables are modified inside the Timer interrupts
volatile int16_t position_M2;

int16_t real_position_x;
int16_t real_position_y;

int16_t speed_M1;
int16_t speed_M2;
int8_t dir_M1;     //(dir=1 positive, dir=-1 negative)
int8_t dir_M2;
int16_t target_position_M1;
int16_t target_position_M2;
int16_t target_speed_M1;
int16_t target_speed_M2;
int16_t acceleration_M1;
int16_t acceleration_M2;
int16_t max_speed;
int16_t max_acceleration;

int16_t pos_stop_M1;
int16_t pos_stop_M2;

// Trajectory prediction
// Puck variables
//int puckPixX;
//int puckPixY;
int puckSize;
int puckCoordX;
int puckCoordY;
int puckOldCoordX;
int puckOldCoordY;
int puckSpeedX;
int puckSpeedY;
int puckOldSpeedX;
int puckOldSpeedY;
int puckSpeedXAverage;
int puckSpeedYAverage;


// Robot variables
uint8_t robot_status;   // Robot modes: 0 Init 1 Defense 2 Defense+Atack 3 Atack
//int robotPixX;
//int robotPixY;
int robotCoordX;
int robotCoordY;
int robotCoordSamples = 0;
int robotCoordXAverage = 0;
int robotCoordYAverage = 0;
int robotMissingStepsErrorX;
int robotMissingStepsErrorY;
int defense_position;
int attack_position;
long attack_time;
int attack_pos_x;
int attack_pos_y;
int attack_predict_x;
int attack_predict_y;
uint8_t attack_status;
int8_t predict_status;  // 1 Puck is going to goal
int8_t predict_bounce;  // 1 if puck came from a bounce
int8_t predict_bounce_status;
int predict_x;    // X position at impact (mm)
int predict_y;
int predict_x_old=0;
int predict_y_old=0;
int predict_time;   // time to impact in ms
int predict_x_attack;
int predict_time_attack;
//char tempStr2[80];  // for debug porpouses
boolean testmode = false;

// Camera variables
uint16_t cam_timestamp;

// user defined settings
int16_t user_target_x;
int16_t user_target_y;
int16_t user_target_speed;
int16_t user_target_accel;
int16_t user_max_speed;
int16_t user_max_accel;
int16_t user_robot_defense_position;
int16_t user_robot_defense_attack_position;

// Serial port  variables
char SBuffer[PACKET_SIZE+4];
uint8_t readStatus;
uint8_t readCounter;
uint8_t newPacket;
uint16_t com_pos_x;
uint16_t com_pos_y;
uint16_t com_speed_M1;
uint16_t com_speed_M2;
uint16_t target_x_mm;
uint16_t target_y_mm;

// Some util functions...
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Arduino abs function sometimes fail!
int16_t myAbs(int16_t param)
{
  if (param < 0)
    return -param;
  else
    return param;
}

// Extract sign of a variable
int sign(int val)
{
  if (val < 0)
    return (-1);
  else
    return (1);
}





