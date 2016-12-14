// JJROBOTS AHR AIR HOCKEY ROBOT EVO PROJECT

// USER CONFIGURATIONS HERE
// ROBOT DIMENSIONS, MAX SPEED, MAX ACCELERATION, CALIBRATION...

//#define DEBUG

// ABSOLUTE MAX ROBOT SPEED AND ACCELERATION
// THIS VALUES DEPENDS ON YOUR ROBOT CONSTRUCTION (MOTORS, MECHANICS...)
// RECOMMENDED VALUES FOR 12V POWER SUPPLY
#define MAX_ACCEL 275           // Maximun motor acceleration in (steps/seg2)/1000. Max recommended value:280
#define MAX_SPEED 32000         // Maximun speed in steps/seg. Max absolute value: 32767!!

#define MIN_ACCEL 100
#define MIN_SPEED 5000
#define SCURVE_LOW_SPEED 2500
#define SCURVE_HIGH_SPEED 28000

// Geometric calibration.
// This depends on the pulley teeth. DEFAULT: 200(steps/rev)*8(microstepping) = 1600 steps/rev. 1600/32teeth*2mm(GT2) = 25 steps/mm
#define X_AXIS_STEPS_PER_UNIT 25    
#define Y_AXIS_STEPS_PER_UNIT 25

// This is the center of the table. All units in milimeters
// This are the dimensions of the official air hockey table Exploter from Buffalo
#define TABLE_LENGTH 895
#define TABLE_WIDTH 435
#define ROBOT_CENTER_X TABLE_WIDTH/2   // Center of robot.
#define ROBOT_CENTER_Y TABLE_LENGTH/2

// Absolute Min and Max robot positions in mm (measured from center of robot pusher)
#define ROBOT_MIN_X 60
#define ROBOT_MIN_Y 58
#define ROBOT_MAX_X TABLE_WIDTH-ROBOT_MIN_X
#define ROBOT_MAX_Y ROBOT_CENTER_Y-80

// PuckSize (puck radio in mm)
#define PUCK_SIZE 22

// Initial robot position in mm
// The robot must be at this position at start time
// Default: Centered in X and minimun position in Y
#define ROBOT_INITIAL_POSITION_X TABLE_WIDTH/2
#define ROBOT_INITIAL_POSITION_Y 40   // Measured from center of the robot pusher to the table border

// Robot defense and attack lines
#define ROBOT_DEFENSE_POSITION_DEFAULT 72
#define ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT 280
#define ROBOT_DEFENSE_POSITION_MIN 60
#define ROBOT_DEFENSE_POSITION_MAX 100
#define ROBOT_DEFENSE_ATTACK_POSITION_MIN 140
#define ROBOT_DEFENSE_ATTACK_POSITION_MAX 360

// CORRECTION FOR VISION SYSTEM LAG
#define VISION_SYSTEM_LAG 60   // in miliseconds

// CORRECTION OF MISSING STEPS ON MOTORS
// Coment this lines if you don´t want to make the corrections
#define CORRECT_MISSING_STEPS 1
#define MISSING_STEPS_MAX_ERROR_X 5
#define MISSING_STEPS_MAX_ERROR_Y 5
#define ROBOT_POSITION_CAMERA_CORRECTION_Y 0 // Correction of the position of the camera because the camera point of view and mark position

// PACKET SIZE (UDP MESSAGE)
#define PACKET_SIZE 12  // UDP PACKET SIZE (without sync mark), total size = PACKET_SIZE+3(sync mark)

// Utils (don´t modify)
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ZERO_SPEED 65535






