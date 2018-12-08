#ifndef FINAL_PROJECT_DEFINES
#define FINAL_PROJECT_DEFINES

#define DEBUG // Define me to enable debug stuff.

// State machine defines
#define PATH_PLANNING 0
#define PATH_FINDING_NEXT 1
#define PATH_FOLLOWING 2
#define IN_ELEVATOR 3
#define FOUND_OBJECT 4
#define FIND_PERSON 5
#define CARRY_PERSON 6
#define FINISHED 254
#define DUMMY_STATE 255

// Robot physical constants
#define ROBOT_SPEED 0.0278
#define CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865
#define WHEEL_RADIUS 0.03
#define SPARKI_GRAB_DISTANCE 4
#define SPARKI_GRIP_TIME 5000
#define MAP_SIZE_X .9144
#define MAP_SIZE_Y .9144
#define DISTANCE_MARGIN 0.02 // 2cm of tolerance
#define HEADING_MARGIN 0.061 // ~3.5 degrees of tolerance
#define IR_THRESHOLD 700
#define PING_DIST_THRESHOLD 12 // FIXME
// as a special variable to improve readability; may improve performance as a const too
const short CYCLE_TIME_MS = CYCLE_TIME * 1000;
const int CYCLE_TIME_US = CYCLE_TIME * 1000000;

// Robot digital constants
#define FWD 1
#define NONE 0
#define BCK -1
#define NUM_X_CELLS 6
#define NUM_Y_CELLS 6
#define NUM_FLOORS 2
#define BIG_NUMBER 255
#define FLAME_SENSOR 8

// Bit-masks for manipulating the world map
#define LOWER_HALF 0xF
#define UPPER_HALF 0xF0
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Wallbits
#define W_N     0b10000000
#define W_S     0b01000000
#define W_E     0b00100000
#define W_W     0b00010000
#define W_NS    0b11000000
#define W_NE    0b10100000
#define W_NW    0b10010000
#define W_SE    0b01100000
#define W_SW    0b01010000
#define W_EW    0b00110000
#define W_NSE   0b11100000
#define W_NSW   0b11010000
#define W_NEW   0b10110000
#define W_SEW   0b01110000
#define W_NSEW  0b11110000
#define W_0     0b00000000

// Map locations
#define EXIT_IDX 0
#define ELEVATOR_IDX 35
#define INITIAL_GOAL_I 3
#define INITIAL_GOAL_J 1
#define INITIAL_GOAL_FLOOR 0

// Cell types
#define FREE 0
#define OBJECT 1
#define OBSTACLE 2
#define PERSON 3
#define ENTRY 4
#define OFFICE 5
#define ELEVATOR 6
#define EXIT 7
#define FIRE 9

#endif