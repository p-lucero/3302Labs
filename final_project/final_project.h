#ifndef FINAL_PROJECT_DEFINES
#define FINAL_PROJECT_DEFINES

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
#define SPARKI_GRIP_TIME 2500
#define MAP_SIZE_X .9144 // TODO
#define MAP_SIZE_Y .9144 // TODO
// as a special variable to improve readability; may improve performance as a const too
const short CYCLE_TIME_MS = CYCLE_TIME * 1000;

// Robot digital constants
#define FWD 1
#define NONE 0
#define BCK -1
#define NUM_X_CELLS 6
#define NUM_Y_CELLS 6
#define NUM_FLOORS 2 // FIXME?
#define BIG_NUMBER 255

// Bit-masks for manipulating the world map
#define LOWER_HALF 0xF
#define UPPER_HALF 0xF0
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Cell states in world map
#define FREE 0
#define KNOWN_OBSTACLE 1
#define NEW_OBSTACLE 2
#define TARGET_PERSON 3
#define ENTRYWAY 4
#define OFFICE 5
#define ELEVATOR 6
#define EXIT 7
#define FIRE 9

// Map locations
#define EXIT_IDX 0
#define ELEVATOR_IDX // FIXME; not strictly necessary, but removes the need for find_elevator method
#define INITIAL_GOAL_I 5
#define INITIAL_GOAL_J 5
#define INITIAL_GOAL_FLOOR 0

#endif