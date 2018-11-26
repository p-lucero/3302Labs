// This snippet preserves the include of the sparki header file under Sparkiduino
// while also allowing for easier debugging without a Sparki present
// To make use of this, compile with -DSPARKI_HEADLESS_DEBUGGING in your g++ flags
#ifdef SPARKI_HEADLESS_DEBUGGING
#include "sparki.h"
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#else
#include <sparki.h>
#define M_PI 3.14159
#endif

// State machine defines
#define PATH_PLANNING 0
#define PATH_FOLLOWING 1
#define WRONG_FLOOR 2
#define IN_ELEVATOR 3
#define FOUND_OBJECT 4
#define CARRY_PERSON 5

// Robot physical constants
#define ROBOT_SPEED 0.0278
#define CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865
#define WHEEL_RADIUS 0.03
#define MAP_SIZE_X // TODO
#define MAP_SIZE_Y // TODO
// as a special variable to improve readability; may improve performance as a const too
const short CYCLE_TIME_MS = CYCLE_TIME * 1000;

// Robot digital constants
#define FWD 1
#define NONE 0
#define BCK -1
#define NUM_X_CELLS 6
#define NUM_Y_CELLS 6
#define NUM_FLOORS 2 // FIXME
#define BIG_NUMBER 255

// state machine control variables; prefer byte over int, since it's more efficient
byte current_state;

// World map
byte world_map[NUM_X_CELLS][NUM_Y_CELLS][NUM_FLOORS];

// IK and odometry variables
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err =  0., h_err = 0., phi_l = 0., phi_r = 0.;
float left_speed_pct = 0., right_speed_pct = 0.;
byte left_dir, right_dir, left_wheel_rotating = NONE, right_wheel_rotating = NONE;
byte pose_floor = 0, dest_floor;

void setup() {
  // TODO set up the world map here
  // current_state = PATH_PLANNING
  // goal_state = person_to_rescue
}

void loop() {
  // Clear the LCD of whatever was present last cycle
  // Set up bookkeeping variables; add whatever you need to this set of declarations
  sparki.clearLCD();
  unsigned long begin_time = millis(), end_time;

  switch (current_state){
    case PATH_PLANNING:

    /*
     * 
     * Figure out where next person is
     * figure out quickest path from current to goal (goal being a person) with Djikstra
     * if(next_person.floor() != sparki.floor()){
     *  
     *   current_state = WRONG_FLOOR
     * 
     * }else{
     *   
     *   current_state = PATH_FOLLOWING
     *   
     * }
     */

    break;

    case PATH_FOLLOWING:

      /*
       * 
       * Go from (0,0) to person
       * if(unknown object found){
       * 
       *  Determine whether the object is candle or object
       *  Mark candle on the map
       *  Do we put any unknown objects on map?
       *  current_state = PATH_PLANNING
       *  
       * }
       * 
       */

    break;

    case WRONG_FLOOR:

      /*
       * 
       * Figure out shortest path to elevator
       * Would this just be Djikstra's with a goal of elevator?
       * We might not need a state as well as IN_ELEVATOR state
       * 
       */

    break;

    case IN_ELEVATOR:

      /*
       * 
       * Use bluetooh remote here
       * Wait(5000) for map change-out
       * potentially also just wait until remote feedback
       * 
       */

    break;

    case FOUND_OBJECT:

      /*
       *
       *if(found_object != candle){
       *
       *
       *  Go to object
       *  pickup object
       *  current_state = PATH_PLANNING;
       *       
       *}else{
       *
       * update map with candle info
       * current_state = PATH_PLANNING;
       *
       *}
       *
       *
      */
      
    break;

    case CARRY_PERSON:

    /*
     * 
     * We've already determined the object in front of us is a person
     * Go straight to it
     * Close arms around it
     * Set goal to outside/front door/however we want to do this
     * current_state = PATH_PLANNING
     * 
     */
     
     break;
    
  }

  // Always update the LCD: only call this here to avoid anomalous behavior
  sparki.updateLCD();

  // Check how long to delay, so that we take some time between cycles
  end_time = millis();
  if (end_time - begin_time < CYCLE_TIME_MS)
    delay(CYCLE_TIME_MS - (end_time - begin_time));
  else
    delay(10);
}
