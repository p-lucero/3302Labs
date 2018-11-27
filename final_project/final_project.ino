// Main code file, containing the outline of the logic for the state machine

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

// Header file so that important defines are accessible from all .ino files
#include "final_project.h"

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
byte goal_i, goal_j, goal_floor;

short* path = NULL;

void setup() {
  // TODO set up the world map here
  // current_state = PATH_PLANNING
  // goal_state = person_to_rescue
}

void displayOdometry() {
  sparki.print("X: "); sparki.print(pose_x); sparki.print(" Xg: "); sparki.println(dest_pose_x);
  sparki.print("Y: "); sparki.print(pose_y); sparki.print(" Yg: "); sparki.println(dest_pose_y); 
  sparki.print("T: "); sparki.print(pose_theta*180./M_PI); sparki.print(" Tg: "); sparki.println(dest_pose_theta*180./M_PI);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
  sparki.print("s: "); sparki.println(current_state);
}

void loop() {
  // Clear the LCD of whatever was present last cycle
  // Set up bookkeeping variables; add whatever you need to this set of declarations
  sparki.clearLCD();
  unsigned long begin_time = millis(), end_time;

  switch (current_state){
    case PATH_PLANNING:

      // Closely mimics behavior of STATE_START from lab6

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

    case PATH_FINDING_NEXT:

      // Closely mimics behavior of STATE_HAS_PATH from lab6

    break;

    case PATH_FOLLOWING:

      // Closely mimics behavior of STATE_SEEKING_POSE from lab6

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
