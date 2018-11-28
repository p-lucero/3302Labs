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

void moveForward();
void moveStop();
bool is_robot_at_IK_destination_pose();

// IK and odometry variables
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err =  0., h_err = 0., phi_l = 0., phi_r = 0.;
float left_speed_pct = 0., right_speed_pct = 0.;
byte left_dir, right_dir, left_wheel_rotating = NONE, right_wheel_rotating = NONE;
byte pose_floor = 0;
byte dest_i, dest_j, goal_i, goal_j, goal_floor;
unsigned long last_cycle_time;

short* path = NULL;

void setup() {
  map_create();
  sparki.servo(0); // ensure Sparki's looking forwards
  current_state = PATH_PLANNING;
  goal_i = INITIAL_GOAL_I;
  goal_j = INITIAL_GOAL_J;
  goal_floor = INITIAL_GOAL_FLOOR;
}

void loop() {
  // Clear the LCD of whatever was present last cycle
  // Set up bookkeeping variables; add whatever you need to this set of declarations
  sparki.RGB(RGB_OFF);
  sparki.clearLCD();
  unsigned long begin_time = millis(), end_time;
  byte sparki_i, sparki_j, sparki_idx, goal_idx, path_curr, path_next, path_2next, saved_state;
  int ping_dist;

  updateOdometry((begin_time - last_cycle_time) / 1000.0);
  displayOdometry();

  bool sparki_in_grid = xy_coordinates_to_ij_coordinates(pose_x, pose_y, &sparki_i, &sparki_j);
  sparki_idx = ij_coordinates_to_vertex_index(sparki_i, sparki_j);
  goal_idx = ij_coordinates_to_vertex_index(goal_i, goal_j);

  // check if we've found an object during normal operation

  // FIXME these may be unreliable, but I'd really rather not use their ping() implementation
  // may be worth copying it and coding our own that doesn't utilize a 20ms delay between each ping?
  // while also taking the best value. merits testing to see if it's necessary.
  ping_dist = sparki.ping_single(); 
  if (ping_dist != -1 && current_state != FIND_PERSON && current_state != CARRY_PERSON){
    saved_state = current_state;
    current_state = FOUND_OBJECT;
  }

  switch (current_state){
    case PATH_PLANNING:
      if (path != NULL){
        delete path;
      }

      // If the pose_floor and goal_floor do not match, then this function returns a path to the elevator
      path = get_path_from_dijkstra(sparki_idx, pose_floor, goal_idx, goal_floor);
      if (path == NULL)
        current_state = DUMMY_STATE;
      else
        current_state = PATH_FINDING_NEXT;
      break;

    case PATH_FINDING_NEXT:
      for (byte path_iter = 0; path[path_iter] != -1; path_iter++){
        if (path[path_iter] == sparki_idx){
          path_curr = sparki_idx;
          path_next = path[path_iter + 1];
          if (path_next != -1)
            path_2next = path[path_iter + 2];
          break;
        }
      }

      if (path_next == -1){
        if (goal_floor == 0 && goal_idx == EXIT_IDX){
          // TODO check if we have more people to save
          /* 
          open the gripper and drop a person, if any, safely outside
          if (there are more people to save){
            set goal to be the next person to save
            current_state = PATH_PLANNING
          }
          else
            current_state = FINISHED
          */
        }
        else
          current_state = FIND_PERSON;
      }

      else {
        vertex_index_to_ij_coordinates(path_next, &dest_i, &dest_j);
        ij_coordinates_to_xy_coordinates(dest_i, dest_j, &dest_pose_x, &dest_pose_y);
        if (path_2next == -1){
          dest_pose_theta = 0;
        }

        else switch(path_2next - path_next){
          case NUM_X_CELLS: dest_pose_theta = M_PI / 2; break;
          case -NUM_X_CELLS: dest_pose_theta = M_PI / 2; break;
          case 1: dest_pose_theta = 0; break;
          case -1: dest_pose_theta = M_PI; break;
          default: dest_pose_theta = 0; break;
        }
        current_state = PATH_FOLLOWING;
      }

      break;

    case PATH_FOLLOWING:
    {
      bool path_valid = true;
      // if (any objects put on map at start of loop)
        // iterate through path array
        // if (object(s) occupy a square that's part of the path that we're not past)
          path_valid = false;

      if (path_valid){
        compute_IK_errors();
        compute_IK_wheel_rotations();
        if (is_robot_at_IK_destination_pose()){
          moveStop(); // stop for a second and think so that our odometry doesn't get out of whack
          current_state = PATH_FINDING_NEXT; 
        }
        else{
          set_IK_motor_rotations();
        }
      }
      else
        current_state = PATH_PLANNING;
      break;
    }

    case IN_ELEVATOR:
      moveStop();
      /*
       * TODO
       * Use bluetooh remote here
       * Wait(5000) for map change-out
       * potentially also just wait until remote feedback 
       * 
       */

      pose_floor = goal_floor;
      current_state = PATH_PLANNING;
      break;

    case FOUND_OBJECT:
      moveStop();
      /*
       *
       *if(found_object != candle){ 
       * current_state = CARRY_PERSON;
       *}else{
       * update map with candle info
       * if path is blocked by new object
       *   current_state = PATH_PLANNING;
       * else
       *   current_state = saved_state;
       *}
      */
      
    break;

    case FIND_PERSON:
      if (ping_dist != -1){
        moveStop(); // just in case
        current_state = CARRY_PERSON;
      }
      else {
        // spin
        left_dir = DIR_CCW;
        left_wheel_rotating = FWD;
        left_speed_pct = .5;

        right_dir = DIR_CCW;
        right_wheel_rotating = BCK;
        right_speed_pct = .5;

        // FIXME rotate a little bit slower so we don't miss anyone, maybe unnecessary? needs testing
        sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));
      }
      break;

    case CARRY_PERSON:
      moveForward();
      if (ping_dist <= SPARKI_GRAB_DISTANCE && ping_dist != -1){
        moveStop();
        sparki.gripperClose();
        delay(SPARKI_GRIP_TIME);
        sparki.gripperStop();
        goal_i = 0;
        goal_j = 0;
        goal_floor = 0;
        current_state = PATH_PLANNING;
      }
      break;

    case FINISHED:
      // Saved everybody that we meant to save, or encountered some other "completion" condition.
      // Probably does nothing.

      break;
    
    case DUMMY_STATE:
      // Does nothing. We end up here if something goes wrong and we can't do anything else.
      // We could use the controller here to manually transition to another state,
      // f.e. one that clears Sparki's current odometry information and assumes him
      // to be at the building entrance. TODO

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
  last_cycle_time = begin_time;
}
