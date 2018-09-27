#include <sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3
#define IR_THRESHOLD 700

#define FWD 1
#define NONE 0
#define BCK -1

const float pi = 3.14159;



// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
float delta_x = 0;
float delta_y = 0;
float delta_theta = 0;
float total_distance = 0;


// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART2;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.; // Destination Pose Theta is in radians, as set by the set_pose_destination() function
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (deg), heading error (deg)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;
unsigned long delay_time = 0;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15,0.05, to_radians(90));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Position Controller Functions (Bearing, Heading, Position)

float set_position_err() { // Sets the global p_err to distance from destination and returns value in meters
  int dx = dest_pose_x - pose_x;
  int dy = dest_pose_y - pose_y;
  d_err = sqrt(dx*dx + dy*dy);
  return d_err;
}
float set_bearing_err() { // Sets the global b_err angle between current theta and direction of destination and returns value in degrees
  delta_x = dest_pose_x - pose_x;
  delta_y = dest_pose_y - pose_y;
  b_err = to_degrees(atan2(delta_y, delta_x)); // atan2() returns radians, b_err is set equal to return of to_degrees() in degrees
  return b_err; 
}
  
float set_heading_err() { // Sets the global h_err angle between current theta and target theta at destination and returns value in degrees
  h_err = to_degrees(dest_pose_theta) - pose_theta; // dest_pose_theta is stored in radians, converting to degrees before arithmetic
  return h_err;
}
// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}



bool onlyCenterLineBelowThreshold() {
  // Check if we're centered on a line and can move forwards
  return (line_center < IR_THRESHOLD) && (line_left > IR_THRESHOLD) && (line_right > IR_THRESHOLD);
}

bool allLinesBelowThreshold() {
  // Bad performance usually results if reading edge. If the destination line is longer, may be wise to uncomment the below
  return (line_center < IR_THRESHOLD) && (line_left < IR_THRESHOLD) && (line_right < IR_THRESHOLD); // && (edge_left < IR_THRESHOLD) && (edge_right < IR_THRESHOLD)
}


void updateOdometry() {
  // TODO: Update pose_x, pose_y, pose_theta
  // Bound theta
  
  // from last lab
    
    
    if (delta_theta != 0) {
    pose_theta = pose_theta + delta_theta;
    pose_theta = 360 % int(pose_theta);
    if (pose_theta >= 2 * pi) {
      pose_theta = 0;
    }
  }
  else {
    delta_x = (ROBOT_SPEED*delay_time) * cos(pose_theta)*10;
    delta_y = (ROBOT_SPEED*delay_time) * sin(pose_theta)*10;
    pose_x = pose_x + delta_x;
    pose_y = pose_y + delta_y;
    total_distance += sqrt(delta_x * delta_x + delta_y * delta_y);
  }
  
  
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(delta_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(delta_y); 
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(delta_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      /*
      //  From last lab
        
        
       if ( allLinesBelowThreshold() ) {
        delta_theta = 0;
        pose_x = pose_y = pose_theta = 0;
        //current_state = CONTROLLER_FINISH_LINE;
      }
      else if ( line_left < IR_THRESHOLD )
      {
        delta_theta = -5 * pi / 180;
        sparki.moveLeft(5);
      }
      else if ( line_right < IR_THRESHOLD )
      {
        delta_theta = 5 * pi / 180;
        sparki.moveRight(5);
      }
      // If there's no course correction necessary, just move forward
      else if ( onlyCenterLineBelowThreshold() )
      {
        delta_theta = 0;
        sparki.moveForward();
      }
      break;
      
      */
      
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        
        delta_theta = -5 * pi / 180;
        sparki.moveLeft(5);
        
        
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        delta_theta = 5 * pi / 180;
        sparki.moveRight(5);
      
      } else {
        sparki.moveStop();
      }
      
      updateOdometry();
      
      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      
      sparki.clearLCD();
      
      sparki.print("Pos Err: ");
      sparki.println(set_position_err());
      sparki.print("Bearing Err: ");
      sparki.print(set_bearing_err());
      sparki.println("Heading Err: ");
      sparki.print(set_heading_err());
      sparki.println("");
      
      sparki.updateLCD();
      

      delay(10000);
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:      
      updateOdometry();
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));

      break;
  }

  //sparki.clearLCD();
  //displayOdometry();
  //sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
