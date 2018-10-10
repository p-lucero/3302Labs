#include <sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
//#define CYCLE_TIME .05 //Default 50ms cycle time
#define CYCLE_TIME .10 // CHANGED!!!! 100ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define WHEEL_SPEED 1.86 // radians/second; FIXME very approximate from not well known values!!!
#define CONTROLLER_DUMMY_STATE 0
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

float delta_p, delta_h, delta_b;
// float total_distance = 0;


// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
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
float total_distance = 0.;
float total_theta = 0.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

float bound_angle(float rad) {
  // Only check the angle bound once, hoping that it's not outrageously bad.
  if (rad > M_PI) rad -= 2*M_PI;
  if (rad < -M_PI) rad += 2*M_PI;
  return rad;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  
  
  // Set test cases here!
  set_pose_destination(.15,.5, to_radians(45));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
  
  total_distance = set_d_err();
  total_theta = set_b_err();
  dX = ROBOT_SPEED*(.001); //ROBOT SPEED * TIME OF 1 LOOP = DISTANCE OF 1 LOOP
  dTheta = total_theta*(.001);
  
}

// Position Controller Functions (Bearing, Heading, Position)

float set_d_err() {
  delta_x = dest_pose_x - pose_x;
  delta_y = dest_pose_y - pose_y;
  d_err = sqrt(delta_x*delta_x + delta_y*delta_y);  
  return d_err; 
}

float  set_b_err() {
  delta_x = dest_pose_x - pose_x;
  delta_y = dest_pose_y - pose_y;
  b_err = atan2(delta_y, delta_x) - pose_theta; 
  return b_err; 
}
  
float set_h_err() {  // MUST CALL "set_b_err" FIRST
  h_err = dest_pose_theta - pose_theta - b_err; // Subracting b_err in order to account for change in theta while reducing bearing
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
  float dotx_r, delta_theta, delta_phi_l, delta_phi_r;
  // TODO: Update pose_x, pose_y, pose_theta
  // Bound theta
  
  // from last lab
  
  //CHECK TO MAKE SURE THIS WORKS
  
    delta_phi_l = left_wheel_rotating * WHEEL_SPEED * left_speed_pct / 1000 * 100;
    delta_phi_r = right_wheel_rotating * WHEEL_SPEED * right_speed_pct / 1000 * 100;
    delta_theta = (delta_phi_r - delta_phi_l) * WHEEL_RADIUS / AXLE_DIAMETER; 
    dotx_r = (delta_phi_r + delta_phi_l) * WHEEL_RADIUS / 2;
    delta_x = dotx_r * cos(pose_theta);
    delta_y = dotx_r * sin(pose_theta);
    phi_l += delta_phi_l;
    phi_r += delta_phi_r;
    pose_x += delta_x;
    pose_y += delta_y;
    pose_theta += delta_theta;
    total_distance += sqrt(delta_x * delta_x + delta_y * delta_y);
    // Bound theta
    pose_theta = bound_angle(pose_theta);
    phi_l = bound_angle(phi_l); // may not need to bound these?
    phi_r = bound_angle(phi_r);
    
//     pose_theta = pose_theta + delta_theta;
//     delta_theta = 0;
//     //pose_theta = pose_theta % (2*pi);
//     if (pose_theta >= 2 * pi) { pose_theta = 0; }
  
  
//     delta_x = (dX) * cos(pose_theta);
//     delta_y = (dX) * sin(pose_theta);
//     pose_x = pose_x + delta_x;
//     pose_y = pose_y + delta_y;
//     delta_x = 0;
//     delta_y = 0;
//     total_distance += sqrt(delta_x * delta_x + delta_y * delta_y);
  
//    if (pose_theta > M_PI) pose_theta -= 2.0*M_PI;
//    if (pose_theta <= -M_PI) pose_theta += 2.0*M_PI;
  
    set_d_err();
    set_h_err();
    set_b_err();
}

void reduce_b_err() {
  if (b_err < 0) {
    sparki.moveRight(to_degrees(b_err));
    b_err = 0;
  }
  else if (b_err > 0) {
    sparki.moveRight(to_degrees(b_err));
    b_err = 0;
  }
  delta_theta = b_err;
}

void reduce_d_err(){
    sparki.moveForward(d_err);
    d_err = 0; 
    delta_x = d_err;
}

void reduce_h_err(){
  if (h_err < 0) {
    sparki.moveRight(to_degrees(h_err));
    h_err = 0 ; 
  }else if (h_err > 0 ) {
    sparki.moveRight(to_degrees(h_err));
    h_err = 0; 
  }else {
    h_err = 0 ;
    current_state = CONTROLLER_DUMMY_STATE;
  }
  delta_theta = h_err;
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
      /*  */
      sparki.clearLCD();
      
      sparki.print("Pos Err: ");
      sparki.println(set_d_err());
      sparki.print("Bearing Err: ");
      sparki.print(set_b_err());
      sparki.println("Heading Err: ");
      sparki.print(set_h_err());
      sparki.println("");
      
      sparki.updateLCD();
      
      reduce_b_err();
      reduce_d_err();
      reduce_h_err();

      delay(10000);
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:        
      updateOdometry();
      // Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));

      // FIXME EXTREMELY EXPERIMENTAL.

      // dX = distance_gain * d_err;
      // dTheta = theta_gain * (h_err + b_err);
      delta_p = dX;
      delta_b = 0.01 * b_err;
      delta_h = 0.001
      delta_theta = 0.01 * b_err + 0.001 * h_err;

      right_speed_pct = (2 * dX - delta_theta * AXLE_DIAMETER) / 2 * WHEEL_RADIUS;
      left_speed_pct = (2 * dX + delta_theta * AXLE_DIAMETER) / 2 * WHEEL_RADIUS;
      // Serial.print("right speed: ");
      // Serial.println(right_speed_pct);
      // Serial.print("left speed: ");
      // Serial.println(left_speed_pct);
      
      if (right_speed_pct > 0){
        right_wheel_rotating = FWD;
      }
      else if (right_speed_pct < 0){
        right_wheel_rotating = BCK;
      }
      else {
        right_wheel_rotating = NONE;
      }

      if (left_speed_pct > 0){
        left_wheel_rotating = FWD;
      }
      else if (left_speed_pct < 0){
        left_wheel_rotating = BCK;
      }
      else {
        left_wheel_rotating = NONE;
      }
      
      right_dir = right_wheel_rotating * DIR_CW;
      left_dir = left_wheel_rotating * DIR_CCW;

      if(left_speed_pct > right_speed_pct){
        sparki.motorRotate(MOTOR_LEFT, left_dir, int((left_speed_pct/left_speed_pct)*100));
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int((right_speed_pct/left_speed_pct)*100));
      }else if(left_speed_pct < right_speed_pct){
        sparki.motorRotate(MOTOR_LEFT, left_dir, int((left_speed_pct/right_speed_pct)*100));
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int((right_speed_pct/right_speed_pct)*100));
      }else{
        sparki.motorRotate(MOTOR_LEFT, left_dir, int(100));
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int(100));
      }
      
      
      
      if((d_err < 2) && (b_err < 5)){
        current_state = CONTROLLER_DUMMY_STATE;
        break;
      }
      
       
    
      //sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
      
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));

      break;
    case CONTROLLER_DUMMY_STATE:
      sparki.motorStop(MOTOR_LEFT);
      sparki.motorStop(MOTOR_RIGHT);
      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();
  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}


