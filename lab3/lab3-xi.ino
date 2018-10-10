#include <sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
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
float total_distance = 0;
float X = 0; 
float T = 0; 

float speedleft = 0 ;
float speedright = 0; 


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
  set_pose_destination(0.1, 0.1, to_radians(45));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Position Controller Functions (Bearing, Heading, Position)
//
//float set_d_err() {
//  sparki.clearLCD();
//  delta_x = dest_pose_x - pose_x;
//  delta_y = dest_pose_y - pose_y;
//  d_err = sqrt(delta_x*delta_x + delta_y*delta_y);  
//  //sparki.print("d_err: ");
//  //sparki.println(d_err);
//   //sparki.updateLCD();
//  delay(1000);
//  return d_err; 
//}
//
//float  set_b_err() {
//  delta_x = dest_pose_x - pose_x;
//  delta_y = dest_pose_y - pose_y;
//  b_err = atan2(delta_y, delta_x) - pose_theta; 
//  return b_err; 
//}
//  
//float set_h_err() {  // MUST CALL "set_b_err" FIRST
//  h_err = dest_pose_theta - pose_theta - b_err; // Subracting b_err in order to account for change in theta while reducing bearing
//  return h_err; 
//}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = sqrt(pow(pose_x - dest_pose_x, 2) + pow(pose_y - dest_pose_y, 2)) ; // TODO
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

void calculateErr(){
  d_err = sqrt(pow(pose_x - dest_pose_x, 2) + pow(pose_y - dest_pose_y, 2));
  b_err = pose_theta - atan2(pose_y - dest_pose_y, pose_x - dest_pose_x);
  h_err = dest_pose_theta - pose_theta;
}

void stopMoving(){

    sparki.moveStop();
    pose_x = 0;
    pose_y = 0;
    pose_theta = 0;  
}

void updateOdometry() {
  pose_theta += ROBOT_SPEED * CYCLE_TIME * (right_wheel_rotating * right_speed_pct - left_wheel_rotating * left_speed_pct) / AXLE_DIAMETER;
  if (pose_theta > M_PI) pose_theta -= 2.0*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.0*M_PI;
  pose_x += ROBOT_SPEED * CYCLE_TIME * cos(pose_theta) * (left_wheel_rotating * left_speed_pct + right_wheel_rotating * right_speed_pct) / 2;
  pose_y += ROBOT_SPEED * CYCLE_TIME * sin(pose_theta) * (left_wheel_rotating * left_speed_pct + right_wheel_rotating * right_speed_pct) / 2;
}


//void reduce_b_err() {
//  if (b_err < 0) {
//    sparki.moveRight(to_degrees(b_err));
//    b_err = 0;
//  }
//  else if (b_err > 0) {
//    sparki.moveRight(to_degrees(b_err));
//    b_err = 0;
//  }
//  //delta_theta = b_err;
//  pose_theta += b_err;
//}
//
//void reduce_d_err(){
//    sparki.moveForward(d_err*100);
//    d_err = 0 ; 
//}
//
//void reduce_h_err(){
//  if (h_err < 0) {
//    sparki.moveRight(to_degrees(h_err));
//    h_err = 0 ; 
//  }else if (h_err > 0 ) {
//    sparki.moveRight(to_degrees(h_err));
//    h_err = 0; 
//  }else {
//    h_err = 0 ;
//    updateOdometry();
//    delay(1000); 
//    current_state = CONTROLLER_DUMMY_STATE;
//  }
//}


void displayOdometry() {
  sparki.clearLCD();
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
  sparki.updateLCD();
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
     
  switch (current_state) {
//    case CONTROLLER_FOLLOW_LINE:
//     
//      
//      // Useful for testing odometry updates
//      readSensors();
//      if (line_center < threshold) {
//        // TODO: Fill in odometry code
//        sparki.moveForward();
//      } else if (line_left < threshold) {
//        // TODO: Fill in odometry code
//        
//        delta_theta = -5 * pi / 180;
//        sparki.moveLeft(5);
//        
//        
//      } else if (line_right < threshold) {
//        // TODO: Fill in odometry code
//        delta_theta = 5 * pi / 180;
//        sparki.moveRight(5);
//      
//      } else {
//        sparki.moveStop();
//      }
//      
//      updateOdometry();
//      
//      // Check for start line, use as loop closure
//      if (line_left < threshold && line_right < threshold && line_center < threshold) {
//        pose_x = 0.;
//        pose_y = 0.;
//        pose_theta = 0.;
//      } 
//      break;
//      
//    case CONTROLLER_GOTO_POSITION_PART2:
//      /*  */
//      sparki.clearLCD();
//      
//      sparki.print("Pos Err: ");
//      sparki.println(set_d_err());
//      sparki.print("Bearing Err: ");
//      sparki.print(set_b_err());
//      sparki.println("Heading Err: ");
//      sparki.print(set_h_err());
//      sparki.println("");
//      
//      sparki.updateLCD();
//      
//      reduce_b_err();
//      reduce_d_err();
//      reduce_h_err();
//
//      delay(10000);
//      break;      
    case CONTROLLER_GOTO_POSITION_PART3:        
      updateOdometry();

      calculateErr();
      if (d_err < 0.01 && h_err < 0.01){
        stopMoving();
      }

      
      dX = 0.01 * d_err;
      dTheta = 0.01 * b_err; // + 0.001 * h_err;

   
      
      phi_l = (2 * dX - dTheta * AXLE_DIAMETER) / (2 * WHEEL_RADIUS);
      phi_r = (2 * dX + dTheta * AXLE_DIAMETER) / (2 * WHEEL_RADIUS);
      // Serial.print("right speed: ");
      // Serial.println(right_speed_pct);
      // Serial.print("left speed: ");
      // Serial.println(left_speed_pct);
            
      left_speed_pct = phi_l / max(phi_l, phi_r);
      right_speed_pct = phi_r / max(phi_l, phi_r); 
      
      
      if (right_speed_pct > 0) {
        right_wheel_rotating = FWD;
        right_dir = DIR_CW;
      }
      else if (right_speed_pct < 0) {
        right_wheel_rotating = BCK;
        right_dir = DIR_CCW;
      }
      else {
        right_wheel_rotating = NONE;
      }

      if (left_speed_pct > 0) {
        left_wheel_rotating = FWD;
        left_dir = DIR_CCW;
      }
      else if (left_speed_pct < 0) {
        left_wheel_rotating = BCK;
        left_dir = DIR_CW;
      }
      else {
        left_wheel_rotating = NONE;
      }
      
//      right_dir = right_wheel_rotating * DIR_CW;
//      left_dir = left_wheel_rotating * DIR_CCW;


      if (left_wheel_rotating != NONE) {
        sparki.motorRotate(MOTOR_LEFT, left_dir, int((left_speed_pct)*100));
      }
      if (right_wheel_rotating != NONE) {
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int((right_speed_pct)*100));
      }

      displayOdometry();
      break;
    case CONTROLLER_DUMMY_STATE:
      
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
