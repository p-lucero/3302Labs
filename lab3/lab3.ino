#include <sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define WHEEL_SPEED 1.86 // radians/second
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART2;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)
float total_distance = 0;
unsigned long delta_t = 0;


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_centimeters(double meters) {
  return meters * 100.;
}

float to_meters(double centimeters) {
  return centimeters / 100.;
}

float to_radians(double deg) {
  return deg * M_PI / 180.;
}

float to_degrees(double rad) {
  return rad * 180 / M_PI;
}

float to_seconds(unsigned ms) {
  return ms / 1000.;
}

float to_milliseconds(float s) {
  return s * 1000.;
}

float bound_angle(float rad) {
  // Only check the angle bound once, hoping that it's not outrageously bad.
  if (rad > M_PI) rad -= 2*M_PI;
  if (rad < M_PI) rad += 2*M_PI;
  return rad;
}

float hard_bound_angle(float rad) {
  // For *really* ensuring that the angle is in (-pi, pi).
  while (rad > M_PI) rad -= 2*M_PI;
  while (rad < M_PI) rad += 2*M_PI;
  return rad;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15,0.05, to_radians(135));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  float dx, dy;
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = hard_bound_angle(t);
  orig_dist_to_goal = set_position_err(); // FIXME?
}

float set_position_err() {
  float dx = dest_pose_x - pose_x;
  float dy = dest_pose_y - pose_y;
  d_err = sqrt(dx*dx + dy*dy);
  return d_err;
}

float set_bearing_err() {
  float dx = dest_pose_x - pose_x;
  float dy = dest_pose_y - pose_y;
  b_err = atan2(dy, dx) - pose_theta;
  return b_err;
}

float set_heading_err() {
  h_err = dest_pose_theta - pose_theta;
  return h_err;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry() {
  float delta_x, delta_y, delta_theta, delta_phi_l, delta_phi_r;
  // FIXME?: Update pose_x, pose_y, pose_theta
  for (int i = 0; i < delta_t; i++) {
    // For loop method should work a lot better around curves?
    delta_phi_l = WHEEL_SPEED * left_speed_pct / 1000;
    delta_phi_r = WHEEL_SPEED * right_speed_pct / 1000;
    delta_theta = (right_wheel_rotating * delta_phi_r - left_wheel_rotating * delta_phi_l)
                  * WHEEL_RADIUS / AXLE_DIAMETER; 
    dotx_r = (right_wheel_rotating * delta_phi_r + left_wheel_rotating * delta_phi_l)
                  * WHEEL_RADIUS / 2;
    delta_x = dotx_r * cos(pose_theta);
    delta_y = dotx_r * sin(pose_theta);
    phi_l += delta_phi_l;
    phi_r += delta_phi_r;
    pose_x += delta_x;
    pose_y += delta_y;
    pose_theta += delta_theta;
    // Bound theta
    pose_theta = bound_angle(pose_theta);
    phi_l = bound_angle(phi_l); // may not need to bound these?
    phi_r = bound_angle(phi_r);
  }

  // Update our current estimates of the error in our position, etc.
  set_position_err();
  set_heading_err();
  set_bearing_err();
}

void displayOdometry() {
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
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        right_wheel_rotating = left_wheel_rotating = FWD;
        left_speed_pct = right_speed_pct = 1;
        sparki.moveForward();
      } else if (line_left < threshold) {
        right_wheel_rotating = FWD;
        left_wheel_rotating = BCK;
        left_speed_pct = right_speed_pct = 1;
        sparki.moveLeft();
      } else if (line_right < threshold) {
        right_wheel_rotating = BCK;
        left_wheel_rotating = FWD;
        left_speed_pct = right_speed_pct = 1;
        sparki.moveRight();
      } else {
        right_wheel_rotating = left_wheel_rotating = NONE;
        left_speed_pct = right_speed_pct = 0; // probably extraneous
        sparki.moveStop(); // stop if we reach something that's not line
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = pose_y = pose_theta = 0.;
      } 

      // Make sure to update the odometry so we can see what's happening
      updateOdometry();
      break;   
    case CONTROLLER_GOTO_POSITION_PART2:
      // Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination

      // Reduce bearing error to 0
      if (b_err > 0) {
        sparki.moveLeft(to_degrees(b_err));
      }
      else {
        sparki.moveRight(-to_degrees(b_err));
      }

      // Reduce position error to 0
      sparki.moveForward(to_centimeters(d_err));

      // Reduce heading error to 0
      if (h_err > 0){
        sparki.moveLeft(to_degrees(b_err));
      }
      else {
        sparki.moveRight(-to_degrees(b_err));
      }
      
      break;   
    case CONTROLLER_GOTO_POSITION_PART3:      
      updateOdometry();
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));

      // TODO add calculations here of what the below four values should be.

      right_wheel_rotating = NONE; // change this and the below 3 lines based on calculations of what trajectory to take
      left_wheel_rotating = NONE;
      right_speed_pct = 0; // value between 0 and 1
      left_speed_pct = 0;

      // Perform invariant updates, apply all values to the two motors
      right_dir = right_wheel_rotating;
      left_dir = -left_wheel_rotating;
      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));
      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
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
  delta_t = start_time - millis();
}
