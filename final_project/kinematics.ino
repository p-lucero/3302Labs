#include "final_project.h"

// TODO fill this file with inverse kinematics functions (from lab 6)

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void moveForward() {
  left_dir = DIR_CCW;
  right_dir = DIR_CW;
  left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  left_speed_pct = 1.0;
  right_speed_pct = 1.0;
  sparki.moveForward();
}

void moveStop() {
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  left_speed_pct = 0.0;
  right_speed_pct = 0.0;
  sparki.moveStop();
}

void compute_IK_errors() {
  // Distance, Bearing, and Heading error
  d_err = sqrt( (dest_pose_x-pose_x)*(dest_pose_x-pose_x) + (dest_pose_y-pose_y)*(dest_pose_y-pose_y) );
  b_err = atan2( (dest_pose_y - pose_y), (dest_pose_x - pose_x) ) - pose_theta;
  h_err = dest_pose_theta - pose_theta;
  
  if (b_err <= -M_PI) b_err += 2.*M_PI;
  else if (b_err > M_PI) b_err -= 2.*M_PI;
  
  if (h_err <= -M_PI) h_err += 2.*M_PI;
  else if (h_err > M_PI) h_err -= 2.*M_PI;  
}

void compute_IK_wheel_rotations() {
  float dTheta, dX;
  if (d_err > DISTANCE_MARGIN) { // Get reasonably close before considering heading error
    dTheta = b_err;
    dX  = 0.1 * min(M_PI, d_err); // De-prioritize distance error to help avoid paths through unintended grid cells
  } else {
    dTheta = h_err;
    dX = 0.; // Force 0-update for dX since we're supposedly at the goal position anyway
  }
  
  phi_l = (dX  - (dTheta*AXLE_DIAMETER/2.)) / WHEEL_RADIUS;
  phi_r = (dX  + (dTheta*AXLE_DIAMETER/2.)) / WHEEL_RADIUS;
}

void set_IK_motor_rotations() {
  float wheel_rotation_normalizer = max(abs(phi_l), abs(phi_r));
  if (wheel_rotation_normalizer == 0) { moveStop(); return; }
  left_speed_pct = abs(phi_l) / wheel_rotation_normalizer;
  right_speed_pct = abs(phi_r) / wheel_rotation_normalizer;

  // Figure out which direction the wheels need to rotate
  // FIXME should these zeroes be +-to_radians(1) instead? - Paul
  // this seems more logical and probably works better but I'm not sure; they had the above initially, but I don't get why
  if (phi_l < 0) {
    left_dir = DIR_CW;
    left_wheel_rotating = BCK;
  } else if (phi_l > 0) {
    left_dir = DIR_CCW;
    left_wheel_rotating = FWD;
  } else {
    left_speed_pct = 0.;
    left_wheel_rotating = 0;
  }

  if (phi_r < 0) {
    right_dir = DIR_CCW;
    right_wheel_rotating = BCK;
  } else if (phi_r > 0) {
    right_dir = DIR_CW;
    right_wheel_rotating = FWD;
  } else {
    right_speed_pct = 0.;
    right_wheel_rotating = 0;
  }

  sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
  sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));
}

bool is_robot_at_IK_destination_pose() {
  return (d_err <= DISTANCE_MARGIN && abs(h_err) < HEADING_MARGIN);
}

void displayOdometry() {
  sparki.print("X: "); sparki.print(pose_x); sparki.print(" Xg: "); sparki.println(dest_pose_x);
  sparki.print("Y: "); sparki.print(pose_y); sparki.print(" Yg: "); sparki.println(dest_pose_y); 
  sparki.print("T: "); sparki.print(pose_theta*180./M_PI); sparki.print(" Tg: "); sparki.println(dest_pose_theta*180./M_PI);

//  sparki.print("dX : "); sparki.print(dX ); sparki.print("   dT: "); sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
  sparki.print("s: "); sparki.println(current_state);
}

//lab 6 odometry except it takes a parameter based on our cycle timme instead of a constant
void updateOdometry(float cycle_time) {
  pose_x += cos(pose_theta) * cycle_time * ROBOT_SPEED 
         * (float(left_wheel_rotating)*left_speed_pct 
            + float(right_wheel_rotating)*right_speed_pct)/2.;
  pose_y += sin(pose_theta) * cycle_time * ROBOT_SPEED 
         * (float(left_wheel_rotating)*left_speed_pct 
            + float(right_wheel_rotating)*right_speed_pct)/2.;
  pose_theta += (float(right_wheel_rotating)*right_speed_pct 
                 - float(left_wheel_rotating)*left_speed_pct)  
              * cycle_time * ROBOT_SPEED / AXLE_DIAMETER;
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

// unsigned long last_cycle_time = 0;
// int state = 0;

// dummy_loop(){
//   unsigned long begin_time = millis();
//   unsigned long delay_time = 0;
//   updateOdometry();
//   displayOdometry();

//   updateOdometry((begin_time - last_cycle_time) / 1000.0);

//   switch(state):
//     case 0:
      

//       //this would only be in 'carry person' and 'path following'
//       compute_IK_errors();
//       compute_IK_wheel_rotations();
//       if (is_robot_at_IK_destination_pose()){
//         moveStop(); // stop for a second and think so that our odometry doesn't get out of whack
//         current_state = STATE_HAS_PATH; 
//       }
//       else{
//         set_IK_motor_rotations();
//       }

//     break;
  

//   delay_time = millis() - begin_time; // total time that the logic of the loop took

//   // actually perform timing changes
//   if (delay_time < 1000*MIN_CYCLE_TIME)
//     delay(1000*MIN_CYCLE_TIME - delay_time); // make sure each loop takes at least MIN_CYCLE_TIME ms
//   else
//     delay(10);
//   last_cycle_time = begin_time; // Ensure that we use the right value for the next loop, which was the beginning of this loop
  
// }


// FIXME the existing odometry formula (updateOdometry()) from lab 6 is VERY reliant on deterministic cycle time
// we should find a way to fix that so that it doesn't rely on that determinism
// and so that longer cycles can be accomodated without destroying Sparki's accuracy
// hopefully this should be as simple as measuring the time from the start of the previous loop
// to the start of the current loop and dividing by 1000 while casting to float.
