#include "final_project.h"

// TODO fill this file with inverse kinematics functions (from lab 6)

void moveForward() {
  left_dir = DIR_CCW;
  right_dir = DIR_CW;
	left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  left_speed_pct = 1.0;
  right_speed_pct = 1.0;
  sparki.moveForward();
}

// FIXME the existing odometry formula (updateOdometry()) from lab 6 is VERY reliant on deterministic cycle time
// we should find a way to fix that so that it doesn't rely on that determinism
// and so that longer cycles can be accomodated without destroying Sparki's accuracy
// hopefully this should be as simple as measuring the time from the start of the previous loop
// to the start of the current loop and dividing by 1000 while casting to float.