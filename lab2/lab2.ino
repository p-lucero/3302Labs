#include <sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define CONTROLLER_ROTATION_MEASURE 3
#define CONTROLLER_FULL_CALIBRATION 4
#define CONTROLLER_DUMMY_STATE 999

// Threshold for detecting if there is a line underneath the IR sensors.
#define IR_THRESHOLD 700

int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
// Variables for storing sensor data.
int cm_distance = 1000;
int edge_left = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
int edge_right = 1000;

// Time measurement variables, in milliseconds.
unsigned long t_0, t_mid, t, delta_t, t_delay;

// Velocity when moving forward, in centimeters per millisecond
float cm_ms = 0.002784998;
float rad_ms = 0; // Velocity when turning, in radians per millisecond
float pi = 3.14159;

// Total distance travelled, in milliseconds.
float total_distance = 0;

// Current approximate position, taking starting line as a "ground truth".
float pose_x = 0., pose_y = 0., pose_theta = 0.; 
float delta_t_s, delta_x, delta_y, delta_theta;

void setup() {
  sparki.clearLCD();
  delay(1000); // Wait a bit so we don't rush off the line right away.
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  // Read all sensors, and store in appropriate variables.
  cm_distance = sparki.ping();
  edge_left = sparki.edgeLeft();
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  edge_right = sparki.edgeRight();
}

void measure_30cm_speed() {
  // Clear LCD, time how long it takes to move a fixed distance
  // Recalibrate the forward velocity accordingly.
  sparki.clearLCD();
  t_0 = millis();
  sparki.moveForward(30);
  t = millis();
  delta_t = t - t_0;
  // delta_t_s = delta_t / 1000.0;
  sparki.print("Time elapsed: ");
  sparki.println(delta_t);
  cm_ms = 30.0 / delta_t; // Recalibrate the velocity while moving forwards.
  // Comment the above out to stick with a static value.
  sparki.print("Forward velocity: ");
  sparki.println(cm_ms);
  sparki.updateLCD();
  delay(500);
}

void measure_360deg_rotation() {
  // Clear LCD, time how long it takes to rotate a fixed distance
  // Recalibrate the rotational velocity accordingly
  sparki.clearLCD();
  t_0 = millis();
  sparki.moveRight(360);
  t = millis();
  delta_t = t - t_0;
  delta_t_s = delta_t / 1000.0;
  sparki.print("Time elapsed: ");
  sparki.println(delta_t);
  rad_ms = 2 * pi / delta_t;
  sparki.print("Rotational velocity: ");
  sparki.println(rad_ms);
  sparki.updateLCD();
  delay(500);
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
  if (delta_theta != 0) {
    // If we're turning, multiply the rotational velocity times the time taken
    // times either or -1 for the direction in which we're turning
    pose_theta += rad_ms * delta_theta * delta_t;
    if (pose_theta >= 2 * pi) {
      // Loop the angle around
      pose_theta = 0;
    }
  }
  else {
    // Otherwise, we've moved forward at least a little bit
    // Update the odometry for how much we've moved forwards
    delta_x = cm_ms * cos(pose_theta) * delta_t;
    delta_y = cm_ms * sin(pose_theta) * delta_t;
    pose_x = pose_x + delta_x;
    pose_y = pose_y + delta_y;
    total_distance += sqrt(delta_x * delta_x + delta_y * delta_y);
  }
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("Theta: ");
  sparki.println(pose_theta / pi * 180);
  sparki.print("Distance: ");
  sparki.println(total_distance);
}

void loop() {

  sparki.clearLCD();

  readSensors();

  updateOdometry();
  displayOdometry();

  t_0 = millis();

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Follow the line, applying course correction when it curves
      if ( allLinesBelowThreshold() ) {
        // delta_theta = 0;
        pose_x = pose_y = pose_theta = 0;
      }
      else if ( line_left < IR_THRESHOLD )
      {
        delta_theta = -1;
        sparki.moveLeft();
      }
      else if ( line_right < IR_THRESHOLD )
      {
        delta_theta = 1;
        sparki.moveRight();
      }
      // If there's no course correction necessary, just move forward
      else if ( onlyCenterLineBelowThreshold() )
      {
        delta_theta = 0;
        sparki.moveForward();
      }
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
    case CONTROLLER_ROTATION_MEASURE:
      measure_360deg_rotation();
      break;
    case CONTROLLER_FULL_CALIBRATION:
      measure_30cm_speed();
      measure_360deg_rotation();
      current_state = CONTROLLER_FOLLOW_LINE;
      delta_t = 0; // Avoid change in odometry during the first iteration of the loop
      break;
  }

  // Measure the amount of time that the logic of the loop took
  t_mid = millis();

  sparki.updateLCD();
  // Try to ensure that each loop takes as close to 100ms as we can manage
  t_delay = (100 - (t_mid - t_0));
  if (t_delay > 0){
    delay(t_delay);
  }

  // Ensure that loop timings are right for the odometry
  t = millis();
  delta_t = t - t_0;
}
