#include <sparki.h>

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define CONTROLLER_CALIBRATE_FOLLOW 3
#define CONTROLLER_DUMMY_STATE 999

// Threshold for detecting if there is a line underneath the IR sensors.
#define IR_THRESHOLD 700

// Change this variable to determine which controller to run
int current_state = CONTROLLER_CALIBRATE_FOLLOW;

// Variables for storing sensor data.
int cm_distance = 1000;
int edge_left = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
int edge_right = 1000;

// Variables for storing state related auxiliary information.
int right_wheel_rotating = 0;
int left_wheel_rotating = 0;

// Time measurement variables, in milliseconds.
unsigned long t_0, t_logic, t, delta_t, t_delay;

// Constants of the environment that we are working in
float m_s = 0; // 0.002784998 or that times ten depending on the scale;
float rad_ms = 0; // Velocity when turning, in radians per millisecond
float pi = 3.14159;
float sparki_wheel_radius = 0; // FIXME
float sparki_wheel_circumference = sparki_wheel_radius * 2 * pi;
float sparki_axle_distance = 0; // FIXME

// Total distance travelled, in centimeters. Also number of laps.
float total_distance = 0;
unsigned lap_counter = 0;

// Current approximate position, taking starting line as a "ground truth".
float pose_x = 0., pose_y = 0., pose_theta = 0.; 

// More delta variables, in floats to avoid lack of precision.
float delta_t_s, delta_x, delta_y, delta_theta, dotx_r;

void setup() {
  // Boring setup code.
  sparki.clearLCD();
  delay(1000); // Wait a bit so we don't rush off the line right away.
  pose_x = 0.; // probably unnecessary
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
  // Recalibrate the angular velocity accordingly.
  float radians_travelled = 30 / sparki_wheel_circumference * 2 * pi;
  sparki.clearLCD();
  t_0 = millis();
  sparki.moveForward(30);
  t = millis();
  delta_t = t - t_0;
  pose_x += 30;
  rad_ms = radians_travelled / delta_t;
  sparki.print("Time elapsed: ");
  sparki.println(delta_t);
  m_s = 30.0 / delta_t * 10; // Recalibrate the velocity while moving forwards.
  // Comment the above out to stick with a static value, if one is provided in definitions.
  sparki.print("Forward velocity: ");
  sparki.println(m_s);
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
  // Update odometry using kinematics based on angular velocity and whatnot
  delta_theta = (right_wheel_rotating - left_wheel_rotating) * rad_ms * sparki_wheel_radius / sparki_axle_distance * delta_t;
  dotx_r = (right_wheel_rotating + left_wheel_rotating) * rad_ms * sparki_wheel_radius / 2;
  delta_x = dotx_r * cos(pose_theta) * delta_t;
  delta_y = dotx_r * sin(pose_theta) * delta_t;
  pose_x += delta_x;
  pose_y += delta_y;
  pose_theta += delta_theta;
  total_distance += sqrt(delta_x * delta_x + delta_y * delta_y);
}

void displayOdometry() {
  // Print all of the major odometry info that we have
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
  // Start our timing, perform all of the necessary setup, display some stuff, figure out logic
  t_0 = millis();

  sparki.clearLCD();

  readSensors();

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Print our odometry, since we need to display that in this case
      updateOdometry();
      displayOdometry();

      // Follow the line, applying course correction when it curves
      if ( allLinesBelowThreshold() ) 
      {
        pose_x = pose_y = pose_theta = 0; // Comment me out to disable loop closure
        lap_counter++; // We've probably found the end of the lap, so increment this
        left_wheel_rotating = right_wheel_rotating = 1;
        sparki.moveForward(); // Ensure that we move off the line rather than just spinning there
      }
      else if ( line_left < IR_THRESHOLD )
      {
        right_wheel_rotating = 1;
        left_wheel_rotating = -1;
        sparki.moveLeft();
      }
      else if ( line_right < IR_THRESHOLD )
      {
        right_wheel_rotating = -1;
        left_wheel_rotating = 1;
        sparki.moveRight();
      }
      // If there's no course correction necessary, just move forward
      else if ( onlyCenterLineBelowThreshold() )
      {
        left_wheel_rotating = right_wheel_rotating = 1;
        sparki.moveForward();
      }
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
    case CONTROLLER_CALIBRATE_FOLLOW:
      measure_30cm_speed();
      current_state = CONTROLLER_FOLLOW_LINE;
      break;
  }

  // Measure the amount of time that the logic of the loop took
  t_logic = millis();

  // Display whatever we printed to the LCD during our state machine logic
  sparki.updateLCD();

  // Try to ensure that each loop takes as close to 100ms as we can manage
  t_delay = (100 - (t_logic - t_0));
  if (t_delay > 0){
    delay(t_delay);
  }

  // Ensure that delta_t timings are right for the odometry
  t = millis();
  delta_t = t - t_0;
}
