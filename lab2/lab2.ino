#include <sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define CONTROLLER_DUMMY_STATE 999

#define IR_THRESHOLD 700


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int cm_distance = 1000;
int edge_left = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
int edge_right = 1000;
unsigned long t_0, t, delta_t, delta_t_s, t_delay;
float cm_ms = 0.002784998;
float pi = 3.14159;

float total_distance = 0;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
float delta_x, delta_y, delta_theta;

void setup() {
  sparki.clearLCD();
  //  sparki.RGB(RGB_RED);
  delay(1000);
  //  sparki.RGB(RGB_GREEN);
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  cm_distance = sparki.ping();
  edge_left = sparki.edgeLeft();
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  edge_right = sparki.edgeRight();
}

void measure_30cm_speed() {
  t_0 = millis();
  Serial.println(t_0);
  sparki.moveForward(30);
  t = millis();
  delta_t = t - t_0;
  delta_t_s = delta_t / 1000;
  Serial.println(delta_t);
  sparki.print("Time elapsed: ");
  sparki.println(delta_t);
  sparki.print("Velocity: ");
  sparki.println(30.0 / delta_t);
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
    pose_theta = pose_theta + delta_theta;
    if (pose_theta >= 2 * pi) {
      pose_theta = 0;
    }
  }
  else {
    delta_x = cm_ms * cos(pose_theta) * 100;
    delta_y = cm_ms * sin(pose_theta) * 100;
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

  // TODO: Insert loop timing/initialization code here

  sparki.clearLCD();

  readSensors();

  t_0 = millis();

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Follow the line, applying course correction when it curves
      if ( allLinesBelowThreshold() ) {
        delta_theta = 0;
        pose_x = pose_y = pose_theta = 0;
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
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }

  t = millis();
  delta_t = t - t_0;

  updateOdometry();
  displayOdometry();

  sparki.updateLCD();
  delay(1000 * CYCLE_TIME);
}

