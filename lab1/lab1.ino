#include <Sparki.h>

// State definitions
#define STATE_SEARCH_OBJ 0
#define STATE_FOUND_OBJ 1
#define STATE_GRAB 2
#define STATE_TURN_AROUND 3
#define STATE_FIND_LINE 4
#define STATE_CENTER_LINE 5
#define STATE_FOLLOW_LINE 6
#define STATE_FOUND_START 7
#define STATE_COMPLETE 8

// Thresholds for sensor input validation
#define FOUND_THRESHOLD 30
#define GRAB_THRESHOLD 4
#define IR_THRESHOLD 700

// Set up some global variables with default values to be replaced during operation
int current_state = STATE_SEARCH_OBJ;
int cm_distance = 1000;
int edge_left = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
int edge_right = 1000;
int delay_ms = 100; // Variable delay MS to use; defaults to 100 to run at 10 Hz


void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance = sparki.ping();
  edge_left = sparki.edgeLeft();
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  edge_right = sparki.edgeRight();
}

// Helper navigational functions
bool onlyCenterBelowThreshold(){
  // Check if we're centered on a line, i.e. only center is reading below threshold
  return (line_center < IR_THRESHOLD) && (line_right > IR_THRESHOLD) && (line_left > IR_THRESHOLD) && (edge_left > IR_THRESHOLD) && (edge_right > IR_THRESHOLD);
}

bool onlyCenterLineBelowThreshold(){
  // Check if we're centered on a line and can move forwards
  return (line_center < IR_THRESHOLD) && (line_left > IR_THRESHOLD) && (line_right > IR_THRESHOLD);
}

bool allLinesBelowThreshold(){
  // Bad performance usually results if reading edge. If the destination line is longer, may be wise to uncomment the below
  return (line_center < IR_THRESHOLD) && (line_left < IR_THRESHOLD) && (line_right < IR_THRESHOLD); // && (edge_left < IR_THRESHOLD) && (edge_right < IR_THRESHOLD)
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  // Print debug information about what's going on; current state, ultrasonic sensor data
  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);
  sparki.print("DISTANCE: ");
  sparki.println(cm_distance);

  // Print debug information about infrared sensor data; TODO tidy up to use a for loop
  sparki.print("INFRAREDS: ");
  sparki.print(edge_left);
  sparki.print(" ");
  sparki.print(line_left);
  sparki.print(" ");
  sparki.print(line_center);
  sparki.print(" ");
  sparki.print(line_right);
  sparki.print(" ");
  sparki.println(edge_right);

  // Your state machine code goes here
  switch(current_state){
    case STATE_SEARCH_OBJ:
      // We're still looking for the object; spin left until we see it
      sparki.moveLeft();
      if (cm_distance < FOUND_THRESHOLD) 
      {
        current_state = STATE_FOUND_OBJ;
      }
      break;
    case STATE_FOUND_OBJ:
      // We've found something to move forwards to; get close to it so we can grab
      delay_ms = 10; // Make the movement more responsive; TODO tune me! 1 ms?
      sparki.moveForward(1);
      if (cm_distance <= GRAB_THRESHOLD) 
      {
        current_state = STATE_GRAB;
      }
      break;
    case STATE_GRAB:
      // We're probably close enough to grab now
      delay_ms = 100; // No need to run super-fast now, change back to 10 Hz
      sparki.gripperClose(); // Close the gripper, wait for it to close
      delay(5000);
      sparki.gripperStop();
      current_state = STATE_TURN_AROUND;
      break;
    case STATE_TURN_AROUND:
      // Turn around...
      sparki.moveRight(180);
      current_state = STATE_FIND_LINE;
      break;
    case STATE_FIND_LINE:
      //... and find a line to follow by walking forwards until found
      if (line_center < IR_THRESHOLD) 
      {
        current_state = STATE_CENTER_LINE;
      }
      else 
      {
        sparki.moveForward();
      }
      break;
    case STATE_CENTER_LINE:
      // Center ourselves on the line; if only the center is reading a line, we're probably good
      if ( onlyCenterBelowThreshold() )
      {
        current_state = STATE_FOLLOW_LINE;
      }
      else 
      {
        // Spin until centered well
        sparki.moveRight(10);
      }
      break;
    case STATE_FOLLOW_LINE:
      // Follow the line, applying course correction when it curves
      if ( line_left < IR_THRESHOLD )
      {  
        sparki.moveLeft(5);
      }
      else if ( line_right < IR_THRESHOLD )
      {  
        sparki.moveRight(5);
      }
      // If there's no course correction necessary, just move forward
      else if ( onlyCenterLineBelowThreshold() )
      {
        sparki.moveForward();
      }
      // If this check passes, we're at our destination
      else if ( allLinesBelowThreshold() )
      {
        current_state = STATE_FOUND_START;
      }
      break;
    case STATE_FOUND_START:
      // Beep, release the object, and move backward a little bit
      sparki.beep();
      sparki.gripperOpen();
      delay(5000);
      sparki.gripperStop();
      sparki.moveBackward(5);
      current_state = STATE_COMPLETE;
      break;
    case STATE_COMPLETE:
      // Do nothing; we're done
      break;
  }

  // Update the LCD and wait a bit so we don't overload the robot
  sparki.updateLCD();
  delay(delay_ms);
}
