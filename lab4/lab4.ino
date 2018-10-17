#include <sparki.h>
#include <math.h> //Included for floor()

#define ROBOT_SPEED 0.0278
#define MIN_CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865

#define FWD 1
#define NONE 0
#define BCK -1

// Screen size
#define SCREEN_X_RES 128.
#define SCREEN_Y_RES 64.

// Map size
#define NUM_X_CELLS 6
#define NUM_Y_CELLS 6

// Start line is 18", 2" from bottom left corner
#define START_LINE_X .4572 
#define START_LINE_Y .0508 

#define SERVO_POS_DEG 45

float rx; // robot frame x
float ry; // robot frame y
float wx; // world frame x
float wy; // world frame y
int obj_i; // map space
int obj_j; // map space
float distance; // sensor distance measurement

// int current_state = 1; // unnecessary allocation?
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
unsigned long last_cycle_time = 0;

float pose_x = 0., pose_y = 0., pose_theta = 0., pose_servo = 0.;
int left_wheel_rotating = 0, right_wheel_rotating = 0;

// FINISHED: Define world_map multi-dimensional array
bool world_map[NUM_X_CELLS][NUM_Y_CELLS]; 

// FINISHED: Figure out how many meters of space are in each grid cell
const float CELL_RESOLUTION_X = .6 / NUM_X_CELLS;  // Line following map is ~60cm x ~42cm
const float CELL_RESOLUTION_Y = .42 / NUM_Y_CELLS; // Line following map is ~60cm x ~42cm

void setup() {
  pose_x = START_LINE_X;
  pose_y = START_LINE_Y;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  pose_servo = to_radians(SERVO_POS_DEG);

  sparki.servo(-to_degrees(pose_servo)); // Servo axis points down instead of up, so we need to negate it to be consistent with the robot's rotation axis

  // FINISHED: Initialize world_map
  for (int i = 0; i < NUM_X_CELLS; i++){
    for (int j = 0; j < NUM_Y_CELLS; j++){
      world_map[i][j] = 1; // everything starts out as unoccupied until otherwise determined
    }
  }

  sparki.clearLCD();
  displayMap();
  sparki.updateLCD(); // Improve responsiveness of map printing
  delay(1000);  
  last_cycle_time = millis();
}

float to_radians(float deg) {
  return deg * 3.14159 / 180.;
}

float to_degrees(float rad) {
  return rad * 180. / 3.14159;
}

// Ultrasonic Sensor Readings -> Robot coordinates
//XI -> FUNCTIONS transfprm_us_to_robot_coords & FUNCTION transform_robot_to_world_coords 
void transform_us_to_robot_coords(float dist, float theta, float *rx, float *ry) {
  
  //Finished 
  *rx = dist * cos(pose_servo);
  *ry = dist * sin(pose_servo);
}
//XI -> 
// Robot coordinates -> World frame coordinates
//Formula : A     A          B        A
//           Q  =    R    *    Q    +    P  
//                B
void transform_robot_to_world_coords(float x, float y, float *gx, float *gy) {
    
    //Finished: Need double check  
    *gx = cos(pose_theta)*x  - sin(pose_theta)*y + pose_x;
    *gy = sin(pose_theta)*x  + cos(pose_theta)*y + pose_y;
  
}
//Ian:
bool transform_xy_to_grid_coords(float x, float y, int *i, int *j) {
  // FINISHED: Return 0 if the X,Y coordinates were out of bounds
  if((x >= .6) || (y >= .42)){
    return 0; // returns 0 if x or y is beyond paper in positive direction
  }
  if((x < 0) || (y < 0)){
    return 0; // returns 0 if x or y is boyond paper in negative direction
  }
  // FINISHED: Set *i and *j to their corresponding grid coords  
  *i = floor(x / CELL_RESOLUTION_X);
  *j = floor(y / CELL_RESOLUTION_Y);
  return 1;
}

//Ian:
// Turns grid coordinates into world coordinates (grid centers)
bool transform_grid_coords_to_xy(int i, int j, float *x, float *y) {
  // FINISHED: Return 0 if the I,J coordinates were out of bounds
  if((i < 0) || (j < 0)){
    return 0;
  }
  if((i >= NUM_X_CELLS) || (j >= NUM_Y_CELLS)){
    return 0;
  }

  // FINISHED: Set *x and *y
  *x = (i * CELL_RESOLUTION_X) + (CELL_RESOLUTION_X / 2);
  *y = (j * CELL_RESOLUTION_Y) + (CELL_RESOLUTION_Y / 2);
  return 1;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = float(sparki.ping()) / 100.;
}

void moveRight() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = BCK;
  sparki.moveRight();
}

void moveLeft() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = FWD;
  sparki.moveLeft();
}

void moveForward() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  sparki.moveForward();
}

void moveBackward() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = BCK;
  sparki.moveBackward();
}

void moveStop() {
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  sparki.moveStop();
}

void updateOdometry(float cycle_time) {
  pose_x += cos(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_y += sin(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_theta += (right_wheel_rotating - left_wheel_rotating) * cycle_time * ROBOT_SPEED / AXLE_DIAMETER;
}

void displayMap() {
  // FINISHED: Measure how many pixels will be taken by each grid cell
  const int PIXELS_PER_X_CELL = int(SCREEN_X_RES/NUM_X_CELLS);
  const int PIXELS_PER_Y_CELL = int(SCREEN_Y_RES/NUM_Y_CELLS); 
  const int MAX_X_COORD = PIXELS_PER_X_CELL * NUM_X_CELLS;
  const int MAX_Y_COORD = PIXELS_PER_Y_CELL * NUM_Y_CELLS;
  int cur_cell_x=-1, cur_cell_y=-1;
  int x0, y0, x1, y1;

  // FINISHED: Make sure that if the robot is "off-grid", e.g., at a negative grid position or somewhere outside your grid's max x or y position that you don't try to plot the robot's position!
  transform_xy_to_grid_coords(pose_x, pose_y, &cur_cell_x, &cur_cell_y);
  
  // FINISHED: Draw Map
  // Quick and dirty but lazy debug printing method
  // for (int i = NUM_X_CELLS - 1; i > 0; i--){
  //   for (int j = 0; j < NUM_Y_CELLS; j++){
  //     if (i == cur_cell_x && j == cur_cell_y){
  //       sparki.print(2);
  //     }
  //     else {
  //       sparki.print(world_map[i][j]); 
  //     }
  //     sparki.print(" ");
  //   }
  //   sparki.println();
  // }
  
  // more robust printing method that may also break
  for (int i = 0; i < NUM_X_CELLS; i++){
    for (int j = 0; j < NUM_Y_CELLS; j++){
      x0 = PIXELS_PER_X_CELL * i;
      y0 = MAX_Y_COORD - PIXELS_PER_Y_CELL * (j + 1);
      x1 = PIXELS_PER_X_CELL * (i + 1);
      y1 = MAX_Y_COORD - PIXELS_PER_Y_CELL * j;
      if (i == cur_cell_x && j == cur_cell_y){
        // print a sparki as a circle
        sparki.drawCircle((x0 + x1)/2, (y0 + y1)/2, min(PIXELS_PER_X_CELL, PIXELS_PER_Y_CELL)/2);
      }
      else if (world_map[i][j] == 1){
        // print an empty space
        sparki.drawRect(x0, y0, x1, y1);
      }
      else if (world_map[i][j] == 0){
        // print a filled space
        sparki.drawRectFilled(x0, y0, x1, y1);
      }
    }
  }
}

// Helper functions for working with Dijkstra's/A* search next lab
int cell_coords_to_id(int i, int j){
  return j * NUM_X_CELLS + i;
}

void cell_id_to_coords(int id, int* i, int* j){
  *i = id % NUM_X_CELLS;
  *j = id / NUM_X_CELLS;
}

// Only useful if working strictly with A*, because Dijkstra's uses no heuristic
int manhattan_distance_heur(int i1, int j1, int i2, int j2){
  return (abs(i1 - i2) + abs(j1 - j2));
}

// Checks if the two cells are adjacent and non-occupied; could be booleanized, but int may be more extensible?
int cost_to_move(int id1, int id2){
  int i1, i2, j1, j2;
  cell_id_to_coords(id1, &i1, &j1);
  cell_id_to_coords(id2, &i2, &j2);
  if (((abs(i1 - i2) == 1 && j1 - j2 == 0) ||
      (abs(j1 - j2) == 1 && i1 - i2 == 0)) &&
     world_map[i1][j1] && world_map[i2][j2]){
    return 1;
  }
  else {
    return 99; 
  }
} 

void serialPrintOdometry() {
  Serial.print("\n\n\nPose: ");
  Serial.print("\nX: ");
  Serial.print(pose_x);
  Serial.print("\nY: ");
  Serial.print(pose_y);
  Serial.print("\nT: ");
  Serial.print(pose_theta * 180. / 3.14159);
  Serial.print("\n");
}

void displayOdometry() {
  sparki.println("Pose: ");
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("T: ");
  sparki.println(pose_theta * 180. / 3.14159);
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long begin_movement_time = 0;
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  float elapsed_time;
  bool found_object = 0;
  readSensors();

  elapsed_time = (millis() - last_cycle_time) / 1000.0;
  updateOdometry(elapsed_time);
  // serialPrintOdometry(); // TODO uncomment this once timing issues are worked out.

  // FINISHED: Check if sensors found an object
  transform_us_to_robot_coords(distance, pose_servo, &rx, &ry); // Use the value calculated in readSensors, rather than reinventing the wheel
  transform_robot_to_world_coords(rx, ry, &wx, &wy);
  if(transform_xy_to_grid_coords(wx, wy, &obj_i, &obj_j)){
    world_map[obj_i][obj_j] = 0; // FIXME is this working?
  }
  
  // Mapping Code
  // Commenting this out prevents servo twitching, but this may be useful later if we want to change pose_servo
  // sparki.servo(-to_degrees(pose_servo));
  sparki.clearLCD();
  displayMap();
  sparki.updateLCD();

  if (line_center < threshold) {
    moveForward();
  } else if (line_left < threshold) {
    moveLeft();
  } else if (line_right < threshold) {
    moveRight();
  } else {
    moveStop();
  }
  last_cycle_time = millis(); // Start timer for last motor command to determine cycle time
  
  // Check for start line, use as loop closure
  // NOTE: Assumes robot is moving counter-clockwise around the map (by setting pose_theta = 0)!
  //       If your robot is moving clockwise, set pose_theta to pi radians (i.e., pointing left).
  if (line_left < threshold && line_right < threshold && line_center < threshold) {
    pose_x = START_LINE_X;
    pose_y = START_LINE_Y;
    pose_theta = 0.;
  } 

  end_time = millis(); // FIXME way too much delay is happening here, and Sparki's movement changes are extremely slow. Why???
  delay_time = end_time - begin_time;

  // some debug code to investigate why timing is so weird
  Serial.print("Time at start is ");
  Serial.print(begin_time);
  Serial.print(", time at end is ");
  Serial.print(end_time);
  Serial.print(", delaying for ");
  Serial.println(delay_time < 1000 * MIN_CYCLE_TIME ? 1000 * MIN_CYCLE_TIME - delay_time : 10);

  // actually perform timing changes
  if (delay_time < 1000*MIN_CYCLE_TIME)
    delay(1000*MIN_CYCLE_TIME - delay_time); // make sure each loop takes at least MIN_CYCLE_TIME ms
  else
    delay(10);
}
