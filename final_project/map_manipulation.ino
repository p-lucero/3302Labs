#include "final_project.h"

// TODO fill this file with functions that deal with anything map related
// f.e. getting/setting information about a cell in the map

// Wallbits
#define W_N     0b10000000
#define W_S     0b01000000
#define W_E     0b00100000
#define W_W     0b00010000
#define W_NS    0b11000000
#define W_NE    0b10100000
#define W_NW    0b10010000
#define W_SE    0b01100000
#define W_SW    0b01010000
#define W_EW    0b00110000
#define W_NSE   0b11100000
#define W_NSW   0b11010000
#define W_NEW   0b10110000
#define W_NSEW  0b11110000
#define W_0     0b00000000


//byte map[MAP_SIZE_X][MAP_SIZE_Y][MAP_SIZE_Z]; //3d byte array. Z corresponds to number of floors in building

//**MAP BUILDING**//

/* Call this function to initialize the map */
void map_create(){
  // Configuring First Floor K=0
  cell_make(0, 0, 0, EXIT, W_S);
  cell_make(0, 1, 0, FREE, W_S);
  cell_make(0, 2, 0, FREE, W_S);
  cell_make(0, 3, 0, FREE, W_NS);
  cell_make(0, 4, 0, FREE, W_NS);
  cell_make(0, 5, 0, FREE, W_SE);
  cell_make(1, 0, 0, FREE, W_SE);
  cell_make(1, 1, 0, OBJECT, W_0);
  cell_make(1, 2, 0, ENTRY, W_E);
  cell_make(1, 3, 0, OFFICE, W_SW);
  cell_make(1, 4, 0, ENTRY, W_S);
  cell_make(1, 5, 0, FREE, W_E);
  cell_make(2, 0, 0, FREE, W_S);
  cell_make(2, 1, 0, OBJECT, W_N);
  cell_make(2, 2, 0, OFFICE, W_NE);
  cell_make(2, 3, 0, EXIT, W_S);
  cell_make(2, 4, 0, OFFICE, W_NW);
  cell_make(2, 5, 0, FREE, W_EW);
  cell_make(3, 0, 0, FREE, W_NW);
  cell_make(3, 1, 0, FREE, W_S);
  cell_make(3, 2, 0, FREE, W_NS);
  cell_make(3, 3, 0, FREE, W_NS);
  cell_make(3, 4, 0, FREE, W_S);
  cell_make(3, 5, 0, FREE, W_E);
  cell_make(4, 0, 0, OFFICE, W_SW);
  cell_make(4, 1, 0, ENTRY, W_0);
  cell_make(4, 2, 0, OFFICE, W_S);
  cell_make(4, 3, 0, OFFICE, W_SE);
  cell_make(4, 4, 0, FREE, W_NW);
  cell_make(4, 5, 0, FREE, W_E);
  cell_make(5, 0, 0, OFFICE, W_NW);
  cell_make(5, 1, 0, OFFICE, W_N);
  cell_make(5, 2, 0, OFFICE, W_N);
  cell_make(5, 3, 0, OFFICE, W_NE);
  cell_make(5, 4, 0, OBJECT, W_NSEW);
  cell_make(5, 5, 0, ELEVATOR, W_NE);
  
  // Configuring 2nd Floor K=1
  
}


//**CELL CREATION**//

void cell_settype(int i, int j, int k, int type){
  world_map[i][j][k] = (type & 0b00001111) | (world_map[i][j][k] & 0b11110000);
}

void cell_setwalls(int i, int j, int k, int wallbits){
  world_map[i][j][k] = (wallbits & 0b11110000) | (world_map[i][j][k] & 0b00001111);
}

void cell_batchsettype(int i, int j, int k, int type, int num){
  //TODO: fill in function
}

void cell_make(int i, int j, int k, int type, int wallbits){
  // Set Cell Type
  if(type != -1){ // Pass in -1 as type to leave type as-is
    world_map[i][j][k] = (type & 0b00001111) | (world_map[i][j][k] & 0b11110000);
  }
  // Set Cell Walls
  world_map[i][j][k] = (wallbits & 0b11110000) | (world_map[i][j][k] & 0b00001111);
  
  // Doubling Walls
  if(wallbits & W_N){ // Wall added to the North
    if(j+1 < NUM_Y_CELLS){ // NOT an edge case
      world_map[i][j+1][k] = (W_S & 0b11110000) | (world_map[i][j+1][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_S){ // Wall added to the South
    if(j-1 >= 0){ // NOT an edge case
      world_map[i][j-1][k] = (W_N & 0b11110000) | (world_map[i][j-1][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_E){ // Wall added to the East
    if(i+1 < NUM_X_CELLS){ // NOT an edge case
      world_map[i+1][j][k] = (W_E & 0b11110000) | (world_map[i+1][j][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_W){ // Wall added to the West
    if(i-1 >= 0){ // NOT an edge case
      world_map[i-1][j][k] = (W_W & 0b11110000) | (world_map[i-1][j][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  else{
    // No double wall set - Only possible on edge cases.
    return;
  }
}

//**CELL INTERACTION**//

int cell_gettype(int i, int j, int k){
  int type = world_map[i][j][k] & 0b00001111;
  return type;
}

bool cell_isfree(int i, int j, int k){
  return !(world_map[i][j][k] & LOWER_HALF);
}

bool cell_haswall_N(int i, int j, int k){
  return (world_map[i][j][k] & W_N);
}

bool cell_haswall_S(int i, int j, int k){
  return (world_map[i][j][k] & W_S);
}

bool cell_haswall_E(int i, int j, int k){
  return (world_map[i][j][k] & W_E);
}

bool cell_haswall_W(int i, int j, int k){
  return (world_map[i][j][k] & W_W);
}

