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

// Cell types
#define FREE 0
#define OBJECT 1
#define OBSTACLE 2
#define PERSON 3
#define ENTRY 4
#define OFFICE 5
#define ELEVATOR 6
#define EXIT 7
#define FIRE 9


byte map[MAP_SIZE_X][MAP_SIZE_Y][MAP_SIZE_Z]; //3d byte array. Z corresponds to number of floors in building

//**MAP BUILDING**//

/* Call this function to initialize the map */
void map_create(){
  //TODO: Add cell creation commands here  
}


//**CELL CREATION**//

void cell_settype(int i, int j, int k, int type){
  map[i][j][k] = (type & 0b00001111) | (map[i][j][k] & 0b11110000);
}

void cell_setwalls(int i, int j, int k, int wallbits){
  map[i][j][k] = (wallbits & 0b11110000) | (map[i][j][k] & 0b00001111);
}

void cell_batchsettype(int i, int j, int k, int type, int num){
  //TODO: fill in function
}

void cell_make(int i, int j, int k, int type, int wallbits){
  // Set Cell Type
  if(type != -1){ // Pass in -1 as type to leave type as-is
    map[i][j][k] = (type & 0b00001111) | (map[i][j][k] & 0b11110000);
  }
  // Set Cell Walls
  map[i][j][k] = (wallbits & 0b11110000) | (map[i][j][k] & 0b00001111);
  
  // Doubling Walls
  if(wallbits & W_N){ // Wall added to the North
    if(j+1 < NUM_Y_CELLS){ // NOT an edge case
      map[i][j+1][k] = (W_S & 0b11110000) | (map[i][j+1][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_S){ // Wall added to the South
    if(j-1 >= 0){ // NOT an edge case
      map[i][j-1][k] = (W_N & 0b11110000) | (map[i][j-1][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_E){ // Wall added to the East
    if(i+1 < NUM_X_CELLS){ // NOT an edge case
      map[i+1][j][k] = (W_W & 0b11110000) | (map[i+1][j][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_W){ // Wall added to the West
    if(i-1 >= 0){ // NOT an edge case
      map[i-1][j][k] = (W_W & 0b11110000) | (map[i-1][j][k] & 0b00001111);
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
  int type = map[i][j][k] & 0b00001111;
  return type;
}

bool cell_isfree(int i, int j, int k){
  if((map[i]j][k] & 0b00001111) == 0){
    return 1;
  }
  else{
   return 0; 
  }
}

bool cell_haswall_N(int i, int j, int k){
  if(map[i][j][k] & W_N){
    return 1;
  }
  return 0;
}

bool cell_haswall_S(int i, int j, int k){
  if(map[i][j][k] & W_S){
    return 1;
  }
  return 0;
}

bool cell_haswall_E(int i, int j, int k){
  if(map[i][j][k] & W_E){
    return 1;
  }
  return 0;
}

bool cell_haswall_W(int i, int j, int k){
  if(map[i][j][k] & W_W){
    return 1;
  }
  return 0;
}


