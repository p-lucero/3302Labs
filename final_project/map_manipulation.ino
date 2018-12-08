#include "final_project.h"

//**MAP BUILDING**//

/* Call this function to initialize the map */
void map_create(){
  
  byte i; byte j; byte k; byte t; byte w;
  byte curr_type, curr_wall;
  
  for(k = 0; k < NUM_FLOORS; k++){
    t = 0; w = 0;
    for(j = 0; j < NUM_Y_CELLS; j++){
      for(i = 0; i < NUM_X_CELLS; i++){
        // curr_type = pgm_read_byte_near(&types[k * NUM_X_CELLS * NUM_Y_CELLS + t]);
        // curr_wall = pgm_read_byte_near(&walls[k * NUM_X_CELLS * NUM_Y_CELLS + t]);
        cell_make(i, j, k, curr_type, curr_wall);
        t++;
      }
    }
  }
  /* OLD METHOD - WRONG STUFF ANYWAY - KEEP AS REFERENCE
  // Configuring First Floor K=0
  cell_make(0, 0, 0, EXIT, W_S);
  cell_make(0, 1, 0, FREE, W_S);
  cell_make(0, 2, 0, FREE, W_S);
  cell_make(0, 3, 0, FREE, W_NS);
  cell_make(0, 4, 0, FREE, W_NS);
  cell_make(0, 5, 0, FREE, W_SE);
  cell_make(1, 0, 0, FREE, W_W);
  cell_make(1, 1, 0, OBJECT, W_0);
  cell_make(1, 2, 0, ENTRY, W_E);
  cell_make(1, 3, 0, OFFICE, W_SW);
  cell_make(1, 4, 0, ENTRY, W_S);
  cell_make(1, 5, 0, FREE, W_E);
  cell_make(2, 0, 0, FREE, W_W);
  cell_make(2, 1, 0, OBJECT, W_N);
  cell_make(2, 2, 0, OFFICE, W_NE);
  cell_make(2, 3, 0, OFFICE, W_S);
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
  */
}


//**CELL CREATION**//

void cell_settype(byte i, byte j, byte k, byte type){
  world_map[i][j][k] = (type & 0b00001111) | (world_map[i][j][k] & 0b11110000);
}

void cell_setwalls(byte i, byte j, byte k, byte wallbits){
  world_map[i][j][k] = (wallbits & 0b11110000) | (world_map[i][j][k] & 0b00001111);
}

// void cell_batchsettype(byte i, byte j, byte k, byte type, byte num){
//   //TODO: fill in function
// }

void cell_make(byte i, byte j, byte k, byte type, byte wallbits){
  // Set Cell Type
  if(type != -1){ // Pass in -1 as type to leave type as-is
    world_map[i][j][k] = (type & 0b00001111) | (world_map[i][j][k] & 0b11110000);
  }
  // Set Cell Walls
  world_map[i][j][k] = (wallbits & 0b11110000) | (world_map[i][j][k] & 0b00001111);
  
  // Doubling Walls
  if(wallbits & W_N){ // Wall added to the North
    if(short(j)+1 < NUM_Y_CELLS){ // NOT an edge case
      world_map[i][j+1][k] = (W_S | (world_map[i][j+1][k] & 0b11110000)) | (world_map[i][j+1][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_S){ // Wall added to the South
    if(short(j)-1 >= 0){ // NOT an edge case
      world_map[i][j-1][k] = (W_N | (world_map[i][j-1][k] & 0b11110000)) | (world_map[i][j-1][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_E){ // Wall added to the East
    if(short(i)+1 < NUM_X_CELLS){ // NOT an edge case
      world_map[i+1][j][k] = (W_E | (world_map[i+1][j][k] & 0b11110000)) | (world_map[i+1][j][k] & 0b00001111);
    }
    else{
      return;
    }
  }
  if(wallbits & W_W){ // Wall added to the West
    if(short(i)-1 >= 0){ // NOT an edge case
      world_map[i-1][j][k] = (W_W | (world_map[i-1][j][k] & 0b11110000)) | (world_map[i-1][j][k] & 0b00001111);
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

//**CELL byteERACTION**//

byte cell_gettype(byte i, byte j, byte k){
  byte type = world_map[i][j][k] & 0b00001111;
  return type;
}

bool cell_isfree(byte i, byte j, byte k){
  return !(world_map[i][j][k] & LOWER_HALF);
}

bool cell_haswall_N(byte i, byte j, byte k){
  return (world_map[i][j][k] & W_N);
}

bool cell_haswall_S(byte i, byte j, byte k){
  return (world_map[i][j][k] & W_S);
}

bool cell_haswall_E(byte i, byte j, byte k){
  return (world_map[i][j][k] & W_E);
}

bool cell_haswall_W(byte i, byte j, byte k){
  return (world_map[i][j][k] & W_W);
}


