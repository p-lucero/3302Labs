#include "final_project.h"

bool arr_empty(byte* arr, byte len){
	for (byte i = 0; i < len; i++){
		if (arr[i] >= 0 && arr[i] < BIG_NUMBER){
      return true;
    }
	}
  return false;
}

int get_min_index(short *arr, int len) {
  int min_idx=0;
  for (int i=0;i < len; ++i) {
    if (arr[min_idx] < 0 || (arr[i] < arr[min_idx] && arr[i] >= 0)) {
      min_idx = i;
    }
  }
  if (arr[min_idx] == -1) return -1; 
  return min_idx;
}

bool vertex_index_to_ij_coordinates(byte v_idx, byte *i, byte *j) {
  *i = v_idx % NUM_X_CELLS;
  *j = v_idx / NUM_X_CELLS;
  
  if (*i < 0 || *j < 0 || *i >= NUM_X_CELLS || *j >= NUM_Y_CELLS) return false;
  return true;
}

byte ij_coordinates_to_vertex_index(byte i, byte j) {
  return j*NUM_X_CELLS + i;  
}

// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool ij_coordinates_to_xy_coordinates(byte i, byte j, float *x, float *y) {
  if (i < 0 || j < 0 || i >= NUM_X_CELLS || j >= NUM_Y_CELLS) return false;
  
  *x = (i+0.5)*(MAP_SIZE_X/NUM_X_CELLS);
  *y = (j+0.5)*(MAP_SIZE_Y/NUM_Y_CELLS);
  return true;  
}

// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool xy_coordinates_to_ij_coordinates(float x, float y, byte *i, byte *j) {
  if (x < 0 || y < 0 || x >= MAP_SIZE_X || y >= MAP_SIZE_Y) return false;
  
  *i = byte((x/MAP_SIZE_X) * NUM_X_CELLS);
  *j = byte((y/MAP_SIZE_Y) * NUM_Y_CELLS);

  return true;  
}

byte get_travel_cost(byte vertex_source, byte vertex_dest, byte floor) {
  bool are_neighboring, blocked;
  byte travel_direction;

  byte s_i, s_j, s_type; // Source i,j, type
  byte d_i, d_j, d_type; // Dest i,j, type

  if (vertex_source == vertex_dest)
    return 0; // degenerate case where they are the same vertex; probably unnecessary

  if (!vertex_index_to_ij_coordinates(vertex_source, &s_i, &s_j)) return BIG_NUMBER;
  if (!vertex_index_to_ij_coordinates(vertex_dest, &d_i, &d_j)) return BIG_NUMBER;

  are_neighboring = (abs(s_i - d_i) + abs(s_j - d_j) <= 1); // 4-Connected world

  // Too far to travel
  if (!are_neighboring)
    return BIG_NUMBER;

  // Which way are we going?
  if (d_j - s_j == 1)
    travel_direction = NORTH;
  else if (d_i - s_i == 1)
    travel_direction = EAST;
  else if (d_j - s_j == -1)
    travel_direction = SOUTH;
  else
    travel_direction = WEST;

  // Check if there's a wall in our way at all
  switch(travel_direction){
    case NORTH:
      blocked = cell_haswall_N(s_i, s_j, floor);
      break;
    case EAST:
      blocked = cell_haswall_E(s_i, s_j, floor);
      break;
    case SOUTH:
      blocked = cell_haswall_S(s_i, s_j, floor);
      break;
    case WEST:
      blocked = cell_haswall_W(s_i, s_j, floor);
      break;
  }

  // Impassable because walls
  if (blocked)
    return BIG_NUMBER;

  s_type = cell_gettype(s_i, s_j, floor);
  d_type = cell_gettype(d_i, d_j, floor);

  // Impassable for reasons other than walls
  if (s_type == KNOWN_OBSTACLE || s_type == NEW_OBSTACLE || s_type == FIRE)
    return BIG_NUMBER;
  if (d_type == KNOWN_OBSTACLE || d_type == NEW_OBSTACLE || d_type == FIRE)
    return BIG_NUMBER;

  return 1; // If everything else succeeded, then we can move here
}

short* get_path_from_dijkstra(byte source_id, byte source_floor, byte dest, byte dest_floor){
  
}