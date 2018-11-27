#include "final_project.h"

bool arr_empty(short* arr, byte len){
	for (byte i = 0; i < len; i++){
		if (arr[i] >= 0 && arr[i] < BIG_NUMBER){
      return false;
    }
	}
  return true;
}

int get_min_index(short *arr, byte len) {
  int min_idx=0;
  for (byte i=0;i < len; ++i) {
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

  return 1; // If everything else succeeded, then we can move between these two
}

byte find_elevator(byte floor){
  for (byte i = 0; i < NUM_X_CELLS; i++){
    for (byte j = 0; j < NUM_Y_CELLS; j++){
      if (cell_gettype(i, j, floor) == ELEVATOR)
        return ij_coordinates_to_vertex_index(i, j);
    }
  }
  return BIG_NUMBER; // wow I really want this to never happen
}

short* get_path_from_dijkstra(byte source_id, byte source_floor, byte dest_id, byte dest_floor){
  if (source_floor != dest_floor){
    dest_id = find_elevator(source_floor);
    if (dest_id == BIG_NUMBER){
      Serial.print("Unable to find an elevator on floor "); Serial.println(source_floor);
      Serial.println("Something went very wrong with the mapping in setup()!");
      return NULL;
    }
    dest_floor = source_floor;
  }
  int cur_vertex = -1;
  int cur_cost = -1;
  int min_index = -1;
  int travel_cost = -1;
  byte dist[NUM_X_CELLS*NUM_Y_CELLS];
  short prev[NUM_X_CELLS*NUM_Y_CELLS];
  short Q_cost[NUM_Y_CELLS*NUM_X_CELLS];

  // Initialize our variables
  for (int i = 0; i < NUM_X_CELLS * NUM_Y_CELLS; ++i) {
    Q_cost[i] = -1;
    dist[i] = BIG_NUMBER;
    prev[i] = -1;
  }

  dist[source_id] = 0;
  Q_cost[source_id] = 0;
  while (!arr_empty(Q_cost,NUM_X_CELLS*NUM_Y_CELLS)) {
    min_index = get_min_index(Q_cost, NUM_X_CELLS*NUM_Y_CELLS);
    if (min_index < 0) {
      break; // Queue is empty somehow!
    }
    cur_vertex = min_index; // Vertex ID is same as array indices
    cur_cost = Q_cost[cur_vertex]; // Current best cost for reaching vertex
    Q_cost[cur_vertex] = -1; // Remove cur_vertex from the queue.

    // Iterate through all of node's neighbors and see if cur_vertex provides a shorter
    // path to the neighbor than whatever route may exist currently.
    for (byte neighbor_idx = 0; neighbor_idx < NUM_X_CELLS*NUM_Y_CELLS; ++neighbor_idx) {     
      short alt = -1;
      travel_cost = get_travel_cost(cur_vertex, neighbor_idx, source_floor);
      if (travel_cost == BIG_NUMBER || travel_cost == 0) {
        continue; // Nodes are not neighbors/cannot traverse... skip!
      }
      alt = dist[cur_vertex] + travel_cost;
      if (alt < dist[neighbor_idx]) {
        // New shortest path to node at neighbor_idx found!
        dist[neighbor_idx] = alt;
        Q_cost[neighbor_idx] = alt;
        prev[neighbor_idx] = cur_vertex;
      }
    } 
  }

  byte pathLength = 0;
  int vtx = dest_id;
  int prev_vtx = -1;
  while (vtx != -1){
    pathLength++;
    prev_vtx = vtx;
    vtx = prev[vtx];
  }
  if (prev_vtx != source_id){
    return NULL; // There is no path from the source to the destination
  }
  short* final_path = new short[pathLength + 1];
  final_path[pathLength] = -1;
  byte idx = pathLength - 1;
  
  vtx = dest_id;
  while (vtx != -1){
    final_path[idx] = vtx;
    vtx = prev[vtx];
    idx--;
  }
  
  return final_path;
}