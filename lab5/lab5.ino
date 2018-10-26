/**
 * IMPORTANT: Read through the code before beginning implementation!
 * Your solution should fill in the various "TODO" items within this starter code.
 */

#include <sparki.h>

#define CYCLE_TIME .100

// Number of vertices to discretize the map
#define NUM_X_CELLS 4
#define NUM_Y_CELLS 4

// Map is ~60cm x 42cm
#define MAP_SIZE_X 0.6
#define MAP_SIZE_Y 0.42

#define BIG_NUMBER 9999

// Grid map from Lab 4: values of 1 indicate free space, 0 indicates occupied space
bool world_map[NUM_Y_CELLS][NUM_X_CELLS];

// Start (I,J) grid coordinates
int source_i = 0;
int source_j = 0;

// Destination (I,J) grid coordinates
int goal_i = 3;
int goal_j = 3;

// DON'T CHANGE THESE HERE; they'll be overwritten in setup. Change the two above coordinate pairs instead!
int source_index = 0;
int destination_index = 0;

void setup() {   
  // Dijkstra Setup -- initialize empty world map
  for (int j = 0; j < NUM_Y_CELLS; ++j) {
    for (int i = 0; i < NUM_X_CELLS; ++i) {
      world_map[j][i] = 1;
    }
  }
  source_index = ij_coordinates_to_vertex_index(source_i, source_j);
  destination_index = ij_coordinates_to_vertex_index(goal_i, goal_j);
  // FINISHED: Set up your map here by setting individual cells to 0 to indicate obstacles
  world_map[1][0] = 0;
  world_map[1][1] = 0;
  world_map[2][3] = 0;
}

/*****************************
 * Dijkstra Helper Functions *
 *****************************/

// Return 0 if there are entries in range [0,inf) in arr
// otherwise return 1, signifying empty queue
bool is_empty(int *arr, int len) {
  for (int i=0; i < len; ++i) {
    if (arr[i] >= 0) {
      return false;
    }
  }
  return true;
}


// Return the index with the minimum value in int array "arr" of length "len"
// Assumes positive values only, with values of "-1" indicating 'empty'
int get_min_index(int *arr, int len) {
  int min_val=-1, min_idx=-1;
  for (int i=0;i < len; ++i) {
    if (arr[i] < min_val || min_val == -1) {
      min_val = arr[i];
      min_idx = i;
    }
  }
  return min_idx;
}


/**********************************
 * Coordinate Transform Functions *
 **********************************/

// Converts a vertex index into (I,J) coordinates
// Returns 0 if something went wrong -- assume invalid i and j values being set
bool vertex_index_to_ij_coordinates(int v_idx, int *i, int *j) {
  *i = v_idx % NUM_X_CELLS;
  *j = v_idx / NUM_X_CELLS;
  
  if (*i < 0 || *j < 0 || *i >= NUM_X_CELLS || *j >= NUM_Y_CELLS) return false;
  return true;
}

// Converts (I,J) coordinates into a vertex index
int ij_coordinates_to_vertex_index(int i, int j) {
  return j*NUM_X_CELLS + i;  
}

// Convert (i,j) coordinates into world coordinates
// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. Assigned x and y values are the middle of cell (i,j)
bool ij_coordinates_to_xy_coordinates(int i, int j, float *x, float *y) {
  if (i < 0 || j < 0 || i >= NUM_X_CELLS || j >= NUM_Y_CELLS) return false;
  
  *x = (i+0.5)*(MAP_SIZE_X/NUM_X_CELLS);
  *y = (j+0.5)*(MAP_SIZE_Y/NUM_Y_CELLS);
  return true;  
}

// Convert (x,y) world coordinates into (i,j) grid coordinates
// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool xy_coordinates_to_ij_coordinates(float x, float y, int *i, int *j) {
  if (x < 0 || y < 0 || x >= MAP_SIZE_X || y >= MAP_SIZE_Y) return false;
  
  *i = int((x/MAP_SIZE_X) * NUM_X_CELLS);
  *j = int((y/MAP_SIZE_Y) * NUM_Y_CELLS);

  return true;  
}

/**********************************
 *      Core Dijkstra Functions   *
 **********************************/

// Returns the cost of moving from vertex_source to vertex_dest
int get_travel_cost(int vertex_source, int vertex_dest) {
  int source_i, source_j, dest_i, dest_j;
  
  // Return BIG_NUMBER if we're outside of the grid space that we can accept
  if (!vertex_index_to_ij_coordinates(vertex_source, &source_i, &source_j)
      || !vertex_index_to_ij_coordinates(vertex_dest, &dest_i, &dest_j)){
    return BIG_NUMBER;
  }
  
  // Return 1 if they are adjacent and both unoccupied
  if (((abs(source_i - dest_i) == 1 && source_j - dest_j == 0) ||
      (abs(source_j - dest_j) == 1 && source_i - dest_i == 0)) &&
     world_map[source_i][source_j] && world_map[dest_i][dest_j]){
    return 1;
  }
  
  // Default to return BIG_NUMBER if they are not adjacent or one is occupied
  return BIG_NUMBER;
}


// Allocate and return a list of ints corresponding to the "prev" variable in Dijkstra's algorithm
// The returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
int *run_dijkstra(int source_vertex) {
  // Array mapping vertex_index to distance of shortest path from source_vertex to vertex_index.
  int dist[NUM_X_CELLS*NUM_Y_CELLS];
  
  // Queue for identifying which vertices are still being explored -- Q_cost[vertex_index] = shortest known dist to get to vertex_index. 
  // Q_cost[vertex_index] = -1 if the vertex is no longer being considered.
  int Q_cost[NUM_Y_CELLS*NUM_X_CELLS]; 

  // Initialize memory for prev array
  int *prev = new int[NUM_X_CELLS*NUM_Y_CELLS];
  
  int u, i, j, delta_i, delta_j, neighbor_i, neighbor_j, neighbor, alt;

  /**
   * FINISHED: Insert your Dijkstra's code here
   */
  for (int i = 0; i < NUM_X_CELLS * NUM_Y_CELLS; i++){
    dist[i] = BIG_NUMBER;
    prev[i] = -1;
    Q_cost[i] = BIG_NUMBER;
  }
   
  dist[source_vertex] = 0;

  while (!is_empty(Q_cost, NUM_X_CELLS*NUM_Y_CELLS)){
    u = get_min_index(Q_cost, NUM_X_CELLS*NUM_Y_CELLS);
    Q_cost[u] = -1;
    vertex_index_to_ij_coordinates(u, &i, &j);
    for (int dir = 0; dir < 4; dir++){
      switch(dir){
        case 0:
          delta_i = 1;
          delta_j = 0;
          break;
        case 1:
          delta_i = -1;
          delta_j = 0;
          break;
        case 2:
          delta_i = 0;
          delta_j = 1;
          break;
        case 3:
          delta_i = 0;
          delta_j = -1;
          break;
      }
      neighbor_i = i + delta_i;
      neighbor_j = j + delta_j;     
      if (neighbor_i < 0 || neighbor_i >= NUM_X_CELLS || neighbor_j < 0 || neighbor_j >= NUM_Y_CELLS){
        continue;
      } // This is very necessary to avoid array accesses out of bounds
      neighbor = ij_coordinates_to_vertex_index(neighbor_i, neighbor_j);
      alt = dist[u] + get_travel_cost(u, neighbor);
      if (alt < dist[neighbor]){
        dist[neighbor] = alt;
        prev[neighbor] = u;
        // if (Q_cost[neighbor] != -1){
        //   Q_cost[neighbor] = alt;
        // } // currently broken
      }
    }
  }
  return prev;
}

// Given a populated 'prev' array, a source vertex, and destination vertex,
// allocate and return an integer array populated with the path from source to destination.
// The first entry of your path should be source_vertex and the last entry should be "-1" 
// to indicate the end of the array since paths can be variable length.
int *reconstruct_path(int *prev, int source_vertex, int dest_vertex) {
  int pathLength = 0;
  int vtx = dest_vertex;
  int prev_vtx = -1;
  while (vtx != -1){
    pathLength++;
    prev_vtx = vtx;
    vtx = prev[vtx];
  }
  if (prev_vtx != source_vertex){
    return NULL; // There is no path from the source to the destination
  }
  int* final_path = new int[pathLength + 1];
  final_path[pathLength] = -1;
  int idx = pathLength - 1;
  
  vtx = dest_vertex;
  while (vtx != -1){
    final_path[idx] = vtx;
    vtx = prev[vtx];
    idx--;
  }
  
  return final_path;  
}

void loop () {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  int *prev = NULL;
  int *path = NULL;

  /**
   * FINISHED: Populate prev with dijkstra's algorithm, then populate path with reconstruct_path
   */
  
  prev = run_dijkstra(source_index);
  path = reconstruct_path(prev, source_index, destination_index);
 
  if (prev != NULL) {
    delete prev; 
    prev = NULL; // Once we have the path, don't need to keep prev around in memory anymore.
  }

  sparki.clearLCD();

  // FINISHED
  // Display the final path in the following format:
  //
  //  Source: (0,0)
  //  Goal: (3,1)
  //  0 -> 1 -> 2 -> 6 -> 7
  sparki.print("Source: (");
  sparki.print(source_i);
  sparki.print(",");
  sparki.print(source_j);
  sparki.println(")");
  sparki.print("Goal: (");
  sparki.print(goal_i);
  sparki.print(",");
  sparki.print(goal_j);
  sparki.println(")");
  
  // Print the path; the last element is -1, which is not actually part of our path
  if (path != NULL){ // avoid NPE
    for (int i = 0; path[i] != -1; i++){
      sparki.print(path[i]);
      if (path[i+1] != -1){
        sparki.print(" -> ");  
      }
    }
  }
  else {
    sparki.print("No path from source to goal");
  }
  
  sparki.updateLCD();

  if (path != NULL) {
    delete path; 
    path=NULL; // Important! Delete the arrays returned from run_dijkstra and reconstruct_path when you're done with them!
  } 
 
  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10); // Accept some error
}
