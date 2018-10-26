#include <sparki.h>
#include <math.h> //Included for floor()
#include <limits.h>

#define NUM_X_CELLS 6
#define NUM_Y_CELLS 6

#define DESTINATION 0 // FIXME set this to whatever our destination node happens to be

// linked lists like the below are safe, but they're also memory intensive
// we may have to switch to a circular array with size NUM_X_CELLS * NUM_Y_CELLS at some point?
// but I didn't want to implement that just yet so this is a quick-and-dirty implementation
// FIXME? if we run out of memory and Sparki dies
struct list_elt {
	int id;
	struct list_elt* next;
};

struct list_elt* head = NULL;

bool world_map[NUM_X_CELLS][NUM_Y_CELLS];

int costs[NUM_X_CELLS * NUM_Y_CELLS]; // maintains cost to get somewhere?
int minimum_costs[NUM_X_CELLS * NUM_Y_CELLS]; // what?
int nextPoint[NUM_X_CELLS * NUM_Y_CELLS]; // maintains path to take from any state

void setup(){
	int j, k;
	for (int i = 0; i < NUM_X_CELLS * NUM_Y_CELLS; i++){
		costs[i] = INT_MAX;
		minimum_costs[i] = INT_MIN;
		nextPoint[i] = -1;
		cell_id_to_coords(i, &j, &k);
		world_map[j][k] = true;
	}
	addToList(DESTINATION);
	// other setup code...
}

bool listIsEmpty(){
	return head == NULL;
}

int getFirstInList(){
	int id_storage = head->id;
	struct list_elt* list_storage = head;
	head = head->next;
	free(list_storage);
}

int cell_coords_to_id(int i, int j){
	return j * NUM_X_CELLS + i;
}

void cell_id_to_coords(int id, int* i, int* j){
	*i = id % NUM_X_CELLS;
	*j = id / NUM_X_CELLS;
}

int calculateCostVia(int point, int neighbor){
	// TODO
	return 0;
}

bool isRaise(int id){
	double cost;
	int i, j, delta_i, delta_j, neighbor_i, neighbor_j, neighbor_id;
	cell_id_to_coords(id, &i, &j);
	if (costs[id] < minimum_costs[id]){
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
			if (coords_in_grid(neighbor_i, neighbor_j)){
				neighbor_id = cell_coords_to_id(neighbor_i, neighbor_j);
				cost = calculateCostVia(id, neighbor_id);
				if (cost < costs[id]){
					nextPoint[id] = neighbor_id;
					costs[id] = cost;
				}
			}
		}
	}
	return costs[id] > minimum_costs[id];
}

bool coords_in_grid(int i, int j){
	return i >= 0 && i < NUM_X_CELLS && j >= 0 && j < NUM_Y_CELLS;
}

void expand(int id){
	bool raise = isRaise(id);
	double cost;
	int i, j, delta_i, delta_j, neighbor_i, neighbor_j, neighbor_id;
	cell_id_to_coords(id, &i, &j);
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
		if (coords_in_grid(neighbor_i, neighbor_j)){
			neighbor_id = cell_coords_to_id(neighbor_i, neighbor_j);
			if (raise){
				if (nextPoint[neighbor_id] == id){
					// neighbor.setNextPointAndUpdateCost(currentPoint);
					nextPoint[neighbor_id] = id; // FIXME? + the below
					costs[neighbor_id] = costs[id] + 1;
					addToList(neighbor_id);
				}
				else {
					cost = calculateCostVia(neighbor_id, id);
					if (cost < costs[neighbor_id]){
						minimum_costs[id] = costs[id]
						addToList(id);
					}
				}
			}
			else {
				cost = calculateCostVia(neighbor_id, id);
				if (cost < costs[neighbor_id]){
					// neighbor.setNextPointAndUpdateCost(currentPoint);
					nextPoint[neighbor_id] = id; // FIXME? + the below
					costs[neighbor_id] = costs[id] + 1;
					addToList(neighbor_id);
				}
			}
		}
	}
}

void addToList(int id){
	if (head == NULL){
		head = (struct list_elt*) malloc(sizeof(list_elt));
		head->id = id;
		head->next = NULL;
	}
	else {
		struct list_elt* curr;
		// the following for loop serves no purpose other than to iterate to the final list element
		for (curr = head; curr->next != NULL; curr = curr->next){}
		curr->next = (struct list_elt*) malloc(sizeof(list_elt));
		curr->next->id = id;
		curr->next->next = NULL;
	}
}

void loop(){
	int point, obstacle;
	bool foundObstacle;
	// code that goes before mapping code

	while (!listIsEmpty()){
		point = getFirstInList();
		expand(point);
	}

	// other code, probably to find obstacles etc and set the below variables

	if (foundObstacle){
		addToList(obstacle);
	}

	// other code, probably dealing with loop timing, navigation, and whatnot
	// should also check what the robot's current grid coordinates are and make the next move to make from there
}
