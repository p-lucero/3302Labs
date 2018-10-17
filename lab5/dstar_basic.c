#include <sparki.h>
#include <math.h> //Included for floor()

struct list_elt {
	int id;
	struct list_elt* next; // FIXME broken?
};

struct list_elt* head = NULL;

int costs[NUM_X_CELLS * NUM_Y_CELLS]; // maintains cost to get somewhere?
int minimum_costs[NUM_X_CELLS * NUM_Y_CELLS]; // what?
int nextPoint[NUM_X_CELLS * NUM_Y_CELLS]; // maintains path to take from any state

void setup(){
	for (int i = 0; i < NUM_X_CELLS * NUM_Y_CELLS; i++){
		costs[i] = MAX_INT; // FIXME
	}
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

bool isRaise(int id){
	double cost;
	int i, j, delta_i, delta_j, neighbor_i, neighbor_j, neighbor_id;
	id_to_grid_coords(id, &i, &j); // FIXME?
	if (costs[id] < minimum_costs[id]){
		for (dir = 0; dir < 4; dir++){
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
			neighbor_id = grid_coords_to_id(neighbor_i, neighbor_j);
			// cost = point.calculateCostVia(neighbor); // FIXME
			if (cost < cost[id]){
				point.setNextPointAndUpdateCost(neighbor);
			}
		}
	}
	return costs[id] > minimum_costs[id];
}

bool coords_in_grid(int i, int j){
	return i > 0 && i < NUM_X_CELLS && j > 0 && j < NUM_Y_CELLS;
}

void expand(int id){
	bool raise = isRaise(id);
	double cost;
	int i, j, delta_i, delta_j, neighbor_i, neighbor_j, neighbor_id;
	id_to_grid_coords(id, &i, &j); // FIXME?
	for (dir = 0; dir < 4; dir++){
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
		neighbor_id = grid_coords_to_id(neighbor_i, neighbor_j);
		if (raise){
			if (nextPoint[neighbor_id] == id){
				// neighbor.setNextPointAndUpdateCost(currentPoint);
				nextPoint[neighbor_id] = id; // FIXME
				cost[neighbor_id] = cost[id] + 1; // FIXME
				addToList(neighbor_id);
			}
			else {
				// cost = neighbor.calculateCostVia(currentPoint); // FIXME
				if (cost < cost[neighbor_id]){
					// currentPoint.setMinimumCostToCurrentCost(); // FIXME
					addToList(id);
				}
			}
		}
		else {
			// cost = neighbor.calculateCostVia(currentPoint); // FIXME
			if (cost < cost[neighbor_id]){
				// neighbor.setNextPointAndUpdateCost(currentPoint);
				nextPoint[neighbor_id] = id; // FIXME
				cost[neighbor_id] = cost[id] + 1; // FIXME
				addToList(neighbor_id);
			}
		}
	}
}

void addToList(int id){
	if (head == NULL){
		head = malloc(sizeof(list_elt));
		head->id = id;
		head->next = NULL;
	}
	else {
		struct list_elt* curr;
		// the following for loop serves no purpose other than to iterate to the final list element
		for (curr = head; curr->next != NULL; curr = curr->next){}
		curr->next = malloc(sizeof(list_elt));
		curr->next->id = id;
		curr->next->next = NULL;
	}
}

void loop(){
	// code that goes before mapping code

	while (!listIsEmpty()){
		point = getFirstInList();
		expand(point);
	}

	// other code, probably to find obstacles etc

	if (foundObstacle){ // FIXME
		addToList(obstacle);
	}

	// other code, probably dealing with loop timing, navigation, and whatnot
}

