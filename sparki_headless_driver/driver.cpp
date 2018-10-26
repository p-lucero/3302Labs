// A driver for testing code to run on Sparki when you don't have a Sparki to test with
// Very very minimal

#include <cstddef>
#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "sparki.h"

Sparki sparki;
serial Serial; // aaaaaaaaaaaaaa

// PUT YOUR SPARKI CODE HERE

int main(){
	setup();
	while (true){
		loop();
	}
}