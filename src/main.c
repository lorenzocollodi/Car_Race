//------------------------------------------------------------------------------
// CAR RACE:  Simulation of a car race with N≤20 cars. One of them can be
//  optionally controlloed by the user
//
// TASK_MANAGEMENT: This file contains the main of the program
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INCLUDED LIBRARIES
//------------------------------------------------------------------------------

#include "task_management.h"
#include "car.h"

//------------------------------------------------------------------------------
// The main only calls the initialization and termination functions
//------------------------------------------------------------------------------

int main(void){
  init();
  terminate();
  return 0;
}
