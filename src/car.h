//------------------------------------------------------------------------------
// CAR RACE:  Simulation of a car race with N≤20 cars. One of them can be
//  optionally controlloed by the user
//
// CAR: This file contains the declaration of tasks and all the related
//  functions. It also contains the constants that are used by the program
//  and the global structures and variables. The actual definition of the
//  functions are in the separated file car.c, which also contains the
//  description of what functions and task do.
//
//  The tasks are 5:
//  -graphic: draws the visualized scene onto the screen
//  -keyboard: interprets the user input commands from keyboard
//  -mouse: interprets the mouse inputs
//  -car: computes the car acceleration, according to the user inputs or the
//    desired velocity computed by the sensors. Then integrates acceleration
//    and velocity to get velocity and position
//  -sensor: computes the desired velocity of each car, so that the autopilot
//    algorithm, implemented by car thread, can compute acceleration
//  -experiment: measures some variables during the execution
//------------------------------------------------------------------------------

#ifndef CAR_H_
#define CAR_H_

//------------------------------------------------------------------------------
// INCLUDED LIBRARIES
//------------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <allegro.h>
#include "task_management.h"

//------------------------------------------------------------------------------
// GRAPHIC CONSTANTS
//------------------------------------------------------------------------------

#define XWIN 1080           // x resolution
#define YWIN 960            // y resolution
#define BKG 16              // background color
#define MCOL 14             // menu color
#define CAR_WIDTH 5         // half of car width
#define CAR_HEIGHT 10       // half of car height
#define GRASS 2             // grass color
#define ROAD 7              // road color
#define ARRIVE 15           // color of the arrive line

//------------------------------------------------------------------------------
// CARS CONSTANTS
//------------------------------------------------------------------------------

#define MAX_CARS 20         // maximum number of cars
#define V_ATTR 0.2          // viscous friction coefficient
#define D_ATTR_R 0.2        // dynamic friction coefficient on the road
#define D_ATTR_G 0.8        // dynamic friction coefficient on the grass
#define STEER 0.002         //steering angle
#define N_TRACKS 3          // number of possible tracks
#define MAX_LAPS 5          // maximum number of laps


//------------------------------------------------------------------------------
// AUXILIARY CONSTANTS
//------------------------------------------------------------------------------

#define YES 1
#define NO 0

// steering direction
#define DIR_DX 1
#define DIR_SX -1
#define DIR_STR 0

// Program states
#define END 0
#define MENU 1
#define RACE 2
#define PAUSE 3
#define DRAW 4
#define STARTUP 5

// Sensor constants
#define SSTEP 10
#define SMIN 15
#define SMAX 300

// Number of data recorded in the experiment and its duration in 1/100 of second
#define EXPERIMENT_DURATION 2000


//------------------------------------------------------------------------------
//  STRUCTURES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Definition of a new type velocity consisting of module and orientation. The
// orientation is an angle that will be scaled to have values between 0 and 1
//------------------------------------------------------------------------------

typedef struct velocity{
  float module;
  float orientation;
} velocity;

//------------------------------------------------------------------------------
// Definition of a new type acceleration consisting of module and orientation.
// Module and orientation are integers and can have values -1, 0, +1
//------------------------------------------------------------------------------

typedef struct acceleration{
  int module;                   // +1 if accelerating, -1 if braking
  int orientation;                // +1 if steering right, -1 if steering left
} acceleration;

//------------------------------------------------------------------------------
// Definition of a new type car storing the current state of the car (velocity,
// acceleration and position) along with some variables that represent some
// characteristics of that specific car
//------------------------------------------------------------------------------

typedef struct car{
int c;                // color of the car
velocity  vel;
acceleration  acc;
float power;          // a variable that regulates the maximum reachable speed
float x;              // x position on the screen
float y;              // y position on the sceren
int position;         // final placement (first, second...)
} car;

//------------------------------------------------------------------------------
// Definition of the information measured from the front lidar. It returns
// the distance an obstacle and also says if it is a cas or not
//------------------------------------------------------------------------------

typedef struct front_lidar_measure{
  int is_car;
  int d;
} front_lidar_measure;


//------------------------------------------------------------------------------
//  GLOBAL VARIABLES
//------------------------------------------------------------------------------

// vector of cars, this structure contains all the informations about the cars
static car cars[MAX_CARS];

// Autopilot variable
static int  autopilot = NO;

// Possible car colors
static const int  colors[MAX_CARS] = {100, 150, 160, 170, 180, 190, 200, 50,
                                      70, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// structure containing the desired velocity as computed by the sensors
static velocity   sensors[MAX_CARS];

// state of the program
static int  program_state = STARTUP;

// number of active cars
static int  n_cars = 1;

 // bitmap images storing the available tracks
static BITMAP	*track[N_TRACKS];

// bitmap image storing the drawn track when drawing a new one
static BITMAP	*drawn_track;

// variable indexing the currently chosen track
static int  active_track = 0;

// radius of the drawn circle when drawing a track
static int  radius = 50;

// string_buffer made static because is shared by a couple of threads
static char string_buffer[100];

// variable that indicates which color to draw on the screen when drawing
static int  drawn_color = ROAD;

// number of laps to be reached in order to complete the race
static int  n_laps = 1;

// number of cars that have completed the race
static int  n_arrived = 1;

// vector keeping track of the number of laps completed by each car
static int  laps[MAX_CARS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                              -1, -1, -1, -1, -1, -1, -1, -1};

                              //------------------------------------------------------------------------------
// vector keeping track of which cars are currently on the arrive, otherwise
// every lap could be counted many times
//------------------------------------------------------------------------------

static int  added_lap[MAX_CARS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0};

// variable indicating if the experimental measures have to be recorded or not
static int  measure = NO;

//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
//------------------------------------------------------------------------------

void *graphic_task(void *arg);
void *keyboard_task(void *arg);
void *mouse_task(void *arg);
void *car_task(void *arg);
void *sensor_task(void *arg);
void *experiment_task(void *arg);
void draw_car(int i, BITMAP *screen_buf, BITMAP *car);
void make_pink(BITMAP *car);
void recolor (BITMAP *car, int color);
void draw_menu(BITMAP	*screen_buf);
void screen_startup();
int keyboard_menu();
void keyboard_acc();
void keyboard_steer();
void compute_acc(int car);
void compute_position(int car);
void compute_speed(int car);
int read_sensor(int x0, int y0, float alpha);
void handle_collisions();
void reset_cars();
velocity scan_car(int car);
velocity scan_car_2(int car);
void save_drawn(int i);
void draw_track();
void add_lap(int i);
front_lidar_measure read_front_sensor(int x0, int y0, float alpha);
int are_close(int k, int i);
void collide(int k, int i);
void draw_mode_text(BITMAP *screen_buf);
void keyboard_race();
void keyboard_draw();
void keyboard_pause();
void measure_velocity(FILE *file);
int count_misses();
void zero_misses();

#endif
