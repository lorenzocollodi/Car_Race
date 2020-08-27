//------------------------------------------------------------------------------
// CAR RACE:  Simulation of a car race with N≤20 cars. One of them can be
//  optionally controlloed by the user
//
// TASK_MANAGEMENT: This file contains the tasks initialization, along with
//  the constants and structures that define their behaviour.
//  This file also contains the functions that operate on time structures, such
//  as comparing two time instants or summing two times.
//  Note that the actual definition of every function is in a separated file,
//  task_management.c, together with the description of what the functions do.
//  This file does not define the tasks, which are in another file CAR.H
//------------------------------------------------------------------------------

#ifndef TASK_H_
#define TASK_H_

//------------------------------------------------------------------------------
//INCLUDED LIBRARIES
//------------------------------------------------------------------------------

#include <time.h>
#include <pthread.h>
#include <sched.h>
#include "car.h"


//------------------------------------------------------------------------------
// TASK CONSTANTS
//------------------------------------------------------------------------------

#define GPER 30             // graphic task period in ms
#define KPER 100            // keyboard task period in ms
#define CPER 10             // car task period in ms
#define SPER 10             // sensor task period in ms
#define MPER 20             // mouse task period in ms
#define EPER 10             // experiment task period
#define GPRIORITY 10        // graphic task priority
#define SPRIORITY 80        // sensor task priority
#define CPRIORITY 50        // car task priority
#define KPRIORITY 20        // keyboard task priority
#define MPRIORITY 1         // mouse task priority
#define EPRIORITY 90        // experiment task priority


//------------------------------------------------------------------------------
// TASKS DESCRIPTION
//------------------------------------------------------------------------------

//Thread structure
struct  task_par {
  int period;
  int deadline;
  int priority;
  int dmiss;
  struct timespec at;
  struct timespec dl;
};

// scheduling parameters for each thread
static struct sched_param g_sched_param;
static struct sched_param k_sched_param;
static struct sched_param c_sched_param;
static struct sched_param s_sched_param;
static struct sched_param m_sched_param;
static struct sched_param e_sched_param;

// parameters for each thread15
static struct task_par  gtask_par;
static struct task_par  ktask_par;
static struct task_par  ctask_par;
static struct task_par  stask_par;
static struct task_par  mtask_par;
static struct task_par  etask_par;

// attributes for each thread
static pthread_attr_t ktask_attr;
static pthread_attr_t ctask_attr;
static pthread_attr_t stask_attr;
static pthread_attr_t mtask_attr;
static pthread_attr_t gtask_attr;
static pthread_attr_t etask_attr;

// thread identifiers
static pthread_t  gid;
static pthread_t  kid;
static pthread_t  cid;
static pthread_t  sid;
static pthread_t  mid;
static pthread_t  eid;

// mutex to deal with concurrency
static pthread_mutex_t  state_mux = PTHREAD_MUTEX_INITIALIZER;          // semaphore for program state
static pthread_mutex_t  sensor_mux = PTHREAD_MUTEX_INITIALIZER;         // semaphore for state of the program
static pthread_mutex_t  car_mux = PTHREAD_MUTEX_INITIALIZER;            // semaphore for state of the program
static pthread_mutex_t  new_track = PTHREAD_MUTEX_INITIALIZER;          // semaphore for the new track bitmap


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
//------------------------------------------------------------------------------

void init();
void gtask_init();
void ktask_init();
void ctask_init();
void stask_init();
void mtask_init();
void etask_init();
void set_period(struct task_par *tp);
int deadline_miss(struct task_par *tp);
void wait_for_period(struct task_par *tp);
void time_copy(struct timespec *td, struct timespec ts);
void time_add_ms(struct timespec *t, int ms);
int time_cmp(struct timespec t1, struct timespec t2);
void terminate();

#endif
