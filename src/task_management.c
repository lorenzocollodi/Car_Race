// Contains the definition of the functions declared by task_management.h

#include "task_management.h"

//------------------------------------------------------------------------------
// Initializes allegro and keyboard, than it sets program_state to STARTUP
// and it calls the other initialization functions
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void init(){
  allegro_init();
  install_keyboard();
  install_mouse();
  enable_hardware_cursor();
  gtask_init();
  ktask_init();
  ctask_init();
  stask_init();
  mtask_init();
  etask_init();
  printf("Threads initialized \n");
  return;
}

//------------------------------------------------------------------------------
// Initializes and creates graphic thread and loads track image

// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void gtask_init(){
  set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
  gtask_par.period = GPER;
  gtask_par.deadline = GPER;
  gtask_par.priority = GPRIORITY;
  gtask_par.dmiss = 0;
  pthread_attr_init(&gtask_attr);
  pthread_attr_setinheritsched(&gtask_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&gtask_attr, SCHED_FIFO );
  g_sched_param.sched_priority = gtask_par.priority;
  pthread_attr_setschedparam(&gtask_attr, &g_sched_param);

  if (pthread_create(&gid, &gtask_attr, graphic_task, &gtask_par))
    printf("Error when creating graphic thread \n");
  else
    printf("Correctly created graphic thread \n");
  return;
}

//------------------------------------------------------------------------------
// Initializes and creates mouse thread
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void mtask_init(){
  mtask_par.period = MPER;
  mtask_par.deadline = MPER;
  mtask_par.priority = MPRIORITY;
  mtask_par.dmiss = 0;
  pthread_attr_init(&mtask_attr);
  pthread_attr_setinheritsched(&mtask_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&mtask_attr, SCHED_FIFO );
  m_sched_param.sched_priority = mtask_par.priority;
  pthread_attr_setschedparam(&mtask_attr, &m_sched_param);

  if (pthread_create(&mid, &mtask_attr, mouse_task, &mtask_par))
    printf("Error when creating mouse thread \n");
  else
    printf("Correctly created mouse thread \n");
  return;
}

//------------------------------------------------------------------------------
// Initializes and creates keyboard thread
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void ktask_init(){
  ktask_par.period = KPER;
  ktask_par.deadline = KPER;
  ktask_par.priority = KPRIORITY;
  ktask_par.dmiss = 0;
  pthread_attr_init(&ktask_attr);
  pthread_attr_setinheritsched(&ktask_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&ktask_attr, SCHED_FIFO );
  k_sched_param.sched_priority = ktask_par.priority;
  pthread_attr_setschedparam(&ktask_attr, &k_sched_param);

  if (pthread_create(&kid, &ktask_attr, keyboard_task, &ktask_par))
    printf("Error when creating keyboard thread \n");
  else
    printf("Correctly created keyboard thread \n");
  return;
}

//------------------------------------------------------------------------------
// Initializes the car structure and their position, then it creates the
// thread for position management
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void ctask_init(){
  ctask_par.period = CPER;
  ctask_par.deadline = CPER;
  ctask_par.priority = CPRIORITY;
  ctask_par.dmiss = 0;
  pthread_attr_init(&ctask_attr);
  pthread_attr_setinheritsched(&ctask_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&ctask_attr, SCHED_FIFO );
  c_sched_param.sched_priority = ctask_par.priority;
  pthread_attr_setschedparam(&ctask_attr, &c_sched_param);

  if (pthread_create(&cid, &ctask_attr, car_task, &ctask_par))
    printf("Error when creating car thread \n");
  else
    printf("Correctly created car thread \n");
  return;
}

//------------------------------------------------------------------------------
// Initializes the sensor structure and the desired speed, then it creates the
// thread for sensors management
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void stask_init(){
  int i;
  pthread_mutex_lock(&sensor_mux);
  for(i = 0; i < n_cars; i++){
    sensors[i].module = 5;
    sensors[i].orientation = 0;
  }
  pthread_mutex_unlock(&sensor_mux);
  stask_par.period = SPER;
  stask_par.deadline = SPER;
  stask_par.priority = SPRIORITY;
  stask_par.dmiss = 0;
  pthread_attr_init(&stask_attr);
  pthread_attr_setinheritsched(&stask_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&stask_attr, SCHED_FIFO );
  s_sched_param.sched_priority = stask_par.priority;
  pthread_attr_setschedparam(&stask_attr, &s_sched_param);

  if (pthread_create(&sid, &stask_attr, sensor_task, &stask_par))
    printf("Error when creating sensor thread \n");
  else
    printf("Correctly created sensor thread \n");
  return;
}

//------------------------------------------------------------------------------
// Setup the experiment task
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void etask_init(){
  etask_par.period = EPER;
  etask_par.deadline = EPER;
  etask_par.priority = EPRIORITY;
  etask_par.dmiss = 0;
  pthread_attr_init(&etask_attr);
  pthread_attr_setinheritsched(&etask_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&etask_attr, SCHED_FIFO );
  e_sched_param.sched_priority = etask_par.priority;
  pthread_attr_setschedparam(&etask_attr, &e_sched_param);

  if (pthread_create(&eid, &etask_attr, experiment_task, &etask_par))
    printf("Error when creating experiment thread \n");
  else
    printf("Correctly created experiment thread \n");
  return;
}

//------------------------------------------------------------------------------
// Joins every thread and destroys the corresponding attribute structures. Then
// shuts down allegro
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void terminate(){
  pthread_join(kid, NULL);
  pthread_attr_destroy(&ktask_attr);
  pthread_join(gid, NULL);
  pthread_attr_destroy(&gtask_attr);
  pthread_join(cid, NULL);
  pthread_attr_destroy(&ctask_attr);
  pthread_join(eid, NULL);
  pthread_attr_destroy(&etask_attr);
  pthread_join(sid, NULL);
  pthread_attr_destroy(&stask_attr);
  pthread_join(mid, NULL);
  pthread_attr_destroy(&mtask_attr);
  allegro_exit();
  return;
}

//------------------------------------------------------------------------------
// Takes the task parameters as input and it reinitializes the deadline to
// current time plus period.
//
// inputs:
// *tp: pointer to the task parameters to modify with the new period
//
// returns:
// None
//------------------------------------------------------------------------------

void set_period(struct task_par *tp){
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  time_copy(&(tp->at), t);
  time_copy(&(tp->dl), t);
  time_add_ms(&(tp->at), tp->period);
  time_add_ms(&(tp->dl), tp->period);
  return;
}

//------------------------------------------------------------------------------
// Called at the end of a job. It makes the thread sleep until its deadline
// and then reinitializes it to the old deadline plus period
//
// inputs:
// *tp: pointer to the task parameters containing the period to wait
//
// returns:
// None
//------------------------------------------------------------------------------

void wait_for_period(struct task_par *tp){
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->at), NULL);
  time_add_ms(&(tp->at), tp->period);
  time_add_ms(&(tp->dl), tp->period);
  return;
}

//------------------------------------------------------------------------------
// Checks if the current job missed its deadline and if so it increases the
// deadline counter and returns 1, otherwise it returns 0
//
// inputs:
// *tp: pointer to the task parameters containing the deadline
//
// returns:
// None
//------------------------------------------------------------------------------

int deadline_miss(struct task_par *tp){
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  if (time_cmp(now, tp->dl) > 0) {
    tp->dmiss++;
    return 1;
  }
  return 0;
}

//------------------------------------------------------------------------------
// Copies the time of a timespec structure to another one (td is destination,
// ts is source)
//
// inputs:
// *td: pointer to the destination variable
// *ts: pointer to the source variable
//
// returns:
// None
//------------------------------------------------------------------------------

void time_copy(struct timespec *td, struct timespec ts){
  td->tv_sec = ts.tv_sec;
  td->tv_nsec = ts.tv_nsec;
  return;
}

//------------------------------------------------------------------------------
// Increases the time specified in a timespec structure by an integer
// amount of milliseconds
//
// inputs:
// *t: pointer to the time structure
// ms: umber of ms to add to the time
//
// returns:
// None
//------------------------------------------------------------------------------

void time_add_ms(struct timespec *t, int ms){
  t->tv_sec += ms/1000;
  t->tv_nsec += (ms%1000)*1000000;
  if (t->tv_nsec > 1000000000) {
    t->tv_nsec -= 1000000000;
    t->tv_sec += 1;
  }
  return;
}

//------------------------------------------------------------------------------
// Compares two times and returns 1 if t1 is bugger than t2, -1 if t2 is
// bigger than t1, 0 if they are equal
//
// inputs:
// *t1: pointer to the structure t1
// *t2: pointer to the structure t2
//
// returns:
// an integer indicating which of the two times is higher
//------------------------------------------------------------------------------

int time_cmp(struct timespec t1, struct timespec t2){
  if (t1.tv_sec > t2.tv_sec) return 1;
  if (t1.tv_sec < t2.tv_sec) return -1;
  if (t1.tv_nsec > t2.tv_nsec) return 1;
  if (t1.tv_nsec < t2.tv_nsec) return -1;
  return 0;
}
