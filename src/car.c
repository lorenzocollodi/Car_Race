// Contains the definition of the functions declared by car.h

#include "car.h"

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// This function takes a car number as input and assigns it the acceleration
// needed to reach the desidred speed assigned. The desired speed is foud
// in the sensors structure
//
// inputs:
// car: number of the car that must be updated
//
// returns: None
//------------------------------------------------------------------------------

void compute_acc(int car){

	pthread_mutex_lock(&sensor_mux);
  velocity	des = sensors[car];
	pthread_mutex_unlock(&sensor_mux);

	pthread_mutex_lock(&car_mux);
  if (des.module > cars[car].vel.module)
		cars[car].acc.module = 1;
  else
    cars[car].acc.module = 0;

	// Check if the desired orientation is actually right or left
  if ((cars[car].vel.orientation - des.orientation > 0.01 &&
        cars[car].vel.orientation - des.orientation < 0.49) ||
            (cars[car].vel.orientation - des.orientation < -0.51 &&
              cars[car].vel.orientation - des.orientation > - 0.99))
    cars[car].acc.orientation = -1;
  else{
    if ((cars[car].vel.orientation - des.orientation < -0.01 &&
          cars[car].vel.orientation - des.orientation > -0.49) ||
              (cars[car].vel.orientation - des.orientation > 0.51 &&
                cars[car].vel.orientation - des.orientation < 0.99))
    cars[car].acc.orientation = 1;
      else
        cars[car].acc.orientation = 0;
  }

	pthread_mutex_unlock(&car_mux);
  return;
};

//------------------------------------------------------------------------------
// This function takes a car number as input and performs a step of integration
// of the acceleration to calculate and assign the new velocity of the car.
//
// inputs:
// car: number of the car that must be updated
//
// returns:
// None
//------------------------------------------------------------------------------

void compute_speed(int car){
	int c, x, y;
	float friction = D_ATTR_R;

	pthread_mutex_lock(&car_mux);
  velocity	vel = cars[car].vel;
  float power = cars[car].power;
  x = cars[car].x;
  y = cars[car].y;
  acceleration	acc = cars[car].acc;

	pthread_mutex_unlock(&car_mux);

  c = getpixel(track[active_track], x, y);
  if (c == GRASS) friction = D_ATTR_G;

	// Discrete integration of acceleration
  if (vel.module > 0) vel.module = vel.module + (acc.module*power - V_ATTR*vel.module*vel.module - friction) * CPER/1000;
  if (vel.module < 0) vel.module = vel.module + (acc.module*power + V_ATTR*vel.module*vel.module + friction) * CPER/1000;
  if (vel.module == 0) vel.module = vel.module + (acc.module*power) * CPER/1000;

	vel.orientation = vel.orientation + (acc.orientation * vel.module * STEER) * CPER/10;
  if (vel.orientation > 1) vel.orientation -= 1;
  if (vel.orientation < 0) vel.orientation += 1;

	pthread_mutex_lock(&car_mux);
  cars[car].vel = vel;
	pthread_mutex_unlock(&car_mux);
  return;
};

//------------------------------------------------------------------------------
// This function takes a car number as input and performs a step of integration
// of the velocity to calculate and assign the new position of the car.
//
// inputs:
// car: number of the car that must be updated
//
// returns:
// None
//------------------------------------------------------------------------------

void compute_position(int car){
	pthread_mutex_lock(&car_mux);
	velocity	vel = cars[car].vel;
  float	angle = vel.orientation * 2 * M_PI;

	//Discrete 2D integration of velocity
  float	dx = (vel.module*cos(angle)) * CPER/10;
  float	dy = (vel.module*sin(angle)) * CPER/10;

  cars[car].x += dx;
  cars[car].y += dy;
	pthread_mutex_unlock(&car_mux);
  return;
};

//------------------------------------------------------------------------------
// Computes distance from closest obstacle in direction alpha from position
// (x, y), which is the position of the car. Used to realize the lateral sensors
//
// inputs:
// x0: position of the car along x axis
// y0: position of the car along y axis
// alpha: direction line to be scanned for obstacles
//
// returns:
// d: distance from the closest obstacle in direction d
//------------------------------------------------------------------------------

int read_sensor(int x0, int y0, float alpha){
	int	c;
  int	x, y;
  int	d = SMIN;

	// Check for pixels on a certain line until it finds an obstacle. The colors
	// that do not represent an obstacle are the road color and the arrive color
  do {
    x = x0 + d*cos(alpha);
    y = y0 + d*sin(alpha);
    c = getpixel(screen, x, y);
    d = d + SSTEP;
  } while ((d <= SMAX) && ((c == ROAD)||(c==ARRIVE)));

  return d;
}

//------------------------------------------------------------------------------
// This function handles the collisions between nearby cars
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void handle_collisions(){
  int i, k;
  for (i = 0; i< n_cars; i++){
    for (k = i + 1; k< n_cars; k++){
      if(are_close(k, i)) collide(k, i);
    }
  }
  return;
}

//------------------------------------------------------------------------------
// This function positions all the cars in their start position
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void reset_cars(){
  int x, y, c, i;
  c = 1;
	pthread_mutex_lock(&car_mux);
  y = 100;
  n_arrived = 1;
  for(i = 0; i < MAX_CARS; i++){
    do{
      y += 10;
      x = x % XWIN;
      do{
        x = (x + 30);
        c = getpixel(track[active_track], x, y);
      } while(c != ROAD && x < XWIN);
    } while(c != ROAD && y < YWIN);
    added_lap[i] = 0;
    laps[i] = -1;
    cars[i].x = x;
    cars[i].y = y;
    cars[i].power = 1 + rand()*2/(RAND_MAX);
    //printf("Power: %f \n", cars[i].power);
    cars[i].vel.module = 0;
    cars[i].vel.orientation = 0;
    cars[i].acc.module = 0;
    cars[i].acc.orientation = 0;
    cars[i].c = colors[i];
    cars[i].position = 0;
    x += 40;
    y += 10;
  }
  cars[0].power = 1.03;
	pthread_mutex_unlock(&car_mux);
  return;
}

//------------------------------------------------------------------------------
// This function scans the front of the car to find obstacles and returns
// the desired speed.
// The car tries to equilibrate the distance from the first obstacle at +30/-30
// degrees from its pointing direction. It brakes if it is close to an obstacle
// placed in front of it unless the obbstacle is another car. In that case the
// desired speed points to the direction in front of the car (+90/-90 degrees)
// where the next obstacle is farthest, in order to avoid the detected car. The
// second part of finding a way to surpass is implemented by scan_car2 below.
//
// inputs:
// car: number of the car whose sensor must be updated
//
// returns:
// des: the desired velocity
//------------------------------------------------------------------------------

velocity scan_car(int car){

	pthread_mutex_lock(&car_mux);
  float	orientation = (cars[car].vel.orientation);
  int	x = cars[car].x;
  int	y = cars[car].y;
	pthread_mutex_unlock(&car_mux);

  int front_dist = SMAX;
  int right_dist = SMAX;
  int left_dist = SMAX;
  velocity	des;
  float angle;
  int d;
  int i;
  front_lidar_measure front;

  angle = orientation * 2 * M_PI;
  front = read_front_sensor(x, y, angle);
  front_dist = front.d;

  if (front.is_car == 1)
    return  scan_car_2(car);
  else{
    des.module = 3;

// car: number of the car that must be updated
    angle =  M_PI / 6 + orientation * 2 * M_PI;
    right_dist = read_sensor(x, y, angle);

    angle = -M_PI / 6 + orientation * 2 * M_PI;
    left_dist = read_sensor(x, y, angle);

		if (front_dist < 200){
			des = scan_car_2(car);
			des.module = 1.2 + front_dist / 200;
		  return des;
		}

    d = (right_dist - left_dist) / 8;
    //printf("Diff: %i \n", d);
    angle = d;
    des.orientation = angle / SMAX + orientation;
    //printf("OR: %f \n", des.orientation);
    if (des.orientation > 1) des.orientation = des.orientation - 1;
    if (des.orientation < 0) des.orientation = des.orientation + 1;
  }
  return des;
}
velocity scan_car_2(int car){

	pthread_mutex_lock(&car_mux);
  float	orientation = (cars[car].vel.orientation) * 2 * M_PI;
  int	x = cars[car].x;
  int	y = cars[car].y;
	pthread_mutex_unlock(&car_mux);

  float angle;
  int d;
  int	max_dist = 0;
  velocity	des;
  int i;
// car: number of the car that must be updated

  for(i = 0; i < 11; i++){
    angle = M_PI * (i)/ 15 + orientation;
    d = read_sensor(x, y, angle);
    if (d > max_dist){
      max_dist = d;
      des.module = (d - SMIN) / 20;
      des.orientation = angle / (2 * M_PI);
      if (des.orientation > 1) des.orientation = des.orientation - 1;
      if (des.orientation < 0) des.orientation = des.orientation + 1;
    }
  }

  for(i = -1; i > -11; i--){
    angle = M_PI * (i)/ 15 + orientation;
    d = read_sensor(x, y, angle);
    if (d > max_dist){
      max_dist = d;
      des.module = (d - SMIN) / 4;
      des.orientation = angle/(2*M_PI);
      if (des.orientation > 1) des.orientation = des.orientation - 1;
      if (des.orientation < 0) des.orientation = des.orientation + 1;
    }
  }

  return des;
};

//------------------------------------------------------------------------------
// Takes the screen buffer and an integer as input and draws the corresponding
// car on the screen buffer in its current position and orientation

// inputs:
// i: number of the car to be drawn
// *screen_buf: pointer to the screen buffer
// *car: pointer to the car bitmap
//
// returns:
// None
//------------------------------------------------------------------------------

void draw_car(int i, BITMAP *screen_buf, BITMAP *car){

	pthread_mutex_lock(&car_mux);
  float	x = cars[i].x;
  float	y = cars[i].y;
  float	angle = cars[i].vel.orientation;
	pthread_mutex_unlock(&car_mux);

  x -= CAR_HEIGHT;
  y -= CAR_WIDTH;
  rotate_sprite(screen_buf, car, round(x), round(y), ftofix(angle * 256 + 64));
  return;
}

//------------------------------------------------------------------------------
// Changes the color of a car in a bmp to the desired color
//
// inputs:
// *car: pointer to the car bitmap
// color: color assigned to the car
//
// returns:
// None
//------------------------------------------------------------------------------

void recolor (BITMAP *car, int color){
	int  c, x, y;
  int pink = makecol(255, 0, 255);

  for (x = 0; x < car->w; x++){
    for (y = 0; y < car->h; y++) {
      c = getpixel(car, x, y);
      if (c == 51) c = colors[color];
      if (c == 15) c = ARRIVE;
      putpixel(car, x, y, c);
    }
  }
  return;
}

//------------------------------------------------------------------------------
// Makes the white background of a sprite pink. It was used to make the sprite
// background transparent, it could be removed, but it is retained if necessary
//
// inputs:
// *car: pointer to the car bitmap
//
// returns:
// None
//------------------------------------------------------------------------------

void make_pink(BITMAP *car){
	PALETTE	pal;
  int	c, x, y;
  int	pink;
  float	hue, sat, val;


  for (x = 0; x < car->w; x++){
    for (y = 0; y < car->h; y++) {
      c = getpixel(car, x, y);
      rgb_to_hsv(getr(c), getg(c), getb(c), &hue, &sat, &val);
      val = val * 255;
      if (val >= 240) c = pink;
      putpixel(car, x, y, c);
    }
  get_palette(pal);
  save_bitmap("car_2.bmp", car, pal);
  }
  return;
}

//------------------------------------------------------------------------------
// Draw the menu onto the screen buffer using the string buffer
//
// inputs:
// *screen_buf: pointer to the screen buffer
//
// returns:
// None
//------------------------------------------------------------------------------

void draw_menu(BITMAP	*screen_buf){
  clear_to_color(screen_buf, BKG);
  int	line = SCREEN_H/10;
  int i;
  textout_centre_ex(screen_buf, font, "Press S to start race", SCREEN_W/2, line, MCOL, BKG);
  line += 15;
  textout_centre_ex(screen_buf, font, "Press D to draw track", SCREEN_W/2, line, MCOL, BKG);
  line += 15;

  if (autopilot)
    textout_centre_ex(screen_buf, font, "Press A for Autopilot: now active", SCREEN_W/2, line, MCOL, BKG);
  else
    textout_centre_ex(screen_buf, font, "Press A for Autopilot: now inactive", SCREEN_W/2, line, MCOL, BKG);

  line += 15;
  textout_centre_ex(screen_buf, font, "Press up/down to increase/decrease the number of cars", SCREEN_W/2, line, MCOL, BKG);
  line += 15;
  textout_centre_ex(screen_buf, font, "Press right/left to increase/decrease the number of laps", SCREEN_W/2, line, MCOL, BKG);
  line += 15;
  textout_centre_ex(screen_buf, font, "Press 1/2/3 to select track", SCREEN_W/2, line, MCOL, BKG);
  line += 15;
  sprintf(string_buffer, "%i cars competing", n_cars);
  textout_centre_ex(screen_buf, font, string_buffer, SCREEN_W/2, line, MCOL, BKG);
  line += 15;
  sprintf(string_buffer, "%i laps to win", n_laps);
  textout_centre_ex(screen_buf, font, string_buffer, SCREEN_W/2, line, MCOL, BKG);
  line += 20;
  rectfill(screen_buf, SCREEN_W * (3 * (active_track) + 1)/10 - 10, line - 10, SCREEN_W * (3 * (active_track) + 1)/10 + SCREEN_W/5 + 10, line + SCREEN_H/5 + 10, MCOL);
  for(i = 0; i < N_TRACKS; i++){
    stretch_sprite(screen_buf, track[i], SCREEN_W * (3 * i + 1)/10, line, SCREEN_W/5, SCREEN_H/5);
  }
  line += 15 + SCREEN_H/5;
  for(i = 0; i < N_TRACKS; i++){
    sprintf(string_buffer, "Track %i", i + 1);
    textout_centre_ex(screen_buf, font, string_buffer, SCREEN_W * (3 * i + 2)/10, line, MCOL, BKG);
  }
  line += 20;
  if (cars[0].position == 1) textout_centre_ex(screen_buf, font, "You won!", SCREEN_W/2, line, MCOL, BKG);
  if (cars[0].position > 1) textout_centre_ex(screen_buf, font, "You lost!", SCREEN_W/2, line, MCOL, BKG);
  for(i = 0; cars[i].position != 0; i++){
    sprintf(string_buffer, "%i: car %i", cars[i].position, i + 1);
    textout_centre_ex(screen_buf, font, string_buffer, SCREEN_W/2, line + cars[i].position*15, MCOL, BKG);
  }

	if (measure)
	  textout_centre_ex(screen_buf, font, "Press E for data recording: now active", SCREEN_W/2, SCREEN_H - 15, MCOL, BKG);
	else
	  textout_centre_ex(screen_buf, font, "Press E for data recording: now inactive", SCREEN_W/2, SCREEN_H - 15, MCOL, BKG);

  return;
}

//------------------------------------------------------------------------------
// Computes acceleration from keyboard commands, then it updates the user car
// accordingly
// Space for accelerator, down key for brake
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void keyboard_acc(){
  if (key[KEY_SPACE]){
    pthread_mutex_lock(&car_mux);
    cars[0].acc.module = 1;
    pthread_mutex_unlock(&car_mux);
  }
  else{
    if (key[KEY_DOWN] && !cars[0].acc.module){	// Mutex not needed in the condition because acc is only read and cannot be modified anywhere else
      pthread_mutex_lock(&car_mux);
      cars[0].acc.module = -1;
      pthread_mutex_unlock(&car_mux);
    }
    else{
      pthread_mutex_lock(&car_mux);
      cars[0].acc.module = 0;
      pthread_mutex_unlock(&car_mux);
    }
  }
  return;
}

//------------------------------------------------------------------------------
// Compute steer from keyboard commands, then it updates the user car
// accordingly
// Right/left to steer clockwise/counterclockwise
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void keyboard_steer(){
  if (key[KEY_RIGHT]){
    pthread_mutex_lock(&car_mux);
    cars[0].acc.orientation = 1;
    pthread_mutex_unlock(&car_mux);
  }
  else{
    pthread_mutex_lock(&car_mux);
    if (key[KEY_LEFT])
      cars[0].acc.orientation = -1;
    else
      cars[0].acc.orientation = 0;
    }
    pthread_mutex_unlock(&car_mux);
  return;
}

//------------------------------------------------------------------------------
// Displays startup screen for one second. Then it changes
// program state to MENU.
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void screen_startup(){
  clear_to_color(screen, 0);
  textout_centre_ex(screen, font, "Welcome", SCREEN_W/2, SCREEN_H/2, MCOL, BKG);
  struct	timespec dt;
  dt.tv_sec = 1;
  dt.tv_nsec = 0;
  clock_nanosleep(CLOCK_MONOTONIC, 0, &dt, NULL);
  pthread_mutex_lock(&state_mux);
  program_state = MENU;
  pthread_mutex_unlock(&state_mux);
  return;
}

//------------------------------------------------------------------------------
// Handle keyboard commands in MENU state
//
// inputs:
// None
//
// returns:
// an integer indicating if a key was pressed or not
//------------------------------------------------------------------------------

int keyboard_menu(){
  // Pressing S makes the race start
  if (key[KEY_S]){
    reset_cars();
    pthread_mutex_lock(&state_mux);
    program_state = RACE;
		zero_misses();
    pthread_mutex_unlock(&state_mux);
  }

  // Pressing D switches to drawing mode
  if (key[KEY_D]){
    show_mouse(screen);
    pthread_mutex_lock(&new_track);
    drawn_track = create_bitmap(SCREEN_W, SCREEN_H);
    clear_to_color(drawn_track, GRASS);
    pthread_mutex_unlock(&new_track);
    pthread_mutex_lock(&state_mux);
    program_state = DRAW;
    pthread_mutex_unlock(&state_mux);
  }

  // Pressing A switches between autopilot/driven mode
  if (key[KEY_A]){
    autopilot = 1 - autopilot;
    return YES;
  }


	if (key[KEY_E]){
		measure = 1 - measure;
		return YES;
	}

  // Up/Down to change the number of cars
  if (key[KEY_DOWN]){
    if (n_cars > 1) n_cars -= 1;
    return YES;
  }

  if (key[KEY_UP]){
    if (n_cars < MAX_CARS) n_cars += 1;
    return YES;
  }

  // Right/Left to increase/decrease number of laps
  if (key[KEY_LEFT]){
    if (n_laps > 1) n_laps -= 1;
    return YES;
  }

  if (key[KEY_RIGHT]){
    if (n_laps < MAX_LAPS) n_laps += 1;
    return YES;
  }

  // 1/2/3 to select active track
  if (key[KEY_1]){
    active_track = 0;
    return YES;
  }

  if (key[KEY_2]){
    active_track = 1;
    return YES;
  }

  if (key[KEY_3]){
    active_track = 2;
    return YES;
  }

  return NO;
}

//------------------------------------------------------------------------------
// Save the drawn track to corresponding file as indicated by input i
//
// inputs:
// i: number of the file to be overwritten
//
// returns:
// None
//------------------------------------------------------------------------------

void save_drawn(int i){
    show_mouse(NULL);
    PALETTE pal;
    get_palette(pal);
    switch(i){
      case 1:
        save_bitmap("../img/tracks/track_1.bmp", drawn_track, pal);
        break;
      case 2:
        save_bitmap("../img/tracks/track_2.bmp", drawn_track, pal);
        break;
      case 3:
        save_bitmap("../img/tracks/track_3.bmp", drawn_track, pal);
        break;
    }
    pthread_mutex_lock(&state_mux);
    program_state = MENU;
    pthread_mutex_unlock(&state_mux);
    track[0] = load_bitmap("../img/tracks/track_1.bmp", NULL);
    switch(i){
      case 1:
        track[0] = load_bitmap("../img/tracks/track_1.bmp", NULL);
        break;
      case 2:
        track[1] = load_bitmap("../img/tracks/track_2.bmp", NULL);
        break;
      case 3:
        track[2] = load_bitmap("../img/tracks/track_3.bmp", NULL);
        break;
    }
    return;
}

//------------------------------------------------------------------------------
// Draw the track according to the commands coming from the mouse: if pressing
// the left key it draws a circle of radius "radius" on the track
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void draw_track(){
  if (mouse_b & 1) {
    pthread_mutex_lock(&new_track);
    if (drawn_color == ARRIVE)
      circlefill(drawn_track, mouse_x, mouse_y, 12, drawn_color);
    else
      circlefill(drawn_track, mouse_x, mouse_y, radius, drawn_color);
    pthread_mutex_unlock(&new_track);
  }
  return;
}

//------------------------------------------------------------------------------
// Controls car position and if needed increases lap number
//
// inputs:
// i: number of the car to be checked
//
// returns:
// None
//------------------------------------------------------------------------------

void add_lap(int i){
  int c, x, y;
  pthread_mutex_lock(&car_mux);
  x = cars[i].x;
  y = cars[i].y;
  pthread_mutex_unlock(&car_mux);
  c = getpixel(track[active_track], x, y);
  if (c == ARRIVE){
    if (!added_lap[i]){
      laps[i] += 1;
      if (laps[i] == n_laps) cars[i].position = n_arrived++;
    }
    added_lap[i] = 1;
  }
  else
    added_lap[i] = 0;
  return;
}

//------------------------------------------------------------------------------
// Read only the front sensor of a car looking for the closer obstacle
//
// inputs:
// x0: car position on x axis
// y0: car position on y axis
// alpha: angle of orientation of the car
//
// returns:
// ret: it is a fron_lidar_measure structure which contains the distance
// from the closer obstacle and an integer indicating wether it is a cas or not
//------------------------------------------------------------------------------

front_lidar_measure read_front_sensor(int x0, int y0, float alpha){
	int c;
  int	x, y;
  int	d = SMIN;
  front_lidar_measure ret;

  do {
    x = x0 + d*cos(alpha);
    y = y0 + d*sin(alpha);
    c = getpixel(screen, x, y);
    d = d + SSTEP;
  } while ((d <= SMAX) && ((c == ROAD)||(c==ARRIVE)));
  ret.d = d;
  if (d <= SMAX && c != GRASS)
    ret.is_car = YES;
  else
    ret.is_car = NO;

  return ret;
}


//------------------------------------------------------------------------------
// Checks if two cars are close enough to possibly be in collision. The order
// of the cars is irrelevant
//
// inputs:
// k: number of the first car
// i: number of the second car
//
// returns:
// an integer indicating if the cars are close enough to collide
//------------------------------------------------------------------------------

int are_close(int k, int i){
  int x1, x2, y1, y2, dist;
  pthread_mutex_lock(&car_mux);
  x1 = cars[i].x;
  y1 = cars[i].y;
  x2 = cars[k].x;
  y2 = cars[k].y;
  pthread_mutex_unlock(&car_mux);
  dist = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
  if (dist < SMIN + 1) return YES;
  return NO;
}


//------------------------------------------------------------------------------
// Checks if there actually is a collision between cars k and i and handles it.
// In this case the order is not irrelevant, in fact only the second car is
// affected by the collision.
//
// inputs:
// k: number of the first car
// i: number of the second car
//
// returns:
// None
//------------------------------------------------------------------------------

void collide(int k, int i){
  float xa0, xb0, ya0, yb0;
  float xa1, xa2, xb1, xb2, ya1, ya2, yb1, yb2;
  float or_a, or_b;
  float d1, d2, d3, d4;

  pthread_mutex_lock(&car_mux);
  xa0 = cars[i].x;
  ya0 = cars[i].y;
  xb0 = cars[k].x;
  yb0 = cars[k].y;
  or_a = cars[i].vel.orientation * 2 * M_PI;
  or_b = cars[k].vel.orientation * 2 * M_PI;
  pthread_mutex_unlock(&car_mux);

  xa1 = + cos(or_a) * 4;
  ya1 = + sin(or_a) * 4;
  xa2 = - cos(or_a) * 4;
  ya2 = - sin(or_a) * 4;
  xb1 = + cos(or_b) * 4;
  yb1 = + sin(or_b) * 4;
  xb2 = - cos(or_b) * 4;
  yb2 = - sin(or_b) * 4;
  d1 = sqrt(pow((xa1 + xa0 - xb1 - xb0), 2) + pow((ya1 + ya0 - yb1 - yb0), 2));
  d2 = sqrt(pow((xa1 + xa0 - xb2 - xb0), 2) + pow((ya1 + ya0 - yb2 - yb0), 2));
  d3 = sqrt(pow((xa2 + xa0 - xb1 - xb0), 2) + pow((ya2 + ya0 - yb1 - yb0), 2));
  d4 = sqrt(pow((xa2 + xa0 - xb2 - xb0), 2) + pow((ya2 + ya0 - yb2 - yb0), 2));

  if(d1 < 12){
    pthread_mutex_lock(&car_mux);
    cars[i].x = xb0 + (xa1 + xa0 - xb1 - xb0)*(12)/d1 + xb1 - xa1;
    cars[i].y = yb0 + (ya1 + ya0 - yb1 - yb0)*(12)/d1 + yb1 - ya1; //Riprovare
    pthread_mutex_unlock(&car_mux);
  }

  if(d2 < 12){
    pthread_mutex_lock(&car_mux);
    cars[i].x = xb0 + (xa1 + xa0 - xb2 - xb0)*(12)/d2 + xb2 - xa1;
    cars[i].y = yb0 + (ya1 + ya0 - yb2 - yb0)*(12)/d2 + yb2 - ya1; //Riprovare
    pthread_mutex_unlock(&car_mux);
  }

  if(d3 < 12){
    pthread_mutex_lock(&car_mux);
    cars[i].x = xb0 + (xa2 + xa0 - xb1 - xb0)*(12)/d3 + xb1 - xa2;
    cars[i].y = yb0 + (ya2 + ya0 - yb1 - yb0)*(12)/d3 + yb1 - ya2; //Riprovare
    pthread_mutex_unlock(&car_mux);
  }

  if(d4 < 12){
    pthread_mutex_lock(&car_mux);
    cars[i].x = xb0 + (xa2 + xa0 - xb2 - xb0)*(12)/d4 + xb2 - xa2;
    cars[i].y = yb0 + (ya2 + ya0 - yb2 - yb0)*(12)/d4 + yb2 - ya2; //Riprovare
    pthread_mutex_unlock(&car_mux);
  }

  return;
}

//------------------------------------------------------------------------------
// Writes onto the screen buffer when in DRAW mode
//
// inputs:
// *screen_buf: pointer to the screen buffer
//
// returns:
// None
//------------------------------------------------------------------------------

void draw_mode_text(BITMAP *screen_buf){

textout_centre_ex(screen_buf, font, "You are in Drawing mode", SCREEN_W/2, SCREEN_H/2, MCOL, BKG);
textout_centre_ex(screen_buf, font, "1/2/3: Overwrite track 1/2/3    M: return to menu without saving", SCREEN_W/2, SCREEN_H/2 + 15, MCOL, BKG);
textout_centre_ex(screen_buf, font, "T: Draw track    A: Draw arrive line    E: Erase", SCREEN_W/2, SCREEN_H/2 + 30, MCOL, BKG);
switch (drawn_color){
	case GRASS:
		sprintf(string_buffer, "Currently erasing, Width: %i", radius);
		break;
	case ARRIVE:
		sprintf(string_buffer, "Currently drawing arrive line, Width: %i", radius);
		break;
	case ROAD:
		sprintf(string_buffer, "Currently drawing track, Width: %i", radius);
		break;
}
textout_centre_ex(screen_buf, font, string_buffer, SCREEN_W/2, SCREEN_H/2 + 45, MCOL, BKG);
}

//------------------------------------------------------------------------------
// Command interpreter for race mode
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void keyboard_race(){
	if (key[KEY_P]){
		pthread_mutex_lock(&state_mux);
		program_state = PAUSE;
		pthread_mutex_unlock(&state_mux);
	}
	if (key[KEY_Q]){
		pthread_mutex_lock(&state_mux);
		program_state = MENU;
		pthread_mutex_unlock(&state_mux);
	}
	if (autopilot == NO){
		keyboard_acc();
		keyboard_steer();
	}
	return;
}

//------------------------------------------------------------------------------
// Command interpreter for draw mode
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void keyboard_draw(){

	if (key[KEY_M]){
		show_mouse(NULL);
		pthread_mutex_lock(&state_mux);
		program_state = MENU;
		pthread_mutex_unlock(&state_mux);
	}

	if (key[KEY_1]) save_drawn(1);
	if (key[KEY_2]) save_drawn(2);
	if (key[KEY_3]) save_drawn(3);

	if (key[KEY_A]) drawn_color = ARRIVE;

	if (key[KEY_T]) drawn_color = ROAD;

	if (key[KEY_E]) drawn_color = GRASS;

	if (key[KEY_UP] && radius < 81)  radius += 1;

	if (key[KEY_DOWN] && radius > 1)  radius -= 1;
	return;
}

//------------------------------------------------------------------------------
// Command interpreter for pause state
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void keyboard_pause(){
	if (key[KEY_P]){
		pthread_mutex_lock(&state_mux);
		program_state = RACE;
		pthread_mutex_unlock(&state_mux);
	}
	if (key[KEY_Q]){
		pthread_mutex_lock(&state_mux);
		program_state = MENU;
		pthread_mutex_unlock(&state_mux);
	}
	return;
}

//------------------------------------------------------------------------------
// Reads the velocity of the user car and prints it to a csv file
//
// inputs:
// *file: pointer to the file to be written
//
// returns:
// None
//------------------------------------------------------------------------------

void measure_velocity(FILE *file){
	float	module, orientation;
  pthread_mutex_lock(&car_mux);
	module = cars[0].vel.module;
	orientation = cars[0].vel.orientation;
  pthread_mutex_unlock(&car_mux);
	fprintf(file, "%f; %f \n", module, orientation);
	return;
}

//------------------------------------------------------------------------------
// Counts the number of deadline misses occured until now
//
// inputs:
// None
//
// returns:
// count: the number of misses counted
//------------------------------------------------------------------------------

int count_misses(){
	int count = 0;
	count += gtask_par.dmiss;
	count += ktask_par.dmiss;
	count += ctask_par.dmiss;
	count += stask_par.dmiss;
	count += mtask_par.dmiss;
	count += etask_par.dmiss;
	return count;
}

//------------------------------------------------------------------------------
// Reset the number of deadline misses
//
// inputs:
// None
//
// returns:
// None
//------------------------------------------------------------------------------

void zero_misses(){
	gtask_par.dmiss = 0;
	ktask_par.dmiss = 0;
	ctask_par.dmiss = 0;
	stask_par.dmiss = 0;
	mtask_par.dmiss = 0;
	etask_par.dmiss = 0;
}

//----------------------------------------------------------------
// Tasks
//----------------------------------------------------------------


//------------------------------------------------------------------------------
// Car task: during the race it periodically integrates the indicated
// acceleration and the corresponding velocity
//------------------------------------------------------------------------------

void *car_task(void *arg){
  struct	task_par *tp;
  tp = (struct task_par *)arg;
  set_period(tp);
  int	i = 0;
  while(1){
    if(program_state == RACE){
        compute_position(0);
        compute_speed(0);
        add_lap(0);
        if (autopilot) compute_acc(0);
        for(i = 1; i < n_cars; i++){
            compute_position(i);
            compute_speed(i);
            compute_acc(i);
            add_lap(i);
        }
        handle_collisions();
        if (n_arrived == n_cars + 1){
          pthread_mutex_lock(&state_mux);
          program_state = MENU;
          pthread_mutex_unlock(&state_mux);
        };
      }
    if (deadline_miss(tp)){
      set_period(tp);
			ctask_par.dmiss++;
    }
		if (!program_state) return NULL;
    wait_for_period(tp);
  }
}

//------------------------------------------------------------------------------
// Sensor task: it periodically evaluates the desiderd velocity for each car,
// except for the user car if the autopilot is off.
// This is the task that implements the autopilot.
//------------------------------------------------------------------------------

void *sensor_task(void *arg){
  struct	task_par *tp;
  tp = (struct task_par *)arg;
	int i = 0;
  set_period(tp);
  velocity  tmp_des;
	i = 0;

  while(1){

    if (program_state == RACE){
      for(i = 0; i < n_cars; i++){
        tmp_des = scan_car(i);
				pthread_mutex_lock(&sensor_mux);
				sensors[i] = tmp_des;
				pthread_mutex_unlock(&sensor_mux);
				//printf("%f \n", tmp_des.orientation);
      }
    }
    if (deadline_miss(tp)){
      set_period(tp);
			stask_par.dmiss++;
    }
		if (!program_state) return NULL;
    wait_for_period(tp);
  }
}

//------------------------------------------------------------------------------
// Keyboard task: periodically interprets the user inputs. When ESC is pressed
// it changes the program state to END so that every task can know that is
// needs to stop.
//------------------------------------------------------------------------------

void *keyboard_task(void *arg){
  struct	task_par *tp;
  tp = (struct task_par *)arg;
  set_period(tp);
  int	pressed = YES;
	while(!key[KEY_ESC]){
    if (pressed){
      wait_for_period(tp);
      pressed = NO;
      continue;
    }
    switch(program_state){

      case MENU:
        pressed = keyboard_menu();
        break;

      case RACE:
				keyboard_race();
        break;

      case PAUSE:
				keyboard_pause();
        break;

      case DRAW:
				keyboard_draw();
				break;

      default:
        break;
      }

    if (deadline_miss(tp)){
      set_period(tp);
			ktask_par.dmiss++;
    }
    wait_for_period(tp);
  }
	pthread_mutex_lock(&state_mux);
	program_state = END;
	pthread_mutex_unlock(&state_mux);
}

//------------------------------------------------------------------------------
// Graphic task: periodically draw the screen to be visualized, depending on the
// state of the program
//------------------------------------------------------------------------------

void *graphic_task(void *arg){
  struct	task_par *tp;
  tp = (struct task_par *)arg;
  BITMAP	*screen_buf;
  BITMAP	*car_bmp[MAX_CARS];
  int	i = 0;

  // creating screen buffer
	screen_buf = create_bitmap(SCREEN_W, SCREEN_H);

	// loading tracks
  track[0] = load_bitmap("../img/tracks/track_1.bmp", NULL);
  track[1] = load_bitmap("../img/tracks/track_2.bmp", NULL);
  track[2] = load_bitmap("../img/tracks/track_3.bmp", NULL);

  // car bitmaps are loaded with the right color
  for (i = 0; i < MAX_CARS; i++){
    car_bmp[i] = load_bitmap("../img/car.bmp", NULL);
    recolor(car_bmp[i], i);
  }
  if (!car_bmp) printf("Error loadig car bitmap");

  set_period(tp);

	while(1){

    switch(program_state){

      case STARTUP:
        screen_startup();
        set_period(tp);
        break;

      case MENU:
        draw_menu(screen_buf);
        blit(screen_buf, screen, 0, 0, 0, 0, screen_buf->w, screen_buf->h);
        break;

      case RACE:
        blit(track[active_track], screen_buf, 0, 0, 0, 0, screen_buf->w, screen_buf->h);

        for(i = 0; i < n_cars; i++) draw_car(i, screen_buf, car_bmp[i]);

				blit(screen_buf, screen, 0, 0, 0, 0, screen_buf->w, screen_buf->h);
        break;

      case PAUSE:
        textout_centre_ex(screen, font, "Pause", SCREEN_W/2, SCREEN_H/2, MCOL, BKG);
        break;

      case DRAW:
        pthread_mutex_lock(&new_track);
        blit(drawn_track, screen_buf, 0, 0, 0, 0, screen_buf->w, screen_buf->h);
        pthread_mutex_unlock(&new_track);

				draw_mode_text(screen_buf);

				scare_mouse();
        blit(screen_buf, screen, 0, 0, 0, 0, screen_buf->w, screen_buf->h);
        unscare_mouse();
        break;

        default:
        break;
      }
		if (deadline_miss(tp)){
      set_period(tp);
			gtask_par.dmiss++;
    }
		if (!program_state) return NULL;
    wait_for_period(tp);
  }
}

//------------------------------------------------------------------------------
// Mouse task: used to draw the new track. When in draw mode, it periodically
// reads the mouse input and if needed updates the drawn track
//------------------------------------------------------------------------------

void *mouse_task(void *arg){
  struct	task_par *tp;
  tp = (struct task_par *)arg;
  set_period(tp);
  while(1){

    if (program_state == DRAW) draw_track();

    if (deadline_miss(tp)){
      set_period(tp);
			mtask_par.dmiss++;
    }
		if (!program_state) return NULL;
    wait_for_period(tp);
  }
}

//------------------------------------------------------------------------------
// Experiment task: when the experiment is set to run, it periodically saves
// the user car position in the first 20 seconds of race, then it saves the
// number of deadline misses occured in the same time period
//------------------------------------------------------------------------------

void *experiment_task(void *arg){
  struct	task_par *tp;
  tp = (struct task_par *)arg;
  set_period(tp);
	int d_misses = 0;
	int i = 0;
	int loop;
	FILE *data;
	data = fopen("../data.csv","a");
  while(1){
		if (program_state != RACE) {
			i = 0;
		}
    if (program_state == RACE) {
			if (measure){
					if (i < EXPERIMENT_DURATION){
						measure_velocity(data);
						i++;
					}
					else{
						d_misses = count_misses();
						measure = 0;
						printf("%i \n", d_misses);
					}
				}
		}
    if (deadline_miss(tp)){
      set_period(tp);
    }
		if (!program_state) return NULL;
    wait_for_period(tp);
  }
}
