/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/mav_course_customproject1/mav_course_customproject1.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

 #include "mav_customproject1.h"
 #include "firmwares/rotorcraft/guidance/guidance_h.h"
 #include "generated/airframe.h"
 #include "state.h"
 #include "modules/core/abi.h"
 #include <stdio.h>
 #include <time.h>
 // EVADER - START
 // include addition ones
 #include <math.h>
 // EVADER - END

 #define ORANGE_AVOIDER_VERBOSE TRUE
 
 #define PRINT(string,...) fprintf(stderr, "[mav_course_customproject1->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
 #if ORANGE_AVOIDER_VERBOSE
 #define VERBOSE_PRINT PRINT
 #else
 #define VERBOSE_PRINT(...)
 #endif
 
 // EVADER - START
 void evader_init(void);
 void update_evader_position(void);
 float purePursuit2d_compute_desired_heading(float evader_x, float evader_y, float pursuer_x, float pursuer_y);
 float purePursuit2d_controller(float error);
 float purePursuit2d_compute_heading_command(float heading_ref);
 // EVADER - END
 uint8_t chooseRandomIncrementAvoidance(void);
 
 enum navigation_state_t {
   SAFE,
   OBSTACLE_FOUND,
   SEARCH_FOR_SAFE_HEADING,
   OUT_OF_BOUNDS,
   REENTER_ARENA
 };
 
 // define settings
 float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
 float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
 float oag_max_speed = 0.5f;               // max flight speed [m/s]
 float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
 
 // define and initialise global variables
 enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
 int32_t color_count = 0;                // orange color count from color filter for obstacle detection
 int32_t floor_count = 0;                // green color count from color filter for floor detection
 int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
 float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
 int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
 
 const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free
 
 // EVADER - INITIALIZE GLOBAL VARIABLES
 // Evader properties
 static float evader_x = 0.0f;
 static float evader_y = 0.0f;
 static float evader_speed = 0.2f; // Adjust as needed [m/s]
 static float evader_heading = 0.0f;
 static float distance_to_evader = 10.0f; // set initial distance as anything but zero

 static float max_heading_rate = RadOfDeg(15.0f); // max heading rate for the pp controller - equal to some other found later in code
 static float lookahead_distance = 1.0f; //
 // Arena Properties
 static float arena_size = 4.0f; // Radius!! of circular arena in meters. Adjust as needed - eijjah says its 10x10x10


 // EVADER - END

 // This call back will be used to receive the color count from the orange detector
 #ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
 #error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
 #error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
 #endif
 static abi_event color_detection_ev;
 static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                                int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                                int32_t quality, int16_t __attribute__((unused)) extra)
 {
   color_count = quality;
 }
 
 #ifndef FLOOR_VISUAL_DETECTION_ID
 #error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
 #error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
 #endif
 static abi_event floor_detection_ev;
 static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                                int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                                int32_t quality, int16_t __attribute__((unused)) extra)
 {
   floor_count = quality;
   floor_centroid = pixel_y;
 }
 // EVADER - START
void evader_init(void) {
  // Seed the random number generator (if not already seeded)
  static bool seeded = false;
  if (!seeded) {
    srand(time(NULL));
    seeded = true;
  }

  // Generate a random angle and radius
  float random_angle = (float)(rand() % 360) / 180.0 * M_PI; // Random angle in radians
  float random_radius = (float)rand() / RAND_MAX * arena_size; // Random radius within arena

  // Calculate x and y coordinates
  evader_x = random_radius * cosf(random_angle);
  evader_y = random_radius * sinf(random_angle);

  // Initialize evader heading to a random direction
  evader_heading = (float)(rand() % 360) / 180.0 * M_PI;
}

void update_evader_position(void) {
  // Randomly change heading
  float heading_change = (float)(rand() % 60 - 30) / 180.0 * M_PI; // +/- 30 degrees in radians
  evader_heading += heading_change;

  // Keep heading within 0 to 2*PI
  while (evader_heading > 2 * M_PI) evader_heading -= 2 * M_PI;
  while (evader_heading < 0) evader_heading += 2 * M_PI;

  // Update position
  evader_x += evader_speed * cosf(evader_heading) * PERIODIC_FREQUENCY;
  evader_y += evader_speed * sinf(evader_heading) * PERIODIC_FREQUENCY;

  // Keep evader within arena bounds
  float distance_from_center = sqrtf(evader_x * evader_x + evader_y * evader_y);
  if (distance_from_center > arena_size) {
    // Project back onto the circle
    evader_x = evader_x / distance_from_center * arena_size;
    evader_y = evader_y / distance_from_center * arena_size;
  }
}


float purePursuit2d_compute_desired_heading(float evader_x, float evader_y, float pursuer_x, float pursuer_y) {
  // 1. Calculate the vector from the aircraft to the evader
  float delta_x = evader_x - pursuer_x;
  float delta_y = evader_y - pursuer_y;

  // 2. Calculate the distance to the evader (optional, might be useful later)
  distance_to_evader = sqrtf(delta_x * delta_x + delta_y * delta_y);

  // 3. Calculate the lookahead point
  float lookahead_x, lookahead_y;

  // If the evader is closer than the lookahead distance, just go straight to
  // the evader
  if (distance_to_evader <= lookahead_distance) {
    lookahead_x = evader_x;
    lookahead_y = evader_y;
  } else {
    // Calculate the normalized vector from pursuer to evader
    float normalized_delta_x = delta_x / distance_to_evader;
    float normalized_delta_y = delta_y / distance_to_evader;

    // Calculate the lookahead point coordinates
    lookahead_x = pursuer_x + normalized_delta_x * lookahead_distance;
    lookahead_y = pursuer_y + normalized_delta_y * lookahead_distance;
  }

  // 4. Calculate the desired heading to the lookahead point
  float desired_heading = atan2f(lookahead_y - pursuer_y, lookahead_x - pursuer_x);

  // Make sure the heading is between 0 and 2*PI
  while (desired_heading > 2 * M_PI)
    desired_heading -= 2 * M_PI;
  while (desired_heading < 0)
    desired_heading += 2 * M_PI;

  return desired_heading;
}


float purePursuit2d_controller(float error){
  // aunomous proportional controller
  //float max_cmd = 

  float cmd = error;
  return cmd;

}

float purePursuit2d_compute_heading_command(float heading_ref){

  float heading_current = stateGetNedToBodyEulers_f()->psi;
  float heading_error = heading_ref - heading_current;
  float heading_rate_cmd = purePursuit2d_controller(heading_error);

  Bound(heading_rate_cmd, -max_heading_rate, max_heading_rate);

  return heading_rate_cmd;
}

 // EVADER - END


 /*
  * Initialisation function
  */
 void mav_customproject1_init(void) {

   // Initialise random values
   srand(time(NULL));
   chooseRandomIncrementAvoidance();
    
   // EVADER - Initialise evader position
   evader_init();   
   // bind our colorfilter callbacks to receive the color filter outputs
   AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
   AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
 }
 
 /*
  * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
  */
 void mav_customproject1_periodic(void)
 {
   // Only run the mudule if we are in the correct flight mode
   if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
     navigation_state = SEARCH_FOR_SAFE_HEADING;
     obstacle_free_confidence = 0;
     return;
   }
 
   update_evader_position(); // EVADER UPDATE POSITION
   //VERBOSE_PRINT("EVADER - x: %f  y: %f", evader_x, evader_y);

   // compute current color thresholds
   int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
   int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
   float floor_centroid_frac = floor_centroid / (float )front_camera.output_size.h / 2.f;
   
   VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
   VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
   VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);
 
   // update our safe confidence using color threshold
   if(color_count < color_count_threshold){
     obstacle_free_confidence++;
   } else {
     obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
   }
 
   // bound obstacle_free_confidence
   Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
 
   float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);
 
   switch (navigation_state){
     case SAFE:
       if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
         navigation_state = OUT_OF_BOUNDS;
       } else if (obstacle_free_confidence == 0){
         navigation_state = OBSTACLE_FOUND;
       } else {
         // else clause - safe to pursue alternative objectives
         guidance_h_set_body_vel(speed_sp, 0); // legacy code? 

         // EVADER - START
         // only in this clause do we compute PP commands, else the comp would be wasted
         float drone_x = stateGetPositionNed_f()->x;
         float drone_y = stateGetPositionNed_f()->y; // TODO not double call requried

         float desired_heading_command = purePursuit2d_compute_desired_heading(evader_x, evader_y, drone_x, drone_y);

         
         float heading_rate_command = purePursuit2d_compute_heading_command(desired_heading_command);
         guidance_h_set_heading_rate(heading_rate_command);
 
         VERBOSE_PRINT("EVADER - x: %f  y: %f distance: %f\n", evader_x, evader_y, distance_to_evader);

         // EVADER - STOP
       }
 
       break;
     case OBSTACLE_FOUND:
       // stop
       guidance_h_set_body_vel(0, 0);
 
       // randomly select new search direction
       chooseRandomIncrementAvoidance();
 
       navigation_state = SEARCH_FOR_SAFE_HEADING;
 
       break;
     case SEARCH_FOR_SAFE_HEADING:
       guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);
 
       // make sure we have a couple of good readings before declaring the way safe
       if (obstacle_free_confidence >= 2){
         guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
         navigation_state = SAFE;
       }
       break;
     case OUT_OF_BOUNDS:
       // stop
       guidance_h_set_body_vel(0, 0);
 
       // start turn back into arena
       guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));
 
       navigation_state = REENTER_ARENA;
 
       break;
     case REENTER_ARENA:
       // force floor center to opposite side of turn to head back into arena
       if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
         // return to heading mode
         guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
 
         // reset safe counter
         obstacle_free_confidence = 0;
 
         // ensure direction is safe before continuing
         navigation_state = SAFE;
       }
       break;
     default:
       break;
   }
   return;
 }
 
 /*
  * Sets the variable 'incrementForAvoidance' randomly positive/negative
  */
 uint8_t chooseRandomIncrementAvoidance(void)
 {
   // Randomly choose CW or CCW avoiding direction
   if (rand() % 2 == 0) {
     avoidance_heading_direction = 1.f;
     VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
   } else {
     avoidance_heading_direction = -1.f;
     VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
   }
   return false;
 }
 