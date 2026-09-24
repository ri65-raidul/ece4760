
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/sync.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(480))
#define hitTop(b) (b<int2fix15(0))
#define hitLeft(a) (a<int2fix15(0))
#define hitRight(a) (a>int2fix15(640))

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

const fix15 BALL_RAD = int2fix15(15);
const fix15 PEG_RAD = int2fix15(15);

const fix15 BOUNCINESS = float2fix15(0.5);
const fix15 GRAVITY = float2fix15(0.37);

// Create a semaphore
semaphore_t draw_semaphore ;

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(0) ;
  // Choose left or right
  // if (direction) *vx = int2fix15(3) ;
  // else *vx = int2fix15(-3) ;
  // Moving down
  *vx = int2fix15(0);
  *vy = int2fix15(1) ;
}

void spawnPeg(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
  *vx = int2fix15(0);
  *vy = int2fix15(0) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y - 15)) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    spawnBoid(x, y, vx, vy, 0);
  } 
  if (hitRight(*x + 15)) {
    *vx = (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  if (hitLeft(*x - 15)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  } 

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

// Detect wallstrikes, update velocity and position
void BouncePeg(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  fix15 sum_radius = BALL_RAD + PEG_RAD;

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
  
  fix15 dx = *x - int2fix15(320);
  fix15 dy = *y - int2fix15(240);

  // printf("x, %d\n" , (int)(*x));
  // printf("y, %d\n" , (int)(*y));
  // printf("dx, %d\n" , (int)(dx));
  // printf("dy, %d\n" , (int)(dy));
  // printf("rad, %d\n" , (int)(sum_radius));


  if(absfix15(dx) < (sum_radius) && absfix15(dy) < (sum_radius) ) {
    
    // float dx_f = fix2float15(dx);
    // float dy_f = fix2float15(dy);
    // fix15 dist = float2fix15(sqrtf((dx_f * dx_f) + (dy_f * dy_f)));

    fix15 dist = float2fix15(sqrt( fix2float15(multfix15(dx, dx)) + fix2float15(multfix15(dy, dy)) ));
    //printf("Dist: %d\n", (int)(dist >> 15));

    if(dist < (sum_radius)){
      printf("Enters if");
      //Generate normal vector
      fix15 normal_x = divfix(dx, dist);
      fix15 normal_y = divfix(dy, dist);

      // Collision physics
      fix15 intermediate_term = multfix15(-2, (multfix15(normal_x, *vx) + multfix15(normal_y, *vy)));

      // Teleport it outside the collison distance with the peg
      *x = int2fix15(320) + multfix15(normal_x, (sum_radius + int2fix15(1)));
      *y = int2fix15(240) + multfix15(normal_y, (sum_radius + int2fix15(1)));

      // Update its velocity
      *vx = *vx + multfix15(normal_x, intermediate_term);
      *vy = *vy + multfix15(normal_y, intermediate_term);

      // Make a sound

      // Remove some energy from the ball
      *vx = multfix15(BOUNCINESS, *vx);
      *vy = multfix15(BOUNCINESS, *vy); 

      // wallsAndEdges(x, y, vx, vy);

      // Apply gravity
      *vy = *vy + GRAVITY;

    }

  }
  
}



// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.4\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 16)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Spawn a boid
    spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);

    while(1) {
      // Wait for the signal that the buffer's changed
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      // Clear the buffer
      clearLowFrame(0, BLACK);
      // Signal core 1 that it can start drawing
      PT_SEM_SDK_SIGNAL(pt, &draw_semaphore) ;

      BouncePeg(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy);
      // update boid's position and velocity
      wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy) ;
      // draw the boid at its new position
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), 15, color); 
      
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Spawn a boid
    spawnPeg(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy);

    while(1) {
      // Wait for the signal from core 0
      PT_SEM_SDK_WAIT(pt, &draw_semaphore) ;
      
      fillCircle(fix2int15(boid1_x), fix2int15(boid1_y), 15, color); 
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  set_sys_clock_khz(150000, true) ;
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  sem_init(&draw_semaphore, 0, 1) ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  //pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
