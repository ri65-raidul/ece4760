
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
#include "hardware/gpio.h"
#include "hardware/vreg.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

#include <stdfix.h>

#include "hardware/spi.h"



// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15 = 32768
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(350))
#define hitTop(b) (b<int2fix15(0))
#define hitLeft(a) (a<int2fix15(0))
#define hitRight(a) (a>int2fix15(640))

// uS per frame
#define FRAME_RATE 16500

//DMA
// Number of samples per period in sine table
#define sine_table_size 256

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

volatile int data_chan;
volatile int ctrl_chan;

// array for the histogram bins
volatile int bins[15]; // only 15 gaps between pegs for a row of 16 pegs
volatile int bins_max;

// Global counters
volatile long int active_balls = 14100;
volatile long int prev_act_balls = 0;
volatile long int total_balls = 0;
volatile fix15 BOUNCINESS = float2fix15(0.495);
volatile long int frame_number =0;

// Flags
volatile int bounce_mode = 0;
volatile int reset       = 0;
volatile int clockwise   = 0;

// Determines frames per second
volatile long int start_time = 0;
volatile long int end_time = 0;
volatile int frame_time = 0;

// Radii
const fix15 BALL_RAD = int2fix15(4);
const fix15 PEG_RAD = int2fix15(6);
const fix15 TOTAL_RAD = int2fix15(10);


const fix15 GRAVITY = float2fix15(0.37);

// Board Position Constants
const fix15 ver_sep = int2fix15(19);
const fix15 hor_sep = int2fix15(38);
const fix15 half_hor_sep =  int2fix15(19);
const fix15 hor_center = int2fix15(320);
const fix15 ver_top    = int2fix15(50);

// Color Constants
char color = WHITE ;
const char ball_color = WHITE ;
const char TEXT_COLOR = WHITE ;
const char HIST_COLOR = WHITE ;

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size ;


// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * address_pointer = &DAC_data[0] ;


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


// Typedef of a boid
typedef struct {
  fix15 boid_x ;
  fix15 boid_y ;
  fix15 boid_vx ;
  fix15 boid_vy ;
} Boid;

// Typedef of peg position
typedef struct {
  int row;
  int col;
} Peg;

// Typedef of peg coordinates
typedef struct {
  fix15 x;
  fix15 y;
} Peg_xy;

// Buffers used for display
char buffer0[64];
char buffer1[64];
char buffer2[64];
char video_buffer[64];
char bins_buff[15][64];

// Array of the boids
static Boid boids1[12500];
static Boid boids2[12500];

// Array of peg positions
Peg_xy peg_pos[136];

// Storing recently hit pegs
Peg last_peg;
Peg curr_peg;

// Create a semaphore
semaphore_t draw_semaphore ;


// GPIO 2 ISR - Increments/decrements
void gpio_callback(uint gpio, uint32_t event_mask) {
    
    reset = 1;
  
      if (gpio_get(3)) {
        //Counter Clockwise
        if(bounce_mode){
          BOUNCINESS -= 327 ; //fix15 of 0.01
        } else {
          active_balls -= 10;
        }

      } else {
          //Clockwise
          if(bounce_mode){
            if (BOUNCINESS > 0.0)
              BOUNCINESS += 327 ; //fix15 of 0.01
          }
          else {
            if (active_balls > 0)
              active_balls += 10;
          }

      }
    
  }

// GPIO 4 ISR - Switch modes
void gpio_switch(uint gpio, uint32_t event_mask) {

  reset = 1;
  bounce_mode = !bounce_mode;
}

// Create a boid
void spawnBoidrand(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  
  // Start in center of screen
  *x = 10485760; //320
  *y = int2fix15(25);

  // Randomizes velocity in range (-0.5, 0.5)
  fix15 rand_vx = (rand() & 0x7FFF) - float2fix15(0.5) ;
  
  // If random velocity is 0, give it a slight velocity so it doesn't get stuck on the first peg
  if (rand_vx == 0) {
    *vx = rand_vx + 0.01;
  } else {
    *vx = rand_vx; 
  }
  *vx = rand_vx;
  *vy = int2fix15(0) ;

}


void spawnPeg(fix15 x_pos, fix15 y_pos, fix15* x, fix15* y)
{
  // Assign coordinates
  *x = x_pos;
  *y = y_pos;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y )) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  
  // If we hit the bottom, right or left
  if (hitBottom(*y) || hitRight(*x + 15) || hitLeft(*x - 15)) {
    total_balls += 1;
    
    // Count the number of balls fallen through for each histogram bin
    for (int b = 0; b < 15; b++) {
      if ((fix2int15(*x) > 35 + b * 38) && (fix2int15(*x) < 35 + (b + 1)*38)) {
        bins[b] += 1;
        
        // Updating bins max
        if (bins[b] > bins_max) {
          bins_max = bins[b];
          
        }
      }
    }

    // Respawn if we hit the bottom
    spawnBoidrand(x, y, vx, vy);
  } 

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}



// Detect wallstrikes, update velocity and position
void BouncePeg(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  fix15 sum_radius = TOTAL_RAD;

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;

      //fix15 y_pos = ver_top;
      fix15 y_pos;
      fix15 x_pos;

      // Row calculation
      int i_start = 0;
      int i_limit = 0;
      int y_fix = fix2int15(*y); // Variable to store y as int

      // Calculate two nearest columns
      if (y_fix < 50){
        i_start = 0;
        i_limit = 1;
      } else if (y_fix >= 50 && y_fix <= 335){
        i_start = (y_fix-50)/19;
        i_limit = i_start + 2;
      }

      int k = 0;

      // Loop through the calculated rows
      for (int i = i_start; i <= i_limit; i++) {

        // Column calculation
        int j_start = 0;
        int j_limit = 0;
        int x_fix = fix2int15(*x); // Variable to store x as int

        // Calculate two nearest columns
        j_start = (x_fix - (320 - 19*i))/38;
        if (j_start > 0){
          j_start -= 1;
        }
        if(j_start < 0)
          j_start = 0;
        j_limit = j_start + 1;
        if(j_limit > i)
          j_limit = i;

        // Loop through pegs in each column
        for (int j = j_start; j <= j_limit+1; j++) {

          // Calculate index for the peg pos array
          k = (i*(i+1))/2 + j;
          x_pos = peg_pos[k].x;
          y_pos = peg_pos[k].y;

          // Compute x and y distances between ball and peg
          fix15 dx = *x - x_pos;
          fix15 dy = *y - y_pos;

          curr_peg.row = i;
          curr_peg.col = j;

          fix15 abs_dx = absfix15(dx);
          fix15 abs_dy = absfix15(dy);

          // Check if both distances are less than the collision distance
          if(abs_dx < (TOTAL_RAD) && abs_dy < (TOTAL_RAD) ) {
            fix15 max = 0;
            fix15 min = 0;

            // Find the max and min between dx and dy
            if (abs_dx > abs_dy){
              max = abs_dx;
              min = abs_dy;
            } else {
              max = abs_dy;
              min = abs_dx;
            }

            // Calculate distance using alpha max beta min algorithm
            fix15 dist = (max) + (min>>2);

            // Check if we are in collision
            if(dist < (sum_radius)){
              
              //Generate normal vector
              fix15 normal_x;
              fix15 normal_y;
              if (dist > int2fix15(4)){
                normal_x = dx>>3;
                normal_y = dy>>3;
              } else if(dist >= int2fix15(2)) {
                normal_x = dx>>2;
                normal_y = dy>>2;
              } else {
                normal_x = dx>>1;
                normal_y = dy>>1;
              }

              // Collision physics
              fix15 intermediate_term = multfix15(int2fix15(-2), (multfix15(normal_x, *vx) + multfix15(normal_y, *vy)));

              // Teleport it outside the collison distance with the peg
              *x = x_pos + multfix15(normal_x, (sum_radius + int2fix15(1)));
              *y = y_pos + multfix15(normal_y, (sum_radius + int2fix15(1)));

              if(intermediate_term > 0) {
              // Update its velocity
              *vx = *vx + multfix15(normal_x, intermediate_term);
              *vy = *vy + multfix15(normal_y, intermediate_term);
              }
              
              
              // Make a sound
              if (curr_peg.row != last_peg.row && curr_peg.col != last_peg.col){
                dma_start_channel_mask(1u << ctrl_chan) ;
                last_peg.row = curr_peg.row;
                last_peg.col = curr_peg.col;
              }

              // Remove some energy from the ball
              *vx = multfix15(BOUNCINESS, *vx);
              *vy = multfix15(BOUNCINESS, *vy); 

            }

          }
          
        }
      
      }
  
  
  // Apply gravity
  *vy = *vy + GRAVITY;
  
}


// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);


    
    while(1) {
      // Wait for the signal that the buffer's changed
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      start_time = time_us_32();
      // Clear the buffer
      clearLowFrame(0, BLACK);
      // Signal core 1 that it can start drawing
      PT_SEM_SDK_SIGNAL(pt, &draw_semaphore) ;


      // Update previous active balls when new ball is added
      if (prev_act_balls != active_balls) {
        prev_act_balls = active_balls;
      }

      // Collision physics for half the active balls
      for (int i = 0; i < (prev_act_balls * 0.5); i++){
          BouncePeg(&boids1[i].boid_x, &boids1[i].boid_y, &boids1[i].boid_vx, &boids1[i].boid_vy);

      }

      // Wall detection for half the active balls
      for (int i = 0; i < (prev_act_balls * 0.5); i++) {
           wallsAndEdges(&boids1[i].boid_x, &boids1[i].boid_y, &boids1[i].boid_vx, &boids1[i].boid_vy) ;
      }

      // Draw 4 pixel ball for half the active balls
      for (int i = 0; i < (prev_act_balls * 0.5); i++){
           drawCircleCustom(fix2int15(boids1[i].boid_x), fix2int15(boids1[i].boid_y), ball_color) ;
      }

      // Draw histogram using bin data
      for(int i = 0; i < 15; i++){
       
        drawRect((fix2int15(hor_center) - (15 * fix2int15(half_hor_sep)) + (i * fix2int15(hor_sep))), 480, fix2int15(hor_sep) - 2, (bins[i]*100)/bins_max, HIST_COLOR);

      }

      // Print counter of current active balls
      sprintf(buffer0, "Counter of animated balls: %05d", active_balls);
      drawTextTiny8(0, 30, buffer0, TEXT_COLOR, BLACK) ;

      // Print counter of fallen balls
      sprintf(buffer1, "Counter of fallen balls: %d", total_balls);
      drawTextTiny8(0, 40, buffer1, TEXT_COLOR, BLACK) ;
      
      // Calculate the frame time and activate LED if its below the frame time for 60 fps
      frame_number++;
      end_time = time_us_32();
      frame_time = end_time - start_time;
      if (frame_time > 16667) {
        gpio_put(25, 1);
      } else {
        gpio_put(25, 0);
      }

     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    static uint64_t draw_time ;

    draw_time = PT_GET_TIME_usec();
    

    fix15 y_pos = ver_top;

        // Spawn all the pegs
        for (int i = 0; i < 16; i++) {

        fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));

        // Loop to spawn pegs in each row
        for (int j = 0; j <= i; j++) {
          spawnPeg(x_pos, y_pos, &boid1_x, &boid1_y);
        
          x_pos += hor_sep;
        }
        y_pos += ver_sep;
      }

     
    

    while(1) {
      // Wait for the signal from core 0
      PT_SEM_SDK_WAIT(pt, &draw_semaphore) ;
      
      
      // Reset the histogram if number of balls or bounciness has changed
      if (reset) {
        for (int i = 0; i < 15; i++) {
          bins[i] = 0;
        }
        total_balls = 0;
        // Put flag back to 0
        reset = 0;
      }
      
      y_pos = ver_top;

      // Draw the pegs
      for (int i = 0; i < 16; i++) {

        fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));

        // Loop to spawn pegs in each row
        for (int j = 0; j <= i; j++) {
          drawCircle(fix2int15(x_pos), fix2int15(y_pos), fix2int15(PEG_RAD), color); 
          x_pos += hor_sep;
        }
        y_pos += ver_sep;
      }

      // Print time since boot
      sprintf(video_buffer, "Time: %4.1f ms", (float)(PT_GET_TIME_usec()-draw_time)/1000);
      drawTextTiny8(0, 50, video_buffer, TEXT_COLOR, BLACK) ;

      // Print bounciness value
      sprintf(buffer2, "BOUNCINESS: %1.3f", fix2float15(BOUNCINESS));
      drawTextTiny8(0, 60, buffer2, TEXT_COLOR, BLACK) ;

    
      // Collision physics for half the active balls
      for (int i = 0; i < (prev_act_balls * 0.5); i++){
          BouncePeg(&boids2[i].boid_x, &boids2[i].boid_y, &boids2[i].boid_vx, &boids2[i].boid_vy);

      }

      // Wall detection for half the active balls
      for (int i = 0; i < (prev_act_balls * 0.5); i++) {
           wallsAndEdges(&boids2[i].boid_x, &boids2[i].boid_y, &boids2[i].boid_vx, &boids2[i].boid_vy) ;
      }

      // Draw 4 pixel ball for half the active balls
      for (int i = 0; i < (prev_act_balls * 0.5); i++){
           drawCircleCustom(fix2int15(boids2[i].boid_x), fix2int15(boids2[i].boid_y), ball_color) ;
      }

     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){

  // Initialize GPIO for rotary encoder switch
  gpio_init(4) ;
  gpio_set_dir(4, GPIO_IN);
  gpio_pull_up(4);
  gpio_set_irq_enabled_with_callback(4, GPIO_IRQ_EDGE_FALL, true, &gpio_switch);

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
  // Increase the voltage
  vreg_set_voltage(VREG_VOLTAGE_1_30);

  // Set system clock to 375MHz
  set_sys_clock_khz(375000, true) ;
  
  // initialize stido
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // Configure GPIO input 2 and 3 for rotary encoder
  gpio_init(2) ;
  gpio_init(3) ;
  gpio_init(25) ;
  

  gpio_set_dir(2, GPIO_IN) ;
  gpio_set_dir(3, GPIO_IN) ;
  gpio_set_dir(25,GPIO_OUT);


  gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  sem_init(&draw_semaphore, 0, 1) ;

  //==================================== DAC SECTION =========================================
  // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build sine table and DAC data table
    int i ;
    for (i=0; i<(sine_table_size); i++){
        raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
        DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
    }

    // Select DMA channels
    data_chan = dma_claim_unused_channel(true);;
    ctrl_chan = dma_claim_unused_channel(true);;

    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
    channel_config_set_read_increment(&c, false);                       // no read incrementing
    channel_config_set_write_increment(&c, false);                      // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

    dma_channel_configure(
        ctrl_chan,                          // Channel to be configured
        &c,                                 // The configuration we just created
        &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
        &address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers
        false                               // Don't start immediately
    );

  dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                       // yes read incrementing
    channel_config_set_write_increment(&c2, false);                     // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0
    


    dma_channel_configure(
        data_chan,                  // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        DAC_data,                   // The initial read address
        sine_table_size,            // Number of transfers
        false                       // Don't start immediately.
    );
  //==================================== DAC SECTION END =========================================
  

  //initialize all boid values
  for (int i = 0; i < 12500; i++) { // hard coded array length
    // Initialize first boid array
    boids1[i].boid_x = int2fix15(320);
    boids1[i].boid_y = int2fix15(25);
    
    boids1[i].boid_vx = (rand() & 0x7FFF) - float2fix15(0.5) ; // randomization 
    boids1[i].boid_vy = int2fix15(0);

    // Initialize second boid array
    boids2[i].boid_x = int2fix15(320);
    boids2[i].boid_y = int2fix15(0);
    
    boids2[i].boid_vx = (rand() & 0x7FFF) - float2fix15(0.5) ; // randomization
    boids2[i].boid_vy = int2fix15(0);
  }


  // Storing the coordinates of all the pegs
  int k = 0;
  fix15 y_pos = ver_top;
  for (int i = 0; i < 16; i++) {

    fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));

    // Loop to store peg positions for every peg
    for (int j = 0; j <= i; j++) {
      peg_pos[k].x = x_pos;
      peg_pos[k].y = y_pos;
      x_pos += hor_sep;
      k++;
    }
    y_pos += ver_sep;
  }
  
  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
