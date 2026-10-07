
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
#define FRAME_RATE 16500


//DMA
// Number of samples per period in sine table
#define sine_table_size 256

volatile int data_chan;
volatile int ctrl_chan;

// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * address_pointer = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size ;


// the color of the pegs
char color = WHITE ;
const char ball_color = PINK ;
const char TEXT_COLOR = GREEN ;
const char HIST_COLOR = GREEN ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Typedef of a boid
typedef struct {
  fix15 boid_x ;
  fix15 boid_y ;
  fix15 boid_vx ;
  fix15 boid_vy ;
} Boid;

// Array of the boid
static Boid boids[5000];

// array for the histogram bins
volatile int bins[15]; // only 15 gaps between pegs for a row of 16 pegs
volatile int bins_max;


// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;


const fix15 BALL_RAD = int2fix15(3);
const fix15 PEG_RAD = int2fix15(5);
const fix15 TOTAL_RAD = int2fix15(8);


const fix15 GRAVITY = float2fix15(0.37);

const fix15 ver_sep = int2fix15(19);
const fix15 hor_sep = int2fix15(38);
volatile fix15 half_hor_sep =  int2fix15(19);

const fix15 hor_center = int2fix15(320);
const fix15 ver_top    = int2fix15(50);

const fix15 ALPHA = 1;
const fix15 BETA = 1;

// Global counters
volatile long int active_balls =1250;
volatile long int prev_act_balls = 0;
volatile long int total_balls = 0;
volatile fix15 BOUNCINESS = float2fix15(0.5);

typedef struct {
  int row;
  int col;
} Peg;

Peg last_peg;
Peg curr_peg;

typedef struct {
  fix15 x;
  fix15 y;
} Peg_xy;

Peg_xy peg_pos[136];

char buffer0[64];
char buffer1[64];
char buffer2[64];

char video_buffer[64];
char bins_buff[15][64];
// Flags
volatile int bounce_mode = 0;
volatile int reset       = 0;
volatile int clockwise   = 0;

// Create a semaphore
semaphore_t draw_semaphore ;


// GPIO 2 ISR. Increases balls
void gpio_callback(uint gpio, uint32_t event_mask) {
    printf("enters callback\n");
    reset = 1;

    if(gpio == 4) {
      printf("enters bounce\n");
      bounce_mode = !bounce_mode;
    }
    
    if(gpio == 2){
      if (gpio_get(3)) {
        //Counter Clockwise
        if(bounce_mode){
          if (BOUNCINESS > 0.0){
            BOUNCINESS -= 327; //fix15 of 0.5
          }
        } else {
          if (active_balls > 0){
            active_balls -= 10;
          }
        }

      } else {
          //Clockwise
          if(bounce_mode){
            BOUNCINESS += 327; //fix15 of 0.5
          }
          else {
            active_balls += 10;
          }

      }
    }
  }

// GPIO 4 SWITCH
void gpio_switch(uint gpio, uint32_t event_mask) {
  reset = 1;
  bounce_mode = !bounce_mode;
}



// Create a boid
void spawnBoidrand(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(0) ;


  //*vx = float2fix15(dir);
  fix15 rand_vx = (rand() & 0x7FFF) - 16384 ;
  // rand_vx = rand_vx - 32768;
  // if (rand_vx == 0) {
  //   *vx = rand_vx + 0.01;
  // } else {
  //   *vx = rand_vx; // come back to this
  // }
  *vx = rand_vx;
  *vy = int2fix15(0) ;

}


void spawnPeg(fix15 x_pos, fix15 y_pos, fix15* x, fix15* y)
{
  // Start in center of screen
  *x = x_pos;
  *y = y_pos;
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
    total_balls += 1;
    
    for (int b = 0; b < 15; b++) {
      if ((fix2int15(*x) > 35 + b * 38) && (fix2int15(*x) < 35 + (b + 1)*38)) {
        bins[b] += 1;
        //printf("Bins[b]: %d\n", bins[b]);
        // Updating bins max
        if (bins[b] > bins_max) {
          bins_max = bins[b];
          //printf("Bins Max: %d\n", bins_max);
        }
      }
    }

    
    spawnBoidrand(x, y, vx, vy);
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
  fix15 sum_radius = TOTAL_RAD;

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;

      //fix15 y_pos = ver_top;
      fix15 y_pos;
      fix15 x_pos;

      // // Row calculation
      // int i_start = 0;
      // int i_limit = 0;
      // // Variable to store y as int
      // int y_fix = fix2int15(*y);
      // if (y_fix < 50){
      //   i_start = 0;
      //   i_limit = 1;
      // } else if (y_fix >= 50 && y_fix <= 335){
      //   i_start = (y_fix-50)/19 + 1;
      //   i_limit = i_start + 1;
      // }

      int k = 0;

      // row
      for (int i = 0; i < 16; i++) {

        //fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));
        y_pos = peg_pos[k].y;

        // Column calculation
        // int j_start = 0;
        // int j_limit = 0;
        // // Variable to store x as int
        // int x_fix = fix2int15(*x);
        // j_start = (x_fix - (320 - 19*i_start))/38;
        // if (j_start > 0){
        //   j_start -= 1;
        // }
        // j_limit = j_start + 1;
        // printf("x_pos: %d, y_pos: %d\n", x_fix, y_fix);
        // printf("i_start: %d, i_limit: %d\n", i_start, i_limit);
        // printf("j_start: %d, j_limit: %d\n\n\n\n", j_start, j_limit);

        // Loop through each pegs in each row
        // column
        for (int j = 0; j <= i; j++) {
          x_pos = peg_pos[k].x;
          fix15 dx = *x - x_pos;
          fix15 dy = *y - y_pos;

          curr_peg.row = i;
          curr_peg.col = j;

          fix15 abs_dx = absfix15(dx);
          fix15 abs_dy = absfix15(dy);

          if(abs_dx < (TOTAL_RAD) && abs_dy < (TOTAL_RAD) ) {
            fix15 max = 0;
            fix15 min = 0;
            if (abs_dx > abs_dy){
              max = abs_dx;
              min = abs_dy;
            } else {
              max = abs_dy;
              min = abs_dx;
            }
            fix15 dist = (max) + (min>>2);
            //printf("%d, ", fix2int15(dist));

            //fix15 dist = float2fix15(sqrtf( fix2float15(multfix15(dx, dx)) + fix2float15(multfix15(dy, dy)) ));

            // printf("dx squared: %d\n", fix2int15(multfix15(dx,dx)));
            // printf("dy squared: %d\n", fix2int15(multfix15(dy,dy)));
            //printf("Dist: %d\n", (int)(dist >> 15));

            if(dist < (sum_radius)){
              //printf("%d, %d\n", fix2int15(dist), fix2int15(sum_radius));
              //printf("Enters if");
              //Generate normal vector
              //fix15 normal_x = divfix(dx, dist);
              //fix15 normal_y = divfix(dy, dist);
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
          //x_pos += hor_sep;
          k++;
        }
        //y_pos += ver_sep;
      }
  
  
  // Apply gravity
  *vy = *vy + GRAVITY;
  
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
    
    while(1) {
      // Wait for the signal that the buffer's changed
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      // Clear the buffer
      clearLowFrame(0, BLACK);
      // Signal core 1 that it can start drawing
      PT_SEM_SDK_SIGNAL(pt, &draw_semaphore) ;

      // if (clockwise == 1) {
      //   active_balls-=10;
      //   clockwise = 0;
      // }
      // else if (clockwise == 2) {
      //   active_balls+=10;
      //   clockwise = 0;
      // }


      // Spawn a boid if new ball is added
      if (prev_act_balls != active_balls) {
        spawnBoidrand(&boids[prev_act_balls].boid_x, &boids[prev_act_balls].boid_y, &boids[prev_act_balls].boid_vx, &boids[prev_act_balls].boid_vy);
        prev_act_balls = active_balls;
      }

      for (int i = 0; i < prev_act_balls; i++){
          BouncePeg(&boids[i].boid_x, &boids[i].boid_y, &boids[i].boid_vx, &boids[i].boid_vy);

      }

      for (int i = 0; i < prev_act_balls; i++) {
           wallsAndEdges(&boids[i].boid_x, &boids[i].boid_y, &boids[i].boid_vx, &boids[i].boid_vy) ;
      }

      for (int i = 0; i < prev_act_balls; i++){
           fillCircle(fix2int15(boids[i].boid_x), fix2int15(boids[i].boid_y), fix2int15(BALL_RAD), ball_color) ;
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
    // Spawn a peg
    //spawnPeg(hor_center, ver_top, &boid1_x, &boid1_y);

    fix15 y_pos = ver_top;

        for (int i = 0; i < 16; i++) {

        fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));

        // Loop to spawn pegs in each row
        for (int j = 0; j <= i; j++) {
          spawnPeg(x_pos, y_pos, &boid1_x, &boid1_y);
          //printf("xpos: %d", fix2int15(x_pos), " y-pos: %d\n", fix2int15(y_pos));
          //fillCircle(fix2int15(boid1_x), fix2int15(boid1_y), fix2int15(PEG_RAD), color); 
          x_pos += hor_sep;
        }
        y_pos += ver_sep;
      }

    

    while(1) {
      // Wait for the signal from core 0
      PT_SEM_SDK_WAIT(pt, &draw_semaphore) ;
      
      // Spawn pegs for 16 rows
      if (reset) {
        for (int i = 0; i < 15; i++) {
          bins[i] = 0;
        }
        total_balls = 0;
        // Put flag back to 0
        reset = 0;
      }
      
      y_pos = ver_top;

      for (int i = 0; i < 16; i++) {

        fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));

        // Loop to spawn pegs in each row
        for (int j = 0; j <= i; j++) {
          fillCircle(fix2int15(x_pos), fix2int15(y_pos), fix2int15(PEG_RAD), color); 
          x_pos += hor_sep;
        }
        y_pos += ver_sep;
      }

      sprintf(buffer0, "Counter of animated balls: %05d", active_balls);
      drawTextTiny8(0, 30, buffer0, TEXT_COLOR, BLACK) ;

      sprintf(buffer1, "Counter of fallen balls: %d", total_balls);
      drawTextTiny8(0, 40, buffer1, TEXT_COLOR, BLACK) ;

      sprintf(video_buffer, "Time: %4.1f ms", (float)(PT_GET_TIME_usec()-draw_time)/1000);
      drawTextTiny8(0, 50, video_buffer, TEXT_COLOR, BLACK) ;

      //sprintf(buffer2, "Bounciness: %f\n", fix2float15(BOUNCINESS));
      //drawTextTiny8(0, 60, buffer2, GREEN, BLACK) ;

      sprintf(buffer2, "BOUNCINESS: %1.3f", fix2float15(BOUNCINESS));
      drawTextTiny8(0, 60, buffer2, TEXT_COLOR, BLACK) ;


      for(int i = 0; i < 15; i++){
        sprintf(bins_buff[i], "%d", bins[i]);
        //drawTextTiny8(54 +  (i * 38), 360, bins_buff[i], GREEN, BLACK);
        //fillRect(35 + (i * 38), 480, 36, (bins[i]*100)/bins_max, GREEN);
        drawTextTiny8(fix2int15(hor_center) - (14 * fix2int15(half_hor_sep)) + (i * fix2int15(hor_sep)), 360, bins_buff[i], TEXT_COLOR, BLACK);
        // 35 = hor_center - (15 * half_hor_sep)
        fillRect((fix2int15(hor_center) - (15 * fix2int15(half_hor_sep)) + (i * fix2int15(hor_sep))), 480, fix2int15(hor_sep) - 2, (bins[i]*100)/bins_max, HIST_COLOR);
        //printf("bins[%d] height: %d\n", i, (bins[i]*100)/bins_max);

      }

      //printf("%d", active_balls);
      //fillCircle(fix2int15(boid1_x), fix2int15(boid1_y), fix2int15(PEG_RAD), color); 
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  //set_sys_clock_khz(375000, true) ;
  // initialize stio
  //stdio_init_all() ;
  
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

  set_sys_clock_khz(375000, true) ;
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // Configure GPIO input 2 for interrupt
  gpio_init(2) ;
  gpio_init(3) ;
  gpio_init(25) ;
  //gpio_init(4) ;

  gpio_set_dir(2, GPIO_IN) ;
  gpio_set_dir(3, GPIO_IN) ;
  gpio_set_dir(25,GPIO_OUT);
  //gpio_set_dir(4, GPIO_IN);


  //gpio_pull_up(2) ;
  //gpio_pull_up(3) ;
  //gpio_pull_up(4);

  gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  //gpio_set_irq_enabled_with_callback(4, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

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
  //==================================== DAC SECTION =========================================
  

  //initialize all boid values
  for (int i = 0; i < 5000; i++) { // hard coded array length
    boids[i].boid_x = int2fix15(320);
    boids[i].boid_y = int2fix15(0);
    // 0000_0000_0111_1111
    boids[i].boid_vx = rand() & 0x7FFF - float2fix15(0.5); // randomization - change later
    boids[i].boid_vy = int2fix15(0);
  }

  // Hardcoding all the positions of the pegs
  int k = 0;
  fix15 y_pos = ver_top;
  for (int i = 0; i < 16; i++) {

    fix15 x_pos = hor_center - multfix15(half_hor_sep, int2fix15(i));

    // Loop to spawn pegs in each row
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
  //pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
