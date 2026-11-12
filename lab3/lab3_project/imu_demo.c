/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *  - GPIO 11 ---> Push button
 *  - GPIO 4 ---> PWM Output
 *  - GPIO 28 ---> IMU boot
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"


// We will start drawing at column 81
#define LEFT_EDGE 81 

// Rescale the measurements for display
#define OLD_RANGE 500.  // (+/- 250)
#define NEW_RANGE 150.  // (looks nice on VGA)
#define OLD_MIN -250. 
#define OLD_MAX 250. 
#define ACCEL_SCALE 120.0
#define ANGLE_SCALE 1.8


// Place the plots
#define TOP_PLOT_BOTTOM 200
#define BOTTOM_PLOT_BOTTOM 275
#define ANGLE_PLOT_BOTTOM 435
#define PLOT_WIDTH 530
#define PI 3.1416

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Arrays for circular display buffer
fix15 ax[PLOT_WIDTH] ;
fix15 ay[PLOT_WIDTH] ;
fix15 az[PLOT_WIDTH] ;
fix15 gx[PLOT_WIDTH] ;
fix15 gy[PLOT_WIDTH] ;
fix15 gz[PLOT_WIDTH] ;
fix15 comp_angle_array[PLOT_WIDTH];
fix15 des_angle_array[PLOT_WIDTH];
int pwm_output_array[PLOT_WIDTH];
int d_term_array[PLOT_WIDTH];
int p_term_array[PLOT_WIDTH];
int i_term_array[PLOT_WIDTH];

// Pointer to head of buffer
volatile int head = 0 ;

// PWM duty cycle
volatile int control = 10;
volatile int old_control = 10;

// character array
char screentext[40];

// draw speed
volatile int threshold = 40 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 6000
#define CLKDIV  25.0

// GPIO we're using for PWM
#define PWM_OUT 4

uint slice_num ;

// Variables for data calculations
volatile fix15 accel_angle = 0;
volatile fix15 gyro_angle_delta = 0;
volatile fix15 complementary_angle = 0;
volatile fix15 error = 0;
volatile fix15 desired_angle = int2fix15(0);
volatile int count = 0;
volatile fix15 old_error = 0;
volatile fix15 sum_error = 0;
volatile fix15 old_i_mul = 0;
volatile fix15 i_mul = 0;

// Default PID parameters
volatile fix15 KP = float2fix15(200);
volatile fix15 KD = float2fix15(30);
volatile fix15 KI = float2fix15(16);

// Display buffers
char comp_angle_buffer[64];
char des_angle_buffer[64];
char error_buffer[64];
char control_buffer[64];
char kp_buffer[64];
char kd_buffer[64];
char ki_buffer[64];
char delta_error_buffer[64];
char pid_buffer[64];


void drawAxes() {
    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 150, 5, CYAN) ;
    drawHLine(75, 200, 5, CYAN) ;
    drawHLine(75, 250, 5, CYAN) ;
    drawVLine(80, 150, 100, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 200) ;
    writeString(screentext) ;
    sprintf(screentext, "3000") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "-3000") ;
    setCursor(50, 250) ;
    writeString(screentext) ;
    sprintf(screentext, "PID") ;
    setCursor(10, 200) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 25, 5, CYAN) ;
    drawHLine(75, 125, 5, CYAN) ;
    drawHLine(75, 75, 5, CYAN) ;
    drawVLine(80, 25, 100, CYAN) ;
    sprintf(screentext, "3000") ;
    setCursor(50, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "6000") ;
    setCursor(45, 25) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(45, 125) ;
    writeString(screentext) ;
    sprintf(screentext, "control") ;
    setCursor(0, 75) ;
    writeString(screentext) ;

    // Draw angle plot
    drawHLine(75, 275, 5, CYAN) ;
    drawHLine(75, 325, 5, CYAN) ;
    drawHLine(75, 375, 5, CYAN) ;
    drawVLine(80, 275, 100, CYAN) ;
    sprintf(screentext, "90") ;
    setCursor(50, 325) ;
    writeString(screentext) ;
    sprintf(screentext, "180") ;
    setCursor(45, 275) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(45, 375) ;
    writeString(screentext) ;
    sprintf(screentext, "angle") ;
    setCursor(10, 325) ;
    writeString(screentext) ;
}


volatile int throttle = 10 ;

// PID terms
volatile fix15 d_term = 0;
volatile fix15 p_term = 0;
volatile fix15 i_term = 0;
volatile int pid_term = 0;

// Variables for angle sequence
volatile int start_time = 0;
volatile int mode = 0;
volatile int time = -1;
volatile int boot = 0;

// GPIO interrupt
void gpio_callback(uint gpio, uint32_t event_mask) {
    if(boot) {
        if(event_mask & GPIO_IRQ_EDGE_FALL) {
            desired_angle = 0;
        } else if (event_mask & GPIO_IRQ_EDGE_RISE) {
            mode = 1;
            time = 0;
        }
    } else {
        boot = 1;
    }
    
}

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(4));
    gpio_put(14, !gpio_get(14)) ;

    
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
    

    if (old_error != error) {
        old_error = error;
    }

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // Low pass the accelerometer
    int filtered_ax;
    int filtered_az;
    filtered_ax = filtered_ax + ((acceleration[0] - filtered_ax) >> 2);
    filtered_az = filtered_az + ((acceleration[2] - filtered_az) >> 2);
    
    if (filtered_az < int2fix15(0) && desired_angle > int2fix15(90)){
        filtered_az = multfix15(int2fix15(-1), filtered_az);
    }
    accel_angle = multfix15(float2fix15(atan2(filtered_az, filtered_ax)), oneeightyoverpi);

    // Gyro angle delta
    gyro_angle_delta = multfix15(gyro[1], zeropt001);

    // Complementary angle calc
    complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // Calculate the error
    error = desired_angle - complementary_angle - int2fix15(25);

    // Add the error
    i_mul = multfix15(error, zeropt001);
    if (i_mul != old_i_mul){
        sum_error = sum_error + i_mul;
        old_i_mul = i_mul;
    }

    // Make sure to integrator term doesn't go up too much
    if (sum_error > int2fix15(200)){
        sum_error = int2fix15(200);
    } 
    if (sum_error < int2fix15(-200)){
        sum_error = int2fix15(-200);
    }

    // Multiplying each term with their coefficients
    i_term = multfix15(KI, sum_error);
    d_term = multfix15(KD, gyro[1]);
    p_term = multfix15(KP, error);

    if (count == 1) {
        // Proportional Control
        pid_term = fix2int15(p_term - d_term + i_term);
        control = (pid_term > 3700) ? 3500 : pid_term;
        control = (control < 0) ? 0 : control;
        count = 0;
    } 
    count++;

    // Angle sequence logic
    if (mode) {
        if (time == 0){
            desired_angle = int2fix15(90);
        } else if (time == 5000) {
            desired_angle = int2fix15(120);
        } else if (time == 10000) {
            desired_angle = int2fix15(60);
        } else if (time == 15000) {
            desired_angle = int2fix15(90);
            time = 0;
            mode = 0;
        }
        time++;
    }


    // Increment drawspeed controller
    throttle += 1 ;
    // Maintain a circular buffer for display
    if ((throttle >= threshold)) { 
        // Zero speed controller
        throttle = 0 ;

        // Replace oldest data in accel arrays with new sample
        ax[head] = acceleration[0] ;
        ay[head] = acceleration[1] ;
        az[head] = acceleration[2] ;

        // Replace oldest data in gyro arrays with new sample
        gx[head] = gyro[0] ;
        gy[head] = gyro[1] ;
        gz[head] = gyro[2] ;

        // Replace oldest data in angle arrays with new sample
        comp_angle_array[head] = complementary_angle;
        des_angle_array[head] = desired_angle;
        pwm_output_array[head] = control;
        p_term_array[head] = fix2int15(p_term);
        d_term_array[head] = fix2int15(d_term);
        i_term_array[head] = fix2int15(i_term);

        // Move the pointer to the oldest data
        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1) ;

    }
}


static PT_THREAD (protothread_draw(struct pt *pt))
{
    PT_BEGIN(pt) ;

    static int start ;
    static int i ;

    // Draw axes
    drawAxes() ;
    // Copy to other buffer
    copy_buffer_to_other() ;

    while(1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;
        gpio_put(15, !gpio_get(15)) ;
        
        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        // Plot data oldest-->newest
        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            
            // Draw the angle plot
            drawPixel(LEFT_EDGE + i, (ANGLE_PLOT_BOTTOM - 
                (int)(NEW_RANGE*((float)((fix2float15(comp_angle_array[start])*ANGLE_SCALE)-OLD_MIN)/OLD_RANGE))), WHITE) ;
            drawPixel(LEFT_EDGE + i, (ANGLE_PLOT_BOTTOM - 
                (int)(NEW_RANGE*((float)(((fix2float15(des_angle_array[start])-25)*ANGLE_SCALE)-OLD_MIN)/OLD_RANGE))), BLUE) ;

            // Draw the PWM/control value
            drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM - 
                (int)(NEW_RANGE*((float)(((pwm_output_array[start])*0.05)-OLD_MIN)/OLD_RANGE))), WHITE) ;
            
            // Draw the PID terms
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM - 
                (int)(NEW_RANGE*((float)(((p_term_array[start])*0.02)-OLD_MIN)/OLD_RANGE))), GREEN) ;
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM - 
                (int)(NEW_RANGE*((float)(((d_term_array[start])*0.02)-OLD_MIN)/OLD_RANGE))), RED) ;
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM - 
                (int)(NEW_RANGE*((float)(((i_term_array[start])*0.03)-OLD_MIN)/OLD_RANGE))), PINK) ;

            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1) ;
        }

        
        // Print values at the bottom of the VGA display
        sprintf(comp_angle_buffer, "Comp angle: %4.1f", (fix2float15(complementary_angle)+25));
        drawTextVGA437(10, 425, comp_angle_buffer, WHITE, BLACK) ;
        sprintf(des_angle_buffer, "Desired angle: %4.1f", (fix2float15(desired_angle)));
        drawTextVGA437(10, 400, des_angle_buffer, BLUE, BLACK) ;
        sprintf(error_buffer, "Error: %4.1f", fix2float15(error));
        drawTextVGA437(10, 450, error_buffer, BLUE, BLACK) ;
        sprintf(control_buffer, "Control: %d", control);
        drawTextVGA437(390, 450, control_buffer, WHITE, BLACK) ;
        sprintf(kp_buffer, "KP: %4.1f", fix2float15(KP));
        drawTextVGA437(200, 450, kp_buffer, GREEN, BLACK) ;
        sprintf(kd_buffer, "KD: %4.1f", fix2float15(KD));
        drawTextVGA437(200, 400, ki_buffer, PINK, BLACK) ;
        sprintf(ki_buffer, "KI: %4.1f", fix2float15(KI));
        drawTextVGA437(200, 425, kd_buffer, RED, BLACK) ;
        
    }

    PT_END(pt) ;
}

// User input thread
static PT_THREAD (protothread_serial_motor(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int   test_in ;
    static float   test_in1;
    static int number;
    while(1) {
        sprintf(pt_serial_out_buffer, "Enter the number for the desired parameter you want to change and put the desired value after: \n1 - desired beam angle\n2 - Kp\n3 - Kd\n4 - Ki\n");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input strings to numbers
        sscanf(pt_serial_in_buffer,"%d, %f", &test_in, &test_in1) ;
        number = test_in ;

        // Changes the appropriate parameters based on user input
        if (number == 1) {
            desired_angle = float2fix15(test_in1);
        }
        else if (number == 2) {
            KP = float2fix15(test_in1);
        }
        else if (number == 3) {
            KD = float2fix15(test_in1);
        }
        else if (number == 4) {
            KI = float2fix15(test_in1);
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    printf("Debug-core1\n");
    gpio_init(15) ;
    gpio_set_dir(15, GPIO_OUT) ;
    gpio_put(15, 0) ;
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;



    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// POWERING THE IMU  ////////////////////////////
    gpio_init(28) ;
    gpio_set_dir(28, GPIO_OUT) ;
    gpio_put(28, 0);
    sleep_ms(100);
    gpio_put(28, 1);
    sleep_ms(250);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO 4 that they allocated to the PWM
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    //pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// BUTTON CONFIGURATION /////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
   
    gpio_init(11) ;
    gpio_set_dir(11, GPIO_IN) ;
    gpio_pull_up(11);
   
    gpio_set_irq_enabled_with_callback(11, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    
    

   
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    gpio_init(14) ;
    gpio_set_dir(14, GPIO_OUT) ;
    gpio_put(14, 0) ;


    pt_add_thread(protothread_serial_motor) ;
    pt_schedule_start ;

}
