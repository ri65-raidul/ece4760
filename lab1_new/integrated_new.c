
/*
Simple ADC/Protothreads demo

Schedules a single thread, reads/prints ADC value

 */

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <math.h>
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"

// ==========================================
// === protothreads globals
// ==========================================
// protothreads header
#include "pt_cornell_rp2040_v1_4.h"


// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

#define LED_PIN 25
#define ADC_PIN 26
#define ADC_MUX 0

unsigned int keycodes[NUMKEYS] = {      0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                        0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                        0x67, 0x37} ;
unsigned int scancodes[KEYROWS] = {   0xE, 0xD, 0xB, 0x7} ;
unsigned int button = 0x70 ;


char keytext[40];
int prev_key = 0;

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

//DDS parameters
#define two32 4294967296.0 // 2^32 
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)
// the DDS units:
volatile unsigned int phase_accum_main;
// base multiplier with scaling 
volatile unsigned int phase_incr_base = (two32 * 2.5)/Fs ;

// SPI data
uint16_t DAC_data ; // output value

//DAC parameters
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size] ;

// Global variables
volatile unsigned int adc_val ;

// Booleans to set modes
volatile int gen_tone = 1; //default tone generating mode
volatile int record = 0;
volatile int compose_playback = 0;
volatile int in_progress = 0;
volatile int playback = 0;

// Record mode array variables
volatile int recorded[9][3000];
volatile int size[9]; //size of recording for each button

// Loop variables for record mode
volatile int record_ind = 0;
volatile int record_loops = 0;

// Loop variables for compose mode
volatile int compose_ind = 0;
volatile int compose_seq_button = 0;
volatile int compose_loops = 0;

// Button variables for FSM
volatile int button_stored = 0;
volatile int button_pressed;

// Compose mode array variables
volatile int compose_arr_size = 0;
volatile int compose_arr[40];

volatile int freq_base;



// ==================================================
// === Alarm ISR
// ==================================================
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    // Tone Generator Mode - Default
    if(gen_tone || record) {
      // DDS phase and sine table lookup
      phase_accum_main += phase_incr_base * adc_val  ;
      DAC_data = (DAC_config_chan_B | ((sin_table[phase_accum_main>>24] + 2048) & 0xffff))  ;

      // Perform an SPI transaction
      spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
    }
    if(compose_playback || playback){
      phase_accum_main += phase_incr_base * freq_base  ;
      DAC_data = (DAC_config_chan_B | ((sin_table[phase_accum_main>>24] + 2048) & 0xffff))  ;

      // Perform an SPI transaction
      spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;

}

// ==================================================
// === scan function
// ==================================================
int scan_keypad() {
        // Some variables
        static int i ;
        static uint32_t keypad ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ;
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if ((~keypad) & button) break ;
        }
        // If we found a button . . .
        if ((~keypad) & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;

        return i;
}


// ==================================================
// === keypad thread
// ==================================================

// Defining the states
typedef enum {
    NOT_PRESSED,
    MAYBE_PRESSED,
    PRESSED,
    MAYBE_NOT_PRESSED
} state_t;

state_t state;

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Local variables
    static int button;
    static int possible;
    static int compose = 0;
    

    while(1) {

        // FSM for debouncing
        switch(state) {
            case NOT_PRESSED:
                // Initial button scan
                button = scan_keypad();
                if(button == -1) {
                    // Stays in this state while button is not pressed
                    button = scan_keypad();
                }
                else {
                  state = MAYBE_PRESSED;
                  possible = button;
                }
                break;

            case MAYBE_PRESSED:
                // Initial button scan
                button = scan_keypad();
                if(button != possible){
                    // Leaves state if button scanned changes
                    state = NOT_PRESSED;
                }
                else {
                    state = PRESSED;
                    printf("\n%d", possible);
                    
                    // Tone generator toggled by button 0
                    if(button == 0 && gen_tone == 1) 
                      gen_tone = 0;
                    else if (button == 0)
                      gen_tone = 1;
                    
                    // Enter record mode
                    if(button == 10){
                      record = 1;
                    }
                    
                    // When button is pressed in record mode
                    if(record && button != 0 && button != 11 && button != 10){
                      // Store current button
                      button_stored = button;
                      // Set boolean for recording in progress
                      in_progress = 1;
                      // SInitialize size of recording to prevent reading stale values
                      size[button_stored - 1] = 0;
                    }

                    // Detects second button press indicating playback mode
                    if(in_progress == 0 && button != 10 && button != 0 && button != 11){
                      playback = 1;
                      button_pressed = button;
                    }

                    // Store sequence of buttons
                    if (compose && button != 11){
                      
                      //============== DEBUG STATEMENT ============================
                      printf("Appending %d", button);
                    
                      compose_arr[compose_arr_size] = button;
                      compose_arr_size++;
                    }
                    
                    // Enter compose mode
                    if(button == 11 && compose != 1){
                      printf("Compose Mode");
                      compose = 1;
                    } 

                    // Leaving compose mode and start playback of stored buttons
                    else if (button == 11) {
                      compose = 0;
                      compose_playback=1;

                      //============== DEBUG STATEMENT ============================
                      for(int i = 0; i < 20; i++){
                        printf("Coming out of compose mode, %d", compose_arr[i]);
                      }
                      
                    }
                    
                }
                break;

            case PRESSED:
                // Initial button scan
                button = scan_keypad();
                if(button == possible){
                    // Stay in state while previous button matches current button
                    button = scan_keypad();
                    state = PRESSED;
                }
                else{
                  state = MAYBE_NOT_PRESSED;
                }
                break;

            case MAYBE_NOT_PRESSED:
                // Initial button scan
                button = scan_keypad();
                if(button == possible){
                    // Switch state if same button as before is pressed
                    state = PRESSED;
                }
                else {
                    state = NOT_PRESSED;
                    
                    // Stop recording if recording is in progress and button is released
                    if(in_progress){
                      record = 0;
                      in_progress = 0;
                    }
                      
                }
                break;
            
            default: state = NOT_PRESSED;
        };

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// ==================================================
// === toggle25 thread 
// ==================================================
static PT_THREAD (protothread_toggle25(struct pt *pt))
{
    PT_BEGIN(pt);
    static int count;

      while(1) {
        // toggle gpio 25
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // Read the ADC
        adc_val = adc_read() ;

        // Storing recorded values for a button
        if(in_progress){
          
          // Append current ADC read to array for stored button
          recorded[button_stored - 1][count] = adc_val;
          
          //============== DEBUG STATEMENT ============================
          printf("ADC value stored: %d, %d\n", recorded[button_stored - 1][count], adc_val) ;
          
          count++;
          size[button_stored - 1]++;
        }
        else{
          count = 0;
        }
        
        

        // Yield
        PT_YIELD_usec(10000) ;
      } // END WHILE(1)
      // every thread ends with PT_END(pt);
      PT_END(pt);
} // end blink thread

// ==================================================
// === playback thread 
// ==================================================
static PT_THREAD (protothread_playback(struct pt *pt)) 
{
  PT_BEGIN(pt);
  while(1){

  // Compose Mode
  if(compose_playback) {
    
    // Parses through the stored sequence of buttons
    for(compose_seq_button = 0; compose_seq_button < compose_arr_size; compose_seq_button++) {
      
      // Goes through the recorded values for each button
      for(compose_ind = 0; compose_ind < size[compose_arr[compose_seq_button] - 1]; compose_ind++) {
        
        freq_base = recorded[compose_arr[compose_seq_button] - 1][compose_ind];
        PT_YIELD_usec(1000);
      } 
      
    }
    compose_arr_size = 0;
    compose_playback = 0;

  }

  // Record Mode
  if(playback) {
    
    // Parses through recorded values for the button pressed
    for(record_ind = 0; record_ind < size[button_pressed - 1]; record_ind++) {
        //============== DEBUG STATEMENT ============================
        //printf("%d\n", recorded[button_pressed - 1][record_ind]);
        freq_base = recorded[button_pressed - 1][record_ind];
        
        PT_YIELD_usec(1000);
    }
    playback = 0;
  }

    PT_YIELD_usec(1000);
  }

  PT_END(pt);

}

// ========================================
// === core 1 main
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_playback);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === core 0 main
// ========================================
int main(){
  // Overclock
    set_sys_clock_khz(150000, true) ;
  
  //===  start the serial i/o ==================
  stdio_init_all() ;
  // announce the threader version on system reset
  // if there is a seral terminal attached
  printf("\n\rProtothreads RP2040 v1.4\n\r");

  // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

  ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    gpio_set_dir((BASE_KEYPAD_PIN+4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+6), GPIO_IN);
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_up((BASE_KEYPAD_PIN+4)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+5)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+6)) ;


  // Setup the ADC
  adc_init() ;
  adc_gpio_init(ADC_PIN) ;
  adc_select_input(ADC_MUX) ;

  // set up LED gpio 25
  gpio_init(LED_PIN) ;  
  gpio_set_dir(LED_PIN, GPIO_OUT) ;
  gpio_put(LED_PIN, true);


  //dac_test
  // Initialize stdio
    stdio_init_all();
    printf("Hello, DAC!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // === build the sine lookup table =======
   	// scaled to produce values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = (int)(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

      // start core 1 
        multicore_reset_core1();
        multicore_launch_core1(&core1_main);

    // === config threads ========================
    pt_add_thread(protothread_toggle25);
    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;
    //pt_add_thread(protothread_playback) ;
  
  // === initalize the scheduler ===============
  pt_schedule_start ;
} // end main
