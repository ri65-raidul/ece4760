//Rotary Encoder Interface

/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * Simple GPIO interrupt demo.
 * 
 * Wire GPIO 2 to GPIO 3 (thru a resistor).
 * The code toggles GPIO 3, triggering an ISR
 * at every rising edge. The ISR blinks the LED.
 * 
 * Note that the onboard LED is on GPIO 25.
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

volatile int count = 0;

// GPIO ISR. Toggles LED
void gpio_callback() {
    if (gpio_get(3)) {
        // Counter Clockwise
        count--;
        printf("Counter: %d", count);
    } else {
        // clockwise
        count++;
        printf("Counter: %d", count);
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("GPIO interrupt\n");

    // Configure GPIO input 2 for interrupt
    gpio_init(2) ;
    gpio_set_dir(2, GPIO_IN) ;
    gpio_pull_down(2) ;
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);


    while (1) {
        
    }

}