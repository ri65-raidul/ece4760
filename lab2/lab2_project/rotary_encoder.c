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
#include "VGA/vga16_graphics_v3.h"
// #include "pico/stdlib.h"
// #include "hardware/pio.h"
// #include "hardware/dma.h"

// #include "hardware/vreg.h"
// #include "hardware/clocks.h"



volatile int count = 0;
char buffer[64];

// GPIO ISR. Toggles LED
void gpio_callback(uint gpio, uint32_t event_mask) {
    gpio_put(25, !gpio_get(25));
    if (gpio_get(3)) {
        // Counter Clockwise
        printf("Count is being decremented");
        count--;
        printf("Counter = %d", count);
        // sprintf(buffer, "Counter: %d", count);
        // drawTextVGA437(260, 450, buffer, WHITE, BLACK) ;
    } else {
        // clockwise
        printf("Count is being incremented");
        count++;
        printf("Counter: %d", count);
        // sprintf(buffer, "Counter: %d", count);
        // drawTextVGA437(260, 450, buffer, WHITE, BLACK) ;

    }
}

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("GPIO interrupt\n");


    //initVGA();
    // Configure GPIO input 2 for interrupt
    gpio_init(2) ;
    gpio_init(3) ;
    gpio_init(25) ;

    gpio_set_dir(2, GPIO_IN) ;
    gpio_set_dir(3, GPIO_IN) ;
    gpio_set_dir(25,GPIO_OUT);

    gpio_pull_down(2) ;
    gpio_pull_down(3) ;

    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
    
   while(1){
    }   

}