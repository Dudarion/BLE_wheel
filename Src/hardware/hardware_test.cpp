#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"

#define LED_PIN 13

int main(void){
    NRF_P0->DIRSET = (1 << LED_PIN); // Output mode

    while(true){
        NRF_P0->OUTSET = (1 << LED_PIN); // SET 1
        nrf_delay_ms(200);

        NRF_P0->OUTCLR = (1 << LED_PIN); // RESET 
        nrf_delay_ms(200);
    }
}
