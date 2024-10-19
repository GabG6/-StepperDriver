#include "A4988.h"
#include <cmath>

/*
 * Microstepping resolution truth table (Page 6 of A4988 pdf)
 * 0bMS3,MS2,MS1 for 1,2,4,8,16 microsteps
 */
const uint8_t A4988::MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b111};

/*
 * Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */
A4988::A4988(short steps, short dir_pin, short step_pin)
:BasicStepperDriver(steps, dir_pin, step_pin)
{}

A4988::A4988(short steps, short dir_pin, short step_pin, short enable_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, enable_pin)
{}

/*
 * Fully wired.
 * All the necessary control pins for A4988 are connected.
 */
A4988::A4988(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin)
:BasicStepperDriver(steps, dir_pin, step_pin),
    ms1_pin(ms1_pin), ms2_pin(ms2_pin), ms3_pin(ms3_pin)
{}

A4988::A4988(short steps, short dir_pin, short step_pin, short enable_pin, short ms1_pin, short ms2_pin, short ms3_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, enable_pin),
ms1_pin(ms1_pin), ms2_pin(ms2_pin), ms3_pin(ms3_pin)
{}

void A4988::begin(float rpm, short microsteps){
    BasicStepperDriver::begin(rpm, microsteps);

    if (!IS_CONNECTED(ms1_pin) || !IS_CONNECTED(ms2_pin) || !IS_CONNECTED(ms3_pin)){
        return;
    }

    gpio_init(ms1_pin);
    gpio_set_dir(ms1_pin, GPIO_OUT);
    gpio_init(ms2_pin);
    gpio_set_dir(ms2_pin, GPIO_OUT);
    gpio_init(ms3_pin);
    gpio_set_dir(ms3_pin, GPIO_OUT);   
}

/*
 * Set microstepping mode (1:divisor)
 * Allowed ranges for A4988 are 1:1 to 1:16
 * If the control pins are not connected, we recalculate the timing only
 */
short A4988::setMicrostep(short microsteps){
    BasicStepperDriver::setMicrostep(microsteps);

    if (!IS_CONNECTED(ms1_pin) || !IS_CONNECTED(ms2_pin) || !IS_CONNECTED(ms3_pin)){
        return this->microsteps;
    }

    const uint8_t* ms_table = getMicrostepTable();
    size_t ms_table_size = getMicrostepTableSize();

    unsigned short i = 0;
    while (i < ms_table_size){
        if (this->microsteps & (1<<i)){
            uint8_t mask = ms_table[i];
                gpio_put(ms3_pin, (mask & 0b100) ? true : false);
                gpio_put(ms2_pin, (mask & 0b010) ? true : false);
                gpio_put(ms1_pin, (mask & 0b001) ? true : false);
            break;
        }
        i++;
    }
    return this->microsteps;
}

const uint8_t* A4988::getMicrostepTable(){
    return A4988::MS_TABLE;
}

size_t A4988::getMicrostepTableSize(){
    return sizeof(A4988::MS_TABLE) / sizeof(A4988::MS_TABLE[0]);
}

short A4988::getMaxMicrostep(){
    return A4988::MAX_MICROSTEP;
}