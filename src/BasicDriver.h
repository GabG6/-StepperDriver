#ifndef STEPPER_DRIVER_BASE_H
#define STEPPER_DRIVER_BASE_H
#include <stdio.h>
#include "pico/stdlib.h"

// used internally by the library to mark unconnected pins
#define PIN_UNCONNECTED -1
#define IS_CONNECTED(pin) (pin != PIN_UNCONNECTED)

//Define HIGH and LOW
#ifndef HIGH
#define HIGH true
#endif

#ifndef LOW
#define LOW false
#endif

/*
 * calculate the step pulse in microseconds for a given rpm value.
 * 60[s/min] * 1000000[us/s] / microsteps / steps / rpm
 */
#define STEP_PULSE(steps, microsteps, rpm) (60.0*1000000L/steps/microsteps/rpm)

/*
 * Basic Stepper Driver class.
 * Microstepping level should be externally controlled or hardwired.
 */
class BasicStepperDriver {
public:
    enum Mode {CONSTANT_SPEED, LINEAR_SPEED};
    enum State {STOPPED, ACCELERATING, CRUISING, DECELERATING};
    struct Profile {
        Mode mode = CONSTANT_SPEED;
        short accel = 1000;     // acceleration [steps/s^2]
        short decel = 1000;     // deceleration [steps/s^2]    
    };
    static inline void delayMicros(uint64_t delay_us, uint64_t start_us = 0){
        if (delay_us){
            if (!start_us){
                start_us = time_us_64();
            }

            // See https://www.gammon.com.au/millis
            while (time_us_64() - start_us < delay_us);
        }
    }

private:
    // calculation remainder to be fed into successive steps to increase accuracy (Atmel DOC8017)
    long rest;
    unsigned long last_action_end = 0;
    unsigned long next_action_interval = 0;

protected:
    /*
     * Motor Configuration
     */
    short motor_steps;           // motor steps per revolution (usually 200)

    /*
     * Driver Configuration
     */
    short dir_pin;
    short step_pin;
    short enable_pin = PIN_UNCONNECTED;
    bool enable_active_state = true;
    // Get max microsteps supported by the device
    virtual short getMaxMicrostep();
    // current microstep level (1,2,4,8,...), must be < getMaxMicrostep()
    short microsteps = 1;
    // tWH(STEP) pulse duration, STEP high, min value (us)
    static const int step_high_min = 1;
    // tWL(STEP) pulse duration, STEP low, min value (us)
    static const int step_low_min = 1;
    // tWAKE wakeup time, nSLEEP inactive to STEP (us)
    static const int wakeup_time = 0;

    float rpm = 0;

    /*
     * Movement state
     */
    struct Profile profile;

    long step_count = 0;        // current position
    long steps_remaining = 0;   // to complete the current move (absolute value)
    long steps_to_cruise = 0;   // steps to reach cruising (max) rpm
    long steps_to_brake = 0;    // steps needed to come to a full stop
    long step_pulse = 0;        // step pulse duration (microseconds)
    long cruise_step_pulse = 0; // step pulse duration for constant speed section (max rpm)

    // DIR pin state
    short dir_state = false;

    void calcStepPulse(void);

    // this is internal because one can call the start methods while CRUISING to get here
    void alterMove(long steps);

private:
    // microstep range (1, 16, 32 etc)
    static const short MAX_MICROSTEP = 128;

public:
    /*
     * Basic connection: DIR, STEP are connected.
     */
    BasicStepperDriver(short steps, short dir_pin, short step_pin);
    BasicStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin);
    /*
     * Initialize pins, calculate timings etc
     */
    void begin(float rpm=60, short microsteps=1);
    /*
     * Set current microstep level, 1=full speed, 32=fine microstepping
     * Returns new level or previous level if value out of range
     */
    virtual short setMicrostep(short microsteps);
    short getMicrostep(void){
        return microsteps;
    }
    short getSteps(void){
        return motor_steps;
    }
    /*
     * Set target motor RPM (1-200 is a reasonable range)
     */
    void setRPM(float rpm);
    float getRPM(void){
        return rpm;
    };
    float getCurrentRPM(void){
        return (60.0*1000000L / step_pulse / microsteps / motor_steps);
    }
    /*
     * Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated)
     * accel and decel are given in [full steps/s^2]
     */
    void setSpeedProfile(Mode mode, short accel=1000, short decel=1000);
    void setSpeedProfile(struct Profile profile);
    struct Profile getSpeedProfile(void){
        return profile;
    }
    short getAcceleration(void){
        return profile.accel;
    }
    short getDeceleration(void){
        return profile.decel;
    }
    /*
     * Move the motor a given number of steps.
     * positive to move forward, negative to reverse
     */
    void move(long steps);
    /*
     * Rotate the motor a given number of degrees (1-360)
     */
    void rotate(long deg);
    inline void rotate(int deg){
        rotate((long)deg);
    };
    /*
     * Rotate using a float or double for increased movement precision.
     */
    void rotate(double deg);
    /*
     * Configure which logic state on ENABLE pin means active
     * when using SLEEP (default) this is active HIGH
     */
    void setEnableActiveState(short state);
    /*
     * Turn off/on motor to allow the motor to be moved by hand/hold the position in place
     */
    virtual void enable(void);
    virtual void disable(void);

    void startMove(long steps, uint64_t time = 0);
    inline void startRotate(int deg){
        startRotate((long)deg);
    };
    void startRotate(long deg);
    void startRotate(double deg);

    /*
     * Toggle step at the right time and return time until next change is needed (microseconds)
     */
    long nextAction(void);

    /*
     * Optionally, call this to begin braking (and then stop) early
     * For constant speed, this is the same as stop()
     */
    void startBrake(void);

    /*
     * Immediate stop
     * Returns the number of steps remaining.
     */
    long stop(void);

    /*
     * State querying
     */
    enum State getCurrentState(void);

    /*
     * Get the number of completed steps so far.
     * This is always a positive number
     */
    long getStepsCompleted(void){
        return step_count;
    }

    /*
     * Get the number of steps remaining to complete the move
     * This is always a positive number
     */
    long getStepsRemaining(void){
        return steps_remaining;
    }

    /*
     * Get movement direction: forward +1, back -1
     */
    int getDirection(void){
        return (dir_state == true) ? 1 : -1;
    }

    /*
     * Return calculated time to complete the given move
     */
    long getTimeForMove(long steps);

    /*
     * Calculate steps needed to rotate requested angle, given in degrees
     */
    long calcStepsForRotation(long deg){
        return static_cast<long>(deg * motor_steps * microsteps / 360.0);
    }
    long calcStepsForRotation(double deg){
        return static_cast<long>(deg * motor_steps * microsteps / 360.0);
    }

};
#endif // STEPPER_DRIVER_BASE_H