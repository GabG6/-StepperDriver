#include "BasicStepperDriver.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include <cmath>

/*
 * Min/Max functions which avoid evaluating the arguments multiple times.
 * See also https://github.com/arduino/Arduino/issues/2069
 */
template<typename T>
constexpr const T& stepperMin(const T& a, const T& b)
{
    return b < a ? b : a;
}

template<typename T>
constexpr const T& stepperMax(const T& a, const T& b)
{
    return a < b ? b : a;
}

/*
 * Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */
BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, PIN_UNCONNECTED)
{
}

BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), enable_pin(enable_pin)
{
	steps_to_cruise = 0;
	steps_remaining = 0;
	dir_state = false;
	steps_to_brake = 0;
	step_pulse = 0;
    cruise_step_pulse = 0;
	rest = 0;
	step_count = 0;
}

/*
 * Initialize pins, calculate timings etc
 */
void BasicStepperDriver::begin(float rpm, short microsteps){
    // Initialize DIR pin
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, HIGH); // Initial direction

    // Initialize STEP pin
    gpio_init(step_pin);
    gpio_set_dir(step_pin, GPIO_OUT);
    gpio_put(step_pin, LOW); // Initial step state

    // Initialize ENABLE pin if connected
    if (IS_CONNECTED(enable_pin)){
        gpio_init(enable_pin);
        gpio_set_dir(enable_pin, GPIO_OUT);
        disable();
    }


    this->rpm = rpm;
    setMicrostep(microsteps);

    enable();
}

/*
 * Set target motor RPM (1-200 is a reasonable range)
 */
void BasicStepperDriver::setRPM(float rpm){
    if (this->rpm == 0){        // begin() has not been called (old 1.0 code)
        begin(rpm, microsteps);
    }
    this->rpm = rpm;
}

/*
 * Set stepping mode (1:microsteps)
 * Allowed ranges for BasicStepperDriver are 1:1 to 1:128
 */
short BasicStepperDriver::setMicrostep(short microsteps){
    for (short ms=1; ms <= getMaxMicrostep(); ms<<=1){
        if (microsteps == ms){
            this->microsteps = microsteps;
            break;
        }
    }
    return this->microsteps;
}

/*
 * Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated)
 * accel and decel are given in [full steps/s^2]
 */
void BasicStepperDriver::setSpeedProfile(Mode mode, short accel, short decel){
    profile.mode = mode;
    profile.accel = accel;
    profile.decel = decel;
}
void BasicStepperDriver::setSpeedProfile(struct Profile profile){
    this->profile = profile;
}

/*
 * Move the motor a given number of steps.
 * positive to move forward, negative to reverse
 */
void BasicStepperDriver::move(long steps){
    startMove(steps);
    while (nextAction());
}
/*
 * Move the motor a given number of degrees (1-360)
 */
void BasicStepperDriver::rotate(long deg){
    move(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that using this function even once will add 1K to your program size
 * due to inclusion of float support.
 */
void BasicStepperDriver::rotate(double deg){
    move(calcStepsForRotation(deg));
}

/*
 * Set up a new move (calculate and save the parameters)
 */
void BasicStepperDriver::startMove(long steps, uint64_t time){
    float speed;
    // set up new move
    dir_state = (steps >= 0) ? true : false;
    last_action_end = 0;
    next_action_interval = 0;
    steps_remaining = std::abs(steps);
    step_count = 0;
    rest = 0;
    switch (profile.mode){
    case LINEAR_SPEED:
        // speed is in [steps/s]
        speed = rpm * motor_steps / 60.0f;
        if (time > 0){
            // Calculate a new speed to finish in the time requested
            float t = static_cast<float>(time) / 1e6f;                  // convert to seconds
            float d = static_cast<float>(steps_remaining) / static_cast<float>(microsteps);   // convert to full steps
            float a2 = 1.0f / profile.accel + 1.0f / profile.decel;
            float sqrt_candidate = t * t - 2.0f * a2 * d;  // in √b^2-4ac
            if (sqrt_candidate >= 0){
                speed = stepperMin(speed, (t - std::sqrt(sqrt_candidate)) / a2);
            };
        }
        // how many microsteps from 0 to target speed
        steps_to_cruise = static_cast<long>(microsteps * (speed * speed / (2.0f * profile.accel)));
        // how many microsteps are needed from cruise speed to a full stop
        steps_to_brake = static_cast<long>(steps_to_cruise * profile.accel / profile.decel);
        if (steps_remaining < (steps_to_cruise + steps_to_brake)){
            // cannot reach max speed, will need to brake early
            steps_to_cruise = static_cast<long>(steps_remaining * profile.decel / (profile.accel + profile.decel));
            steps_to_brake = steps_remaining - steps_to_cruise;
        }
        // Initial pulse (c0) including error correction factor 0.676 [us]
        step_pulse = static_cast<long>(1e6 * 0.676 * std::sqrt(2.0f / profile.accel / microsteps));
        // Save cruise timing since we will no longer have the calculated target speed later
        cruise_step_pulse = static_cast<long>(1e6 / speed / microsteps);
        break;

    case CONSTANT_SPEED:
    default:
        steps_to_cruise = 0;
        steps_to_brake = 0;
        step_pulse = cruise_step_pulse = STEP_PULSE(motor_steps, microsteps, rpm);
        if (time > static_cast<long>(steps_remaining * step_pulse)){
            step_pulse = static_cast<long>(static_cast<float>(time) / steps_remaining);
        }
    }
}
/*
 * Alter a running move by adding/removing steps
 * FIXME: This is a naive implementation and it only works well in CRUISING state
 */
void BasicStepperDriver::alterMove(long steps){
    switch (getCurrentState()){
    case ACCELERATING: // this also works but will keep the original speed target
    case CRUISING:
        if (steps >= 0){
            steps_remaining += steps;
        } else {
            steps_remaining = stepperMax(steps_to_brake, steps_remaining+steps);
        };
        break;
    case DECELERATING:
        // would need to start accelerating again -- NOT IMPLEMENTED
        break;
    case STOPPED:
        startMove(steps);
        break;
    }
}
/*
 * Brake early.
 */
void BasicStepperDriver::startBrake(void){
    switch (getCurrentState()){
    case CRUISING:  // this applies to both CONSTANT_SPEED and LINEAR_SPEED modes
        steps_remaining = steps_to_brake;
        break;

    case ACCELERATING:
        steps_remaining = step_count * profile.accel / profile.decel;
        break;

    default:
        break; // nothing to do if already stopped or braking
    }
}
/*
 * Stop movement immediately and return remaining steps.
 */
long BasicStepperDriver::stop(void){
    long retval = steps_remaining;
    steps_remaining = 0;
    return retval;
}
/*
 * Return calculated time to complete the given move
 */
long BasicStepperDriver::getTimeForMove(long steps){
    float t = 0.0f;
    long cruise_steps = 0;
    float speed = 0.0f;

    if (steps == 0){
        return 0;
    }
    switch (profile.mode){
        case LINEAR_SPEED:
            startMove(steps);
            cruise_steps = steps_remaining - steps_to_cruise - steps_to_brake;
            speed = rpm * motor_steps / 60,0f;   // full steps/s
            t = (cruise_steps / (microsteps * speed)) +
                std::sqrt(2.0f * steps_to_cruise / profile.accel / microsteps) +
                std::sqrt(2.0f * steps_to_brake / profile.decel / microsteps);
            t *= 1e6f; // seconds -> micros
            break;
        case CONSTANT_SPEED:
        default:
            t = steps * STEP_PULSE(motor_steps, microsteps, rpm);
    }
    return std::round(t);
}
/*
 * Move the motor an integer number of degrees (360 = full rotation)
 * This has poor precision for small amounts, since step is usually 1.8deg
 */
void BasicStepperDriver::startRotate(long deg){
    startMove(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that calling this function will increase program size substantially
 * due to inclusion of float support.
 */
void BasicStepperDriver::startRotate(double deg){
    startMove(calcStepsForRotation(deg));
}

/*
 * calculate the interval til the next pulse
 */
void BasicStepperDriver::calcStepPulse(void){
    if (steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }
    steps_remaining--;
    step_count++;

    if (profile.mode == LINEAR_SPEED){
        switch (getCurrentState()){
        case ACCELERATING:
            if (step_count < steps_to_cruise){
                step_pulse = step_pulse - (2*step_pulse+rest)/(4*step_count+1);
                rest = (2*step_pulse+rest) % (4*step_count+1);
            } else {
                // The series approximates target, set the final value to what it should be instead
                step_pulse = cruise_step_pulse;
                rest = 0;
            }
            break;

        case DECELERATING:
            step_pulse = step_pulse - (2*step_pulse+rest)/(-4*steps_remaining+1);
            rest = (2*step_pulse+rest) % (-4*steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}
/*
 * Yield to step control
 * Toggle step and return time until next change is needed (micros)
 */
long BasicStepperDriver::nextAction(void){
    if (steps_remaining > 0){
        delayMicros(next_action_interval, last_action_end);
        /*
         * DIR pin is sampled on rising STEP edge, so it is set first
         */
        gpio_put(dir_pin, dir_state);
        gpio_put(step_pin, true); 
        uint64_t m = time_us_64();
        long pulse = step_pulse; // save value because calcStepPulse() will overwrite it
        calcStepPulse();
        // We should pull HIGH for at least 1-2us (step_high_min)
        delayMicros(step_high_min);
        gpio_put(step_pin, false);
        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        last_action_end = time_us_64();
        m = last_action_end - m;
        next_action_interval = (pulse > static_cast<long>(m)) ? (pulse - static_cast<long?(m) : 1;
    } else {
        // end of move
        last_action_end = 0;
        next_action_interval = 0;
    }
    return next_action_interval;
}

enum BasicStepperDriver::State BasicStepperDriver::getCurrentState(void){
    enum State state;
    if (steps_remaining <= 0){
        state = STOPPED;
    } else {
        if (steps_remaining <= steps_to_brake){
            state = DECELERATING;
        } else if (step_count <= steps_to_cruise){
            state = ACCELERATING;
        } else {
            state = CRUISING;
        }
    }
    return state;
}
/*
 * Configure which logic state on ENABLE pin means active
 * when using SLEEP (default) this is active HIGH
 */
void BasicStepperDriver::setEnableActiveState(short state){
    enable_active_state = state;
}
/*
 * Enable/Disable the motor by setting a digital flag
 */
void BasicStepperDriver::enable(void){
    if (IS_CONNECTED(enable_pin)){
        gpio_put (enable_pin, enable_active_state);
    };
    delayMicros(2);
}

void BasicStepperDriver::disable(void){
    if (IS_CONNECTED(enable_pin)){
        gpio_put(enable_pin, (enable_active_state == true) ? false : true);
    }
}

short BasicStepperDriver::getMaxMicrostep(){
    return BasicStepperDriver::MAX_MICROSTEP;
}