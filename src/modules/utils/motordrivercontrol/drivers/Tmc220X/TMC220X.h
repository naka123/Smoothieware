//
// Created by user on 26-Sep-19.
//

#pragma once

#include <functional>
#include <map>
#include <bitset>

#include "libs/TMCStepper/TMCStepper.h"
#include "libs/_compat/Marduino_compat.h"
#include "libs/_compat/SoftwareSerial_compat.h"

class StreamOutput;


#define CHOPPER_DEFAULT_12V  { 3, -1, 1 }
#define CHOPPER_DEFAULT_19V  { 4,  1, 1 }
#define CHOPPER_DEFAULT_24V  { 4,  2, 1 }
#define CHOPPER_DEFAULT_36V  { 5,  2, 4 }
#define CHOPPER_PRUSAMK3_24V { 3, -2, 6 }
#define CHOPPER_MARLIN_119   { 5,  2, 3 }

#define CHOPPER_TIMING CHOPPER_DEFAULT_24V
#define SQUARE_WAVE_STEPPING 1
//#define HYBRID_THRESHOLD 1
#define HAS_STEALTHCHOP 1

#define HOLD_MULTIPLIER    0.66  // Scales down the holding current from run current
#define INTERPOLATE       true  // Interpolate X/Y/Z_MICROSTEPS to 256

typedef struct {
    uint8_t toff;
    int8_t hend;
    uint8_t hstrt;
} chopper_timing_t;

static constexpr chopper_timing_t chopper_timing = CHOPPER_TIMING;

/*!
 * \class TMC220X
 * \brief Class representing a TMC220X stepper driver
 */
class TMC220X {
public:

    TMC220X(char designator);

    void init(uint16_t cs);

    void setMicrosteps(int number_of_steps);
    int getMicrosteps(void);

    void setStepInterpolation(int8_t value);

    void setEnabled(bool enabled);
    bool isEnabled();


    void setCurrent(unsigned int current);
    unsigned int getCurrent(void);

    void dumpStatus(StreamOutput *stream, bool readable= true);
    bool setRawRegister(StreamOutput *stream, uint32_t reg, uint32_t val);
    bool checkAlarm();

    using options_t= std::map<char,int>;

    bool set_options(const options_t& options);

private:

    SoftwareSerial *serial_stream;
    TMC2208Stepper *tmc2208Stepper;

    //helper routione to get the top 10 bit of the readout
    inline int getReadoutValue();
    bool check_error_status_bits(StreamOutput *stream);

    // SPI sender
//    inline void send262(unsigned long datagram);
//    std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;

    float resistor; // current sense resitor value in ohm

    //driver control register copies to easily set & modify the registers

    TMC2208_n::GCONF_t gconf; //{0};
    TMC2208_n::CHOPCONF_t chopconf; //{0};
    TMC2208_n::PWMCONF_t pwmconf; //{0};

    //the driver status result
    unsigned long driver_status_result;

    //status values
    int microsteps; //the current number of micro steps

    std::bitset<8> error_reported;

    // only beeded for the tuning app report
    struct {
        int8_t blank_time:8;
        int8_t constant_off_time:5; //we need to remember this value in order to enable and disable the motor
        int8_t h_start:4;
        int8_t h_end:4;
        int8_t h_decrement:3;
        bool cool_step_enabled:1; //we need to remember this to configure the coolstep if it si enabled
        bool started:1; //if the stepper has been started yet
    };
    char designator;

};

