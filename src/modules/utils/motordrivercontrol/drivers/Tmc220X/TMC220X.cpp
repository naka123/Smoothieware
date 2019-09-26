//
// Created by user on 26-Sep-19.
//

#include "TMC220X.h"
#include "mbed.h"
#include "StreamOutput.h"
#include "Kernel.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"
#include "StepTicker.h"


#define motor_driver_control_checksum  CHECKSUM("motor_driver_control")
#define sense_resistor_checksum        CHECKSUM("sense_resistor")
#define current_checksum        CHECKSUM("current")
#define microsteps_checksum        CHECKSUM("microsteps")

//some default values used in initialization
#define DEFAULT_MICROSTEPPING_VALUE 32
#define DEFAULT_CURRENT_VALUE 500
#define DEFAULT_RSENSE_VALUE 110

TMC220X::TMC220X(char d)
{
    designator = d;
    //we are not started yet
    started = false;
    //by default cool step is not enabled
    cool_step_enabled = false;
    error_reported.reset();

}

#undef SQUARE_WAVE_STEPPING

#define tmc2208_tx_pin_checksum            CHECKSUM("tx_pin")
#define tmc2208_rx_pin_checksum            CHECKSUM("rx_pin")


void TMC220X::init(uint16_t cs)
{

    THEKERNEL->streams->printf("TMC220X::init cs: %04X\n", cs);

    const uint32_t thrs = 255;
    const bool stealth = false;

    // read chip specific config entries
    this->resistor= THEKERNEL->config->value(motor_driver_control_checksum, cs, sense_resistor_checksum)->by_default(DEFAULT_RSENSE_VALUE)->as_number(); // in milliohms
    int mA= THEKERNEL->config->value(motor_driver_control_checksum, cs, current_checksum)->by_default(DEFAULT_CURRENT_VALUE)->as_number();
    microsteps= THEKERNEL->config->value(motor_driver_control_checksum, cs, microsteps_checksum)->by_default(DEFAULT_MICROSTEPPING_VALUE)->as_number();


    PinName rx_pin;
    PinName tx_pin;

    THEKERNEL->streams->printf("tx_pin: %s\n", THEKERNEL->config->value(motor_driver_control_checksum, cs, tmc2208_tx_pin_checksum)->by_default("nc")->as_string().c_str());
    THEKERNEL->streams->printf("rx_pin: %s\n", THEKERNEL->config->value(motor_driver_control_checksum, cs, tmc2208_rx_pin_checksum)->by_default("nc")->as_string().c_str());

    // preparing PinName objects from the config string
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(motor_driver_control_checksum, cs, tmc2208_tx_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        tx_pin= port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);

        smoothie_pin->from_string(THEKERNEL->config->value(motor_driver_control_checksum, cs, tmc2208_rx_pin_checksum)->by_default("nc")->as_string());
        if (smoothie_pin->connected() ) {
            smoothie_pin->as_input();
            rx_pin= port_pin((PortName) smoothie_pin->port_number, smoothie_pin->pin);
        } else {
            rx_pin= NC;
        }

        delete smoothie_pin;
    }

    THEKERNEL->streams->printf("tx_pin: %d, rx_pin: %d, NC: %d\n", tx_pin, rx_pin, NC);


    serial_stream= new SoftwareSerial(tx_pin, rx_pin);
    serial_stream->begin(19200);

    tmc2208Stepper = new TMC2208Stepper(serial_stream, this->resistor, false);

    TMC2208_n::GCONF_t gconf{0};

    gconf.pdn_disable = true; // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealth;
    tmc2208Stepper->GCONF(gconf.sr);
//    tmc2208Stepper->stored.stealthChop_enabled = stealth;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
#ifdef SQUARE_WAVE_STEPPING
    chopconf.dedge = true;
#endif
    tmc2208Stepper->CHOPCONF(chopconf.sr);

    tmc2208Stepper->rms_current(mA, HOLD_MULTIPLIER);
    tmc2208Stepper->microsteps(microsteps);
    tmc2208Stepper->iholddelay(10);
    tmc2208Stepper->TPOWERDOWN(128); // ~2s until driver lowers to hold current

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    tmc2208Stepper->PWMCONF(pwmconf.sr);

#ifdef HYBRID_THRESHOLD
    tmc2208Stepper->set_pwm_thrs(thrs);
#else
    UNUSED(thrs);
#endif

    tmc2208Stepper->GSTAT(0b111); // Clear
    tmc2208Stepper->push();
    delay(200);


started = true;

//#if 1
//    //set to a conservative start value
//    setConstantOffTimeChopper(7, 54, 13, 12, 1);
//#else
//    //void TMC26X::setSpreadCycleChopper( constant_off_time,  blank_time,  hysteresis_start,  hysteresis_end,  hysteresis_decrement);
//
//    // openbuilds high torque nema23 3amps (2.8)
//    setSpreadCycleChopper(5, 36, 6, 0, 0);
//    // for 1.5amp kysan @ 12v
//    setSpreadCycleChopper(5, 54, 5, 0, 0);
//    // for 4amp Nema24 @ 12v
//    //setSpreadCycleChopper(5, 54, 4, 0, 0);
//#endif

    setEnabled(false);

    //set a nice microstepping value
    setMicrosteps(DEFAULT_MICROSTEPPING_VALUE);

//    // set stallguard to a conservative value so it doesn't trigger immediately
//    setStallGuardThreshold(10, 1);
}


void TMC220X::dumpStatus(StreamOutput *stream, bool readable)
{
//    if (readable) {
        stream->printf("designator %c, Chip type TMC220X\n", designator);

        check_error_status_bits(stream);

        stream->printf("Current setting: %dmA\n", getCurrent());
//        stream->printf("Coolstep current: %dmA\n", getCoolstepCurrent());

        stream->printf("Microsteps: 1/%d\n", microsteps);

        stream->printf("Register dump:\n");
//        stream->printf(" driver control register: %08lX(%ld)\n", driver_control_register_value, driver_control_register_value);
//        stream->printf(" chopper config register: %08lX(%ld)\n", chopper_config_register, chopper_config_register);
//        stream->printf(" cool step register: %08lX(%ld)\n", cool_step_register_value, cool_step_register_value);
//        stream->printf(" stall guard2 current register: %08lX(%ld)\n", stall_guard2_current_register_value, stall_guard2_current_register_value);
//        stream->printf(" driver configuration register: %08lX(%ld)\n", driver_configuration_register_value, driver_configuration_register_value);
//        stream->printf(" motor_driver_control.xxx.reg %05lX,%05lX,%05lX,%05lX,%05lX\n", driver_control_register_value, chopper_config_register, cool_step_register_value, stall_guard2_current_register_value, driver_configuration_register_value);

//    } else {
//        // TODO hardcoded for X need to select ABC as needed
//        bool moving = THEROBOT->actuators[0]->is_moving();
//        // dump out in the format that the processing script needs
//        if (moving) {
//            stream->printf("#sg%d,p%lu,k%u,r,", getCurrentStallGuardReading(), THEROBOT->actuators[0]->get_current_step(), getCoolstepCurrent());
//        } else {
//            readStatus(TMC26X_READOUT_POSITION); // get the status bits
//            stream->printf("#s,");
//        }
//        stream->printf("d%d,", THEROBOT->actuators[0]->which_direction() ? -1 : 1);
//        stream->printf("c%u,m%d,", getCurrent(), getMicrosteps());
//        // stream->printf('S');
//        // stream->printf(tmc26XStepper.getSpeed(), DEC);
//        stream->printf("t%d,f%d,", getStallGuardThreshold(), getStallGuardFilter());
//
//        //print out the general cool step config
//        if (isCoolStepEnabled()) stream->printf("Ke+,");
//        else stream->printf("Ke-,");
//
//        stream->printf("Kl%u,Ku%u,Kn%u,Ki%u,Km%u,",
//                       getCoolStepLowerSgThreshold(), getCoolStepUpperSgThreshold(), getCoolStepNumberOfSGReadings(), getCoolStepCurrentIncrementSize(), getCoolStepLowerCurrentLimit());
//
//        //detect the winding status
//        if (isOpenLoadA()) {
//            stream->printf("ao,");
//        } else if(isShortToGroundA()) {
//            stream->printf("ag,");
//        } else {
//            stream->printf("a-,");
//        }
//        //detect the winding status
//        if (isOpenLoadB()) {
//            stream->printf("bo,");
//        } else if(isShortToGroundB()) {
//            stream->printf("bg,");
//        } else {
//            stream->printf("b-,");
//        }
//
//        char temperature = getOverTemperature();
//        if (temperature == 0) {
//            stream->printf("x-,");
//        } else if (temperature == TMC26X_OVERTEMPERATURE_PREWARING) {
//            stream->printf("xw,");
//        } else {
//            stream->printf("xe,");
//        }
//
//        if (isEnabled()) {
//            stream->printf("e1,");
//        } else {
//            stream->printf("e0,");
//        }
//
//        //write out the current chopper config
//        stream->printf("Cm%d,", (chopper_config_register & CHOPPER_MODE_T_OFF_FAST_DECAY) != 0);
//        stream->printf("Co%d,Cb%d,", constant_off_time, blank_time);
//        if ((chopper_config_register & CHOPPER_MODE_T_OFF_FAST_DECAY) == 0) {
//            stream->printf("Cs%d,Ce%d,Cd%d,", h_start, h_end, h_decrement);
//        }
//        stream->printf("\n");
//    }
}


bool TMC220X::checkAlarm()
{
    return check_error_status_bits(THEKERNEL->streams);
}


void TMC220X::setCurrent(unsigned int current)
{
    THEKERNEL->streams->printf("TMC220X::setCurrent current: %d\n", current);
    tmc2208Stepper->rms_current(current, HOLD_MULTIPLIER);
    tmc2208Stepper->push();
}

unsigned int TMC220X::getCurrent(void)
{
//    //we calculate the current according to the datasheet to be on the safe side
//    //this is not the fastest but the most accurate and illustrative way
//    double result = (double)(stall_guard2_current_register_value & CURRENT_SCALING_PATTERN);
//    double resistor_value = (double)this->resistor;
//    double voltage = (driver_configuration_register_value & VSENSE) ? 0.165F : 0.31F;
//    result = (result + 1.0F) / 32.0F * voltage / resistor_value * 1000.0F * 1000.0F;
    double result = 42;
    return (unsigned int)result;
}


/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
void TMC220X::setMicrosteps(int number_of_steps)
{
    tmc2208Stepper->microsteps(number_of_steps);
    tmc2208Stepper->push();
    microsteps = number_of_steps;
}

/*
 * returns the effective number of microsteps at the moment
 */
int TMC220X::getMicrosteps(void)
{
    return microsteps;
}

void TMC220X::setStepInterpolation(int8_t value)
{

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = value?INTERPOLATE:0;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
#ifdef SQUARE_WAVE_STEPPING
    chopconf.dedge = true;
#endif
    tmc2208Stepper->CHOPCONF(chopconf.sr);


    tmc2208Stepper->push();
}


// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
// FIXME status registers not reading back correctly, check docs
bool TMC220X::setRawRegister(StreamOutput *stream, uint32_t reg, uint32_t val)
{
    switch(reg) {
        case 255:
//            send262(driver_control_register_value);
//            send262(chopper_config_register);
//            send262(cool_step_register_value);
//            send262(stall_guard2_current_register_value);
//            send262(driver_configuration_register_value);
            stream->printf("Registers written\n");
            break;


        case 1: driver_control_register_value = val; stream->printf("driver control register set to %08lX\n", val); break;
        case 2: chopper_config_register = val; stream->printf("chopper config register set to %08lX\n", val); break;
        case 3: cool_step_register_value = val; stream->printf("cool step register set to %08lX\n", val); break;
        case 4: stall_guard2_current_register_value = val; stream->printf("stall guard2 current register set to %08lX\n", val); break;
        case 5: driver_configuration_register_value = val; stream->printf("driver configuration register set to %08lX\n", val); break;

        default:
            stream->printf("1: driver control register\n");
            stream->printf("2: chopper config register\n");
            stream->printf("3: cool step register\n");
            stream->printf("4: stall guard2 current register\n");
            stream->printf("5: driver configuration register\n");
            stream->printf("255: update all registers\n");
            return false;
    }
    return true;
}

void TMC220X::setEnabled(bool enabled)
{
    // todo
}

bool TMC220X::isEnabled()
{
    return true;
}


#define HAS(X) (options.find(X) != options.end())
#define GET(X) (options.at(X))
bool TMC220X::set_options(const options_t& options)
{
    bool set = false;
    if(HAS('O') || HAS('Q')) {
        // void TMC26X::setStallGuardThreshold(int8_t stall_guard_threshold, int8_t stall_guard_filter_enabled)
//        int8_t o = HAS('O') ? GET('O') : getStallGuardThreshold();
//        int8_t q = HAS('Q') ? GET('Q') : getStallGuardFilter();
//        setStallGuardThreshold(o, q);
        set = true;
    }

    if(HAS('H') && HAS('I') && HAS('J') && HAS('K') && HAS('L')) {
        //void TMC26X::setCoolStepConfiguration(unsigned int lower_SG_threshold, unsigned int SG_hysteresis, uint8_t current_decrement_step_size, uint8_t current_increment_step_size, uint8_t lower_current_limit)
//        setCoolStepConfiguration(GET('H'), GET('I'), GET('J'), GET('K'), GET('L'));
        set = true;
    }

    if(HAS('S')) {
        uint32_t s = GET('S');
        if(s == 0 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //void TMC26X::setConstantOffTimeChopper(int8_t constant_off_time, int8_t blank_time, int8_t fast_decay_time_setting, int8_t sine_wave_offset, uint8_t use_current_comparator)
//            setConstantOffTimeChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 1 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //void TMC26X::setSpreadCycleChopper(int8_t constant_off_time, int8_t blank_time, int8_t hysteresis_start, int8_t hysteresis_end, int8_t hysteresis_decrement);
//            setSpreadCycleChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 2 && HAS('Z')) {
//            setRandomOffTime(GET('Z'));
            set = true;

        } else if(s == 3 && HAS('Z')) {
//            setDoubleEdge(GET('Z'));
            set = true;

        } else if(s == 4 && HAS('Z')) {
            setStepInterpolation(GET('Z'));
            set = true;

        } else if(s == 5 && HAS('Z')) {
//            setCoolStepEnabled(GET('Z') == 1);
            set = true;
        }
    }

    return set;
}


// check error bits and report, only report once
bool TMC220X::check_error_status_bits(StreamOutput *stream)
{
    bool error= false;
//    readStatus(TMC26X_READOUT_POSITION); // get the status bits

//    if (this->getOverTemperature()&TMC26X_OVERTEMPERATURE_PREWARING) {
//        if(!error_reported.test(0)) stream->printf("%c - WARNING: Overtemperature Prewarning!\n", designator);
//        error_reported.set(0);
//    }else{
//        error_reported.reset(0);
//    }
//
//    if (this->getOverTemperature()&TMC26X_OVERTEMPERATURE_SHUTDOWN) {
//        if(!error_reported.test(1)) stream->printf("%c - ERROR: Overtemperature Shutdown!\n", designator);
//        error=true;
//        error_reported.set(1);
//    }else{
//        error_reported.reset(1);
//    }

//    if (this->isShortToGroundA()) {
//        if(!error_reported.test(2)) stream->printf("%c - ERROR: SHORT to ground on channel A!\n", designator);
//        error=true;
//        error_reported.set(2);
//    }else{
//        error_reported.reset(2);
//    }

//    if (this->isShortToGroundB()) {
//        if(!error_reported.test(3)) stream->printf("%c - ERROR: SHORT to ground on channel B!\n", designator);
//        error=true;
//        error_reported.set(3);
//    }else{
//        error_reported.reset(3);
//    }

//    // these seem to be triggered when moving so ignore them for now
//    if (this->isOpenLoadA()) {
//        if(!error_reported.test(4)) stream->printf("%c - ERROR: Channel A seems to be unconnected!\n", designator);
//        error=true;
//        error_reported.set(4);
//    }else{
//        error_reported.reset(4);
//    }

//    if (this->isOpenLoadB()) {
//        if(!error_reported.test(5)) stream->printf("%c - ERROR: Channel B seems to be unconnected!\n", designator);
//        error=true;
//        error_reported.set(5);
//    }else{
//        error_reported.reset(5);
//    }

    // if(error) {
    //     stream->printf("%08X\n", driver_status_result);
    // }
    return error;
}
