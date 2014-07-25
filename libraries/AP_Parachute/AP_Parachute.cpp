// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Parachute.h>
#include <AP_Relay.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_Notify.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLED", 0, AP_Parachute, _enabled, 1),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value when parachute is released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value when parachute is not released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in cm above home bla bla bla
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: Meters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: PITCHROLL
    // @DisplayName: Pitch/roll threshold value in deg
    // @Description: Pitch/roll threshold.  Parachute will be released when this value is exceeded.  0 to disable this release mode.
    // @Range: 0 90
    // @Units: Degrees
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("PITCHROLL", 5, AP_Parachute, _pitchroll_thres, AP_PARACHUTE_PITCHROLL_DEFAULT),

    // @Param: FREEFALL
    // @DisplayName: Freefall acceleration value in m/s2
    // @Description: Freefall acceleration threshold.  Parachute will be released when this value is exceeded.  0 to disable this release mode.
    // @Range: 0 10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FREEFALL", 6, AP_Parachute, _freefall_thres, AP_PARACHUTE_FREEFALL_DEFAULT),

    // @Param: HDOT
    // @DisplayName: Vertical speed in m/s
    // @Description: Vertical speed threshold.  Parachute will be released when this value is exceeded.  0 to disable this release mode.
    // @Range: 0 25
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HDOT", 7, AP_Parachute, _hdot_thres, AP_PARACHUTE_HDOT_DEFAULT),

    // @Param: ALT_MAX
    // @DisplayName: Maximum allowable altitude in m
    // @Description: Maximum altitude threshold.  Parachute will be released after the aircraft altitude exceeds this value. 0 to disable this release mode.
    // @Range: 0 32000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("ALT_MAX", 8, AP_Parachute, _alt_max_thres, AP_PARACHUTE_ALT_MAX_DEFAULT),

    // @Param: DURATION
    // @DisplayName: Abnormal flight pattern threshold duration in ms
    // @Description: Maximum duration of abnormal flight pattern.  Parachute will be released after the specified amount of time has passed.
    // @Range: 100 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("DURATION", 9, AP_Parachute, _duration_thres, AP_PARACHUTE_DURATION_DEFAULT),

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    // set release time to current system time
    _release_time = hal.scheduler->millis();

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;

}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{

        //cliSerial->printf_P(PSTR("Climb rate: %f, roll: %lu (%lu), pitch: %lu (%lu), zacc: %f, throttle: %lu, case: %lu\n"),
        //cliSerial->printf_P(PSTR("bla bla");
    // exit immediately if not enabled or parachute not to be released
    //AP_Notify::flags.ok_flag = _release_time;
    if (_enabled <= 0 || _release_time == 0) {
        return;
    }


    // calc time since release
    uint32_t time_diff = hal.scheduler->millis() - _release_time;

    // check if we should release parachute
    if (!_released) {
        if (time_diff >= AP_PARACHUTE_RELEASE_DELAY_MS) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_on_pwm);
            }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _released = true;
        }
    }else if (time_diff >= AP_PARACHUTE_RELEASE_DELAY_MS + AP_PARACHUTE_RELEASE_DURATION_MS) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_off_pwm);
        }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _released = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}
