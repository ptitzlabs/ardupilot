// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Parachute.h
/// @brief	Parachute release library

#ifndef AP_PARACHUTE_H
#define AP_PARACHUTE_H

#include <AP_Param.h>
#include <AP_Common.h>
#include <AP_Relay.h>

#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_0       0
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_1       1
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_2       2
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_3       3
#define AP_PARACHUTE_TRIGGER_TYPE_SERVO         10

#define AP_PARACHUTE_RELEASE_DELAY_MS           500    // delay in milliseconds between call to release() and when servo or relay actually moves.  Allows for warning to user
#define AP_PARACHUTE_RELEASE_DURATION_MS       1000    // when parachute is released, servo or relay stay at their released position/value for 1000ms (1second)

#define AP_PARACHUTE_SERVO_ON_PWM_DEFAULT      1300    // default PWM value to move servo to when shutter is activated
#define AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT     1100    // default PWM value to move servo to when shutter is deactivated

#define AP_PARACHUTE_ALT_MIN_DEFAULT            0     // default min altitude the vehicle should have before parachute is released

#define AP_PARACHUTE_PITCHROLL_DEFAULT          50     // Maximum pitch/roll in flight (deg)
#define AP_PARACHUTE_FREEFALL_DEFAULT           5      // Freefall detection threshold (m/s2)
#define AP_PARACHUTE_HDOT_DEFAULT               10     // Maximum climb/descent rate   (m/s)
#define AP_PARACHUTE_ALT_MAX_DEFAULT            250    // Maximum allowed altitude (m)
#define AP_PARACHUTE_DURATION_DEFAULT           1000   // Abnormality duration (us)

/// @class	AP_Parachute
/// @brief	Class managing the release of a parachute
class AP_Parachute {

public:

    /// Constructor
    AP_Parachute(AP_Relay& relay) :
        _relay(relay),
        _release_time(0),
        _released(false)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    /// enabled - enable or disable parachute release
    void enabled(bool on_off);

    /// enabled - returns true if parachute release is enabled
    bool enabled() const { return _enabled; }

    /// release - release parachute
    void release();

    /// update - shuts off the trigger should be called at about 10hz
    void update();

    /// alt_min - returns the min altitude above home the vehicle should have before parachute is released
    ///   0 = altitude check disabled
    int16_t alt_min() const { return _alt_min; }

    /// Return custom release conditions
    int8_t pitchroll_thres() const { return _pitchroll_thres; }
    int8_t freefall_thres() const { return _freefall_thres; }
    int8_t hdot_thres() const { return _hdot_thres; }
    int16_t alt_max_thres() const {return _alt_max_thres; }
    int16_t duration_thres() const { return _duration_thres; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _release_type;  // 0:Servo,1:Relay
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated
    AP_Int16    _alt_min;       // min altitude the vehicle should have before parachute is released
    AP_Int8     _pitchroll_thres;// Maximum pitch/roll in flight (deg)
    AP_Int8     _freefall_thres; // Freefall acceleration threshold (m/s2)
    AP_Int8     _hdot_thres;        // Maximum climb/descent rate (m/s)
    AP_Int16    _alt_max_thres;           // Maximum allowed altitude (m)
    AP_Int16    _duration_thres;    // Abnormality duration threshold (us)

    // internal variables
    AP_Relay   &_relay;         // pointer to relay object from the base class Relay. The subclasses could be AP_Relay_APM1 or AP_Relay_APM2
    uint32_t    _release_time;  // system time that parachute is ordered to be released (actual release will happen 0.5 seconds later)
    bool        _released;      // true if the parachute has been released
};

#endif /* AP_PARACHUTE_H */
