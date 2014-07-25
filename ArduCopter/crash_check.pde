/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to detect a crash main ArduCopter code
#ifndef CRASH_CHECK_ITERATIONS_MAX
 # define CRASH_CHECK_ITERATIONS_MAX        20      // 2 second (ie. 10 iterations at 10hz) inverted indicates a crash
#endif
#ifndef CRASH_CHECK_ANGLE_DEVIATION_CD
 # define CRASH_CHECK_ANGLE_DEVIATION_CD    2000    // 20 degrees beyond angle max is signal we are inverted
#endif
#ifndef CRASH_CHECK_ALT_CHANGE_LIMIT_CM
 # define CRASH_CHECK_ALT_CHANGE_LIMIT_CM   50      // baro altitude must not change by more than 50cm
#endif

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// should be called at 10hz
void crash_check()
{
    static uint8_t inverted_count;  // number of iterations we have been inverted
    static int32_t baro_alt_prev;



#if PARACHUTE == ENABLED
    // check parachute
    parachute_check();
#endif

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors.armed() || (g.rc_3.control_in != 0 && !failsafe.radio)) {

        inverted_count = 0;
        return;
    }

//    cliSerial->print_P(PSTR("1"));
    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || control_mode == FLIP) {
        inverted_count = 0;
        return;
    }

    // check angles
    int32_t lean_max = aparm.angle_max + CRASH_CHECK_ANGLE_DEVIATION_CD;
    if (labs(ahrs.roll_sensor) > lean_max || labs(ahrs.pitch_sensor) > lean_max) {
        inverted_count++;

        // if we have just become inverted record the baro altitude
        if (inverted_count == 1) {
            baro_alt_prev = baro_alt;
		//cliSerial->print_P(baro_alt);

        // exit if baro altitude change indicates we are moving (probably falling)
        }else if (labs(baro_alt - baro_alt_prev) > CRASH_CHECK_ALT_CHANGE_LIMIT_CM) {
            inverted_count = 0;
            return;

        // check if inverted for 2 seconds
        }else if (inverted_count >= CRASH_CHECK_ITERATIONS_MAX) {
            // log an error in the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
            // send message to gcs
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Crash: Disarming"));
            // disarm motors
            init_disarm_motors();
        }
    }else{
        // we are not inverted so reset counter
        inverted_count = 0;
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#ifndef PARACHUTE_CHECK_ITERATIONS_MAX
 # define PARACHUTE_CHECK_ITERATIONS_MAX        10      // 1 second (ie. 10 iterations at 10hz) of loss of control triggers the parachute
#endif
#ifndef PARACHUTE_CHECK_ANGLE_DEVIATION_CD
 # define PARACHUTE_CHECK_ANGLE_DEVIATION_CD    3000    // 30 degrees off from target indicates a loss of control
#endif

// parachute_check - disarms motors and triggers the parachute if serious loss of control has been detected
// vehicle is considered to have a "serious loss of control" by the vehicle being more than 30 degrees off from the target roll and pitch angles continuously for 1 second
// should be called at 10hz
void parachute_check()
{
    static uint8_t control_loss_count;	// number of iterations we have been out of control
    static int32_t baro_alt_start;

    static uint8_t tick_count;
    static uint8_t crash_case;

    const Vector3f& target_angle = attitude_control.angle_ef_targets();


    if (tick_count == 0 && motors.armed())
    {
        //cliSerial->printf_P(PSTR("Climb rate: %f (%lu), roll: %f (%lu), pitch: %f (%lu), zacc: %f, throttle: %lu, crash_case: %f\n"),
                //climb_rate/100.0f,
                //parachute.hdot_thres(),
                //((ahrs.roll_sensor/100) % 360)*1.0f,
                //parachute.pitchroll_thres(),
                //((ahrs.pitch_sensor/100) % 360)*1.0f,
                //parachute.pitchroll_thres(),
                //ins.get_accel().z / GRAVITY_MSS,
                //g.rc_3.servo_out/10,
                //crash_case*1.0f);

    }

    //cliSerial->printf_P(PSTR("flags.parachute_release = %lu "),AP_Notify::flags.parachute_release);
    //cliSerial->printf_P(PSTR("ok_flag = %lu\n"),AP_Notify::flags.ok_flag);
    if (motors.armed()){
    
        if (parachute.hdot_thres() != 0)
        {
            if (climb_rate/100.0f < -parachute.hdot_thres() || climb_rate/100.0f > parachute.hdot_thres()) crash_case = 1;
        }
        if (parachute.freefall_thres() !=0)
        {
            if (ins.get_accel().z*1.0f > -parachute.freefall_thres()) crash_case = 2;
        }
        if (parachute.pitchroll_thres() != 0)
        {
            if (((labs(ahrs.roll_sensor)/100) % 360)*1.0f > parachute.pitchroll_thres() || ((labs(ahrs.pitch_sensor)/100)
            %360)*1.0f > parachute.pitchroll_thres() ) crash_case = 3;
        }

        if (g.rc_3.servo_out/10 <= 13 && (baro_alt > (uint32_t)parachute.alt_min() * 100))
        {
            if(labs(ins.get_accel().x)>1 || 
                labs(ins.get_accel().y)>1 || 
                labs(ins.get_accel().z + GRAVITY_MSS)>1) crash_case = 4;
        }
        if (baro_alt > parachute.alt_max_thres()*100) crash_case = 5;
        //if (labs(ahrs.roll_sensor - target_angle.x) > CRASH_CHECK_ANGLE_DEVIATION_CD ||
            //labs(ahrs.pitch_sensor - target_angle.y) > CRASH_CHECK_ANGLE_DEVIATION_CD) crash_case = 0;

    }
        
    tick_count++;
    //return;
    if (tick_count == 2)
    {
        tick_count = 0;
    }
    //parachute.enabled(true);
    parachute.update();
    if (!parachute.enabled()) {
        return;
    }

    if (crash_case > 0 && motors.armed()) {
            control_loss_count++;
            //cliSerial->printf_P(PSTR("%lu\n"),control_loss_count);
            if (control_loss_count > parachute.duration_thres()/100) {
                control_loss_count = parachute.duration_thres()/100;
            }

            if (control_loss_count == PARACHUTE_CHECK_ITERATIONS_MAX) {
                

                switch (crash_case) {
                    case 0:
                        break;
                    case 1:
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("Climb or descent rate too high"));
                        break;
                    case 2:
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("Vehicle in freefall"));
                        break;
                    case 3:
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("Attitude or roll too high"));
                        break;
                    case 4:
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("Motors off in flight"));
                        break;
                    case 5:
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("Altitude too high"));
                        break;
                }
                control_loss_count = 0;
                crash_case = 0;
                parachute_release();
                //crash_case = 0;
                return;

            }
    } else {
        control_loss_count = 0;
        return;
    }
}

//static uint8_t parachute_auto()
//{
    //static uint8_t tick_count;
    //static uint8_t crash_case;

    //const Vector3f& target_angle = attitude_control.angle_ef_targets();


    //if (tick_count == 0)
    //{
        //cliSerial->printf_P(PSTR("Climb rate: %f, roll: %lu (%lu), pitch: %lu (%lu), zacc: %f, throttle: %lu, case: %lu\n"),
        //climb_rate/100.0f,
        //(ahrs.roll_sensor/100) % 360,
        //parachute.pitchroll_thres(),
        //(ahrs.pitch_sensor/100) % 360,
        //parachute.pitchroll_thres(),
        //ins.get_accel().z / GRAVITY_MSS,
        //g.rc_3.servo_out/10,
        //crash_case);
    //}
    
    //if (climb_rate/100.0f < -parachute.hdot_thres()*1.0f || climb_rate/100.0f > parachute.hdot_thres()*1.0f) crash_case = 1;
    //if (ins.get_accel().z > -parachute.freefall_thres()) crash_case = 2;
    //if ((labs(ahrs.roll_sensor)/100) % 360 > parachute.pitchroll_thres() || (labs(ahrs.pitch_sensor)/100) %360 >
        //parachute.pitchroll_thres() ) crash_case = 3;
    //if (motors.armed() && g.rc_3.servo_out/10 < 2)
    //{
        //if(labs(ins.get_accel().x / GRAVITY_MSS)>1.1 || 
            //labs(ins.get_accel().y / GRAVITY_MSS)>1.1 || 
            //labs(ins.get_accel().z / GRAVITY_MSS)>1.1) crash_case = 4;
    //}
    //if (labs(ahrs.roll_sensor - target_angle.x) > CRASH_CHECK_ANGLE_DEVIATION_CD ||
        //labs(ahrs.pitch_sensor - target_angle.y) > CRASH_CHECK_ANGLE_DEVIATION_CD) crash_case = 5;
            


    



    //tick_count++;
    ////return;
    //if (tick_count == 10)
    //{
        //tick_count = 0;
    //}

    //return crash_case;
//}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
static void parachute_release()
{
    // send message to gcs and dataflash
    gcs_send_text_P(SEVERITY_HIGH,PSTR("Parachute: Released!"));
    Log_Write_Event(DATA_PARACHUTE_RELEASED);

    // disarm motors
    init_disarm_motors();

    // release parachute
    parachute.release();
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
static void parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    //if (ap.land_complete || (parachute.alt_min() != 0 && (baro_alt < (uint32_t)parachute.alt_min() * 100))) {
    if ((parachute.alt_min() != 0 && (baro_alt < (uint32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Parachute: Too Low"));
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_TOO_LOW);
        return;
    }

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
