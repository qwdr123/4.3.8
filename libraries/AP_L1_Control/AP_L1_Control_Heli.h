#pragma once

/// @file    AP_L1_Control_Heli.h
/// @brief   L1 Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>

class AP_L1_Control_Heli {
public:
    AP_L1_Control_Heli(AP_AHRS &ahrs, const AP_SpdHgtControl *spdHgtControl)
        : _ahrs(ahrs)
        , _spdHgtControl(spdHgtControl)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* see AP_Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(void) const;
    float lateral_acceleration(void) const;

    // return the turn rate needed to achieve tracking from the last update_*() operation
    int32_t turn_rate_cds(void) const;

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const;

    float crosstrack_error(void) const { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const;
    float turn_distance(float wp_radius) const;
    float turn_distance(float wp_radius, float turn_angle) const;
    float loiter_radius (const float loiter_radius) const;
    void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min = 0.0f);
    void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction);
    void update_heading_hold(int32_t navigation_heading_cd);
    void update_level_flight(void);
    bool reached_loiter_target(void);
    void loiter_angle_reset(void);
    void loiter_angle_update(void);

    // get the total angle traveled while in loiter
    int32_t get_angle_total() { return loiter.sum_cd; }


    // set the default NAVL1_PERIOD
    void set_default_period(float period) {
        _L1_period.set_default(period);
    }

    void set_data_is_stale(void)  {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const {
        return _data_is_stale;
    }

    // this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) {
        _reverse = reverse;
    }

private:
    // reference to the AHRS object
    AP_AHRS &_ahrs;

    // pointer to the SpdHgtControl object
    const AP_SpdHgtControl *_spdHgtControl;

    // lateral acceration in m/s required to fly to the
    // L1 reference point (+ve to right)
    float _latAccDem;

    // L1 tracking distance in meters which is dynamically updated
    float _L1_dist;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // bearing angle (radians) to L1 point
    float _nav_bearing;

    // bearing error angle (radians) +ve to left of track
    float _bearing_error;

    // crosstrack error in meters
    float _crosstrack_error;

    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

    // L1 tracking loop period (sec)
    AP_Float _L1_period;
    // L1 tracking loop damping ratio
    AP_Float _L1_damping;
    // turn rate scale factor
    AP_Float _turn_rate_scale_factor;

    // previous value of cross-track velocity
    float _last_Nu;

    // prevent indecision in waypoint tracking
    void _prevent_indecision(float &Nu);

    // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
    // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
    float _L1_xtrack_i = 0;
    AP_Float _L1_xtrack_i_gain;
    float _L1_xtrack_i_gain_prev = 0;
    uint32_t _last_update_waypoint_us;
    bool _data_is_stale = true;

    float _groundSpeed;

    float _loiter_bank_limit;

    bool _reverse = false;
    float get_yaw();
    float get_yaw_sensor();

    /*
      meta data to support counting the number of circles in a loiter
    */
    struct {
        // previous target bearing, used to update sum_cd
        int32_t old_target_bearing_cd;

        // Total desired rotation in a loiter.  Used for Loiter Turns commands. 
        int32_t total_cd;

        // total angle completed in the loiter so far
        int32_t sum_cd;

        // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
        int8_t direction = 1;

        // start time of the loiter.  Milliseconds.
        uint32_t start_time_ms;

        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        uint32_t time_max_ms;
    } loiter;

};
