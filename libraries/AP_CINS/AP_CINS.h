/*
  CINS state estimator for AP_AHRS
 */

#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/LieGroups.h>
#include <AP_Common/Location.h>

#include <deque>

class AP_CINS {
public:
    AP_CINS();
    void init(void);
    void update();

    struct {
        AP_Float gpspos_att;
        AP_Float gpsvel_att;
        AP_Float gps_lag;
        AP_Float gps_pos;
        AP_Float gps_vel;
        AP_Float mag_att;
        AP_Float Q11;
        AP_Float Q22;
        AP_Float gps_pos_gyr_bias;
        AP_Float gps_vel_gyr_bias;
        AP_Float mag_gyr_bias;
        AP_Float sat_gyr_bias;
        AP_Float gps_pos_acc_bias;
        AP_Float gps_vel_acc_bias;
        AP_Float mag_acc_bias;
        AP_Float sat_acc_bias;
    } gains;

    Vector3f get_accel() const {
        return state.accel.tofloat();
    }
    Vector3f get_gyro() const {
        return state.gyro.tofloat();
    }
    Quaternion get_quat() const {
        QuaternionF quat;
        quat.from_rotation_matrix(state.XHat.rot());
        return quat.tofloat();
    }
    Location get_location() const {
        Location loc = state.origin;
        loc.offset(state.XHat.pos().x, state.XHat.pos().y);
        loc.alt -= state.XHat.pos().z * 100;
        return loc;
    }
    Vector3f get_velocity() const {
        return state.XHat.vel().tofloat();
    }
    bool healthy(void) const {
        return state.have_origin;
    }
    bool get_origin(Location &loc) {
        loc = state.origin;
        return state.have_origin;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt);
    void update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt);
    void update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt);
    void update_attitude_from_compass();
    bool init_yaw(void);
    bool get_compass_yaw(ftype &yaw_rad, ftype &dt);
    bool get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt);

    void compute_bias_update_compass(const Vector3F& mag_tru, const Vector3F& mag_ref, const ftype& dt);
    void compute_bias_update_imu(const SIM23& Gamma);
    void saturate_bias(Vector3F& bias_correction, const Vector3F& current_bias, const ftype& saturation, const ftype& dt) const;

    struct {
        Vector3F accel;
        Vector3F gyro;
        Location origin;
        bool have_origin;

        //XHat and ZHat for CINS
        Gal3F XHat; // State estimate containing attitude, velocity, position
        SIM23 ZHat; // Auxiliary state for estimator maths

        // Gyro Bias
        Vector3F gyr_bias;
        Vector3F gyr_bias_correction;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } gyr_bias_gain_mat;

        // Accel Bias
        Vector3F acc_bias;
        Vector3F acc_bias_correction;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } acc_bias_gain_mat;
    } state;

    // buffers for storing old position and velocity data with timestamps.
    // used to compensate for the GNSS pos and vel measurement delay.
    struct {
        ftype internal_time = 0.;
        std::deque<std::pair<ftype,Vector3F>> stamped_pos;
        std::deque<std::pair<ftype,Vector3F>> stamped_vel;
    } buffers;

    uint32_t last_gps_update_ms;
    bool done_yaw_init;
    uint32_t last_mag_us;
};

