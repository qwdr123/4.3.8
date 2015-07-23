/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_HMC5843.cpp - Arduino Library for HMC5843 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Compass_HMC5843.h"

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

// constructor
AP_Compass_HMC5843::AP_Compass_HMC5843(Compass &compass):
    AP_Compass_Backend(compass),
    _retry_time(0),
    _i2c_sem(NULL),
    _mag_x(0),
    _mag_y(0),
    _mag_z(0),
    _mag_x_accum(0),
    _mag_y_accum(0),
    _mag_z_accum(0),
    _accum_count(0),
    _last_accum_time(0),
    _compass_instance(0),
    _product_id(0)
{}

// detect the sensor
AP_Compass_Backend *AP_Compass_HMC5843::detect(Compass &compass)
{
    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

// read_register - read a register value
bool AP_Compass_HMC5843::read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_HMC5843::write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_HMC5843::read_raw()
{
    uint8_t buff[6];

    if (hal.i2c->readRegisters(COMPASS_ADDRESS, 0x03, 6, buff) != 0) {
        hal.i2c->setHighSpeed(false);
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[0]) << 8) | buff[1];
    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        rz = (((int16_t)buff[2]) << 8) | buff[3];
        ry = (((int16_t)buff[4]) << 8) | buff[5];
    } else {
        ry = (((int16_t)buff[2]) << 8) | buff[3];
        rz = (((int16_t)buff[4]) << 8) | buff[5];
    }
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_HMC5843::accumulate(void)
{
    if (!_initialised) {
        /* someone has tried to enable a compass for the first time
         * mid-flight .... we can't do that yet (especially as we won't
         * have the right orientation!) */

        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   if (_accum_count != 0 && (tnow - _last_accum_time) < 13333) {
      // the compass gets new data at 75Hz
      return;
   }

   if (!_i2c_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _i2c_sem->give();

   if (result) {

      /* the _mag_N values are in the range -2048 to 2047, so we can
       * accumulate up to 15 of them in an int16_t. Let's make it 14
       * for ease of calculation. We expect to do reads at 10Hz, and
       * we get new data at most 75Hz, so we don't expect to
       * accumulate more than 8 before a read */

      _mag_x_accum += _mag_x;
      _mag_y_accum += _mag_y;
      _mag_z_accum += _mag_z;
      _accum_count++;
      if (_accum_count == 14) {
         _mag_x_accum /= 2;
         _mag_y_accum /= 2;
         _mag_z_accum /= 2;
         _accum_count = 7;
      }
      _last_accum_time = tnow;
   }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_HMC5843::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;
    return true;
}


bool AP_Compass_HMC5843::_detect_version()
{
    _base_config = 0x0;

    if (!write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        return false;
    }
    if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        /* a 5883L supports the sample averaging config */
        _product_id = AP_COMPASS_TYPE_HMC5883L;
        return true;
    } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
        _product_id = AP_COMPASS_TYPE_HMC5843;
        return true;
    } else {
        /* not behaving like either supported compass type */
        return false;
    }

}

// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_HMC5843::init()
{
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->delay(10);

    _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get HMC5843 semaphore"));
    }

    if (!_detect_version()) {
        goto errout;
    }

    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0f / 1090;  // adjustment for runtime vs calibration gain
    }

    if (!_calibrate(calibration_gain, expected_x, expected_yz, gain_multiple)) {
        goto errout;
    }

    // leave test mode
    if (!re_initialise()) {
        goto errout;
    }

    read();

#if 0
    hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), 
                          _scaling[0], _scaling[1], _scaling[2]);
#endif

    _compass_instance = register_compass();
    set_dev_id(_compass_instance, _product_id);

    _initialised = true;

    _i2c_sem->give();
    hal.scheduler->resume_timer_procs();

    return true;
errout:

    _i2c_sem->give();
    hal.scheduler->resume_timer_procs();
    return false;
}

bool AP_Compass_HMC5843::_calibrate(uint8_t calibration_gain,
        uint16_t expected_x,
        uint16_t expected_yz,
        float gain_multiple)
{
    int numAttempts = 0, good_count = 0;
    bool success = false;

    while (success == 0 && numAttempts < 25 && good_count < 5)
    {
        numAttempts++;

        /* force positiveBias (compass should return 715 for all channels) */
        if (!write_register(ConfigRegA, PositiveBiasConfig)) {
            continue;   /* compass not responding on the bus */
        }

        hal.scheduler->delay(50);

        /* set gains */
        if (!write_register(ConfigRegB, calibration_gain) ||
            !write_register(ModeRegister, SingleConversion)) {
            continue;
        }

        hal.scheduler->delay(50);

        if (!read_raw()) {
            continue;
        }

        hal.scheduler->delay(10);

        float cal[3];

        /* hal.console->printf_P(PSTR("mag %d %d %d\n"), _mag_x, _mag_y, _mag_z); */

        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        /* hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f\n"), cal[0], cal[1], cal[2]); */

        /* we throw away the first two samples as the compass may
         * still be changing its state from the application of the
         * strap excitation. After that we accept values in a
         * reasonable range */

        if (numAttempts <= 2) {
            continue;
        }

#define IS_CALIBRATION_VALUE_VALID(val) (val > 0.7f && val < 1.35f)

        if (IS_CALIBRATION_VALUE_VALID(cal[0]) &&
            IS_CALIBRATION_VALUE_VALID(cal[1]) &&
            IS_CALIBRATION_VALUE_VALID(cal[2])) {

            _scaling[0] += cal[0];
            _scaling[1] += cal[1];
            _scaling[2] += cal[2];

            good_count++;
        /* hal.console->printf_P(PSTR("car=%.2f %.2f %.2f good\n"), cal[0], cal[1], cal[2]); */
        }

#undef IS_CALIBRATION_VALUE_VALID

#if 0
        /* useful for debugging */
        hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
#endif
    }

    if (good_count >= 5) {
        /*
          The use of gain_multiple below is incorrect, as the gain
          difference between 2.5Ga mode and 1Ga mode is already taken
          into account by the expected_x and expected_yz values.  We
          are not going to fix it however as it would mean all
          APM1/APM2 users redoing their compass calibration. The
          impact is that the values we report on APM1/APM2 are lower
          than they should be (by a multiple of about 0.6). This
          doesn't have any impact other than the learned compass
          offsets
         */
        _scaling[0] = _scaling[0] * gain_multiple / good_count;
        _scaling[1] = _scaling[1] * gain_multiple / good_count;
        _scaling[2] = _scaling[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        _scaling[0] = 1.0;
        _scaling[1] = 1.0;
        _scaling[2] = 1.0;
    }

    return success;
}

void AP_Compass_HMC5843::read()
{
    if (!_initialised) {

        /* someone has tried to enable a compass for the first time
         * mid-flight .... we can't do that yet (especially as we won't
         * have the right orientation!) */

        return;
    }

    if (_retry_time != 0) {

        if (hal.scheduler->millis() < _retry_time) {
            return;
        }

        if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
            hal.i2c->setHighSpeed(false);
            return;
        }
    }

    if (_accum_count == 0) {
       accumulate();
       if (_retry_time != 0) {
          hal.i2c->setHighSpeed(false);
          return;
       }
    }

    Vector3f field(_mag_x_accum * _scaling[0],
                   _mag_y_accum * _scaling[1],
                   _mag_z_accum * _scaling[2]);
    field /= _accum_count;

    _accum_count = 0;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    // rotate to the desired orientation
    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        field.rotate(ROTATION_YAW_90);
    }

    publish_field(field, _compass_instance);
    _retry_time = 0;
}
