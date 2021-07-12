#include "SIM_I2CDevice.h"

#include <AP_Common/Bitmask.h>

namespace SITL {

class MS5525 : public I2CDevice
{
public:

    MS5525();

protected:

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

private:

    // float pressure;
    // float temperature;

    // time we last updated the measurements (simulated internal
    // workings of sensor)
    uint32_t last_update_ms;

    void reset();

    enum class Command : uint8_t {
        RESET = 0x1E,

        READ_CONVERSION = 0x00,

        // prom reading commands:
        READ_C0 = 0xa0,
        READ_C1 = 0xa2,
        READ_C2 = 0xa4,
        READ_C3 = 0xa6,
        READ_C4 = 0xa8,
        READ_C5 = 0xaa,
        READ_C6 = 0xac,
        READ_CRC = 0xae,

        // conversion start commands:
        CONVERT_D2_OSR_1024 = 0x54,
        CONVERT_D1_OSR_1024 = 0x44,
    };

    enum class State : uint8_t {
        COLD = 5,
        COLD_WAIT = 6,

        UNINITIALISED = 7,

        RUNNING = 17,

        RESET_START = 27,
        RESET_WAIT = 28,

        CONVERSION_D1_START = 37,
        CONVERSION_D1_WAIT = 38,

        CONVERSION_D2_START = 47,
        CONVERSION_D2_WAIT = 48,
    };

    State state = State::COLD;

    uint32_t command_start_us;

    uint8_t read_prom_addr;


    uint8_t convert_out[3];

    // this data comes from the datasheet page 7
    const uint16_t prom[8] {
        0xFFFF,  // reserved
        36402, // C1, pressure sensitivity
        39473, // C2, pressure offset
        40393, // C3, temperature coeff of press sensit
        29523, // C4, temperature cofff of press offs
        29854, // C5, ref temperature
        21917,  // C6, temperature coeff of temperature
        0x000c  // checksum
    };
    bool prom_loaded = false;

    // for 5525DSO-pp001DS
    const uint8_t Qx_coeff[6] {
        15, 17, 7, 5, 7, 21
    };


    uint16_t conversion_time_osr_1024_us = 2280;

    void convert(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2);
    void convert_D1();
    void convert_D2();

    void convert_forward(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C);
};

} // namespace SITL
