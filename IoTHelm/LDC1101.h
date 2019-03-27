#ifndef _LDC1101_H_
#define _LDC1101_H_

/**
* @file LDC1101.h
* @brief this header file will contain all required
* definitions for the functions to interface with Texas
* Instruments' LDC1101.
*
* @author Victor Sluiter & Bob Giesberts
*
* @date 2015-12-09
*/

#include "mbed.h"

#ifndef PI
#define PI 3.14
#endif

typedef enum {  LDC_RESPONSE_192 = 2, \
                LDC_RESPONSE_384 = 3, \
                LDC_RESPONSE_768 = 4, \
                LDC_RESPONSE_1536= 5, \
                LDC_RESPONSE_3072= 6, \
                LDC_RESPONSE_6144= 7} LDC_RESPONSE;

typedef enum {  LDC_MODE_ACTIVE   = 0, \
                LDC_MODE_STANDBY  = 1, \
                LDC_MODE_SHUTDOWN = 2} LDC_MODE;

typedef enum {  DIVIDER_1 = 0, \
                DIVIDER_2 = 1, \
                DIVIDER_4 = 2, \
                DIVIDER_8 = 3} DIVIDER;

typedef enum {  RPMAX_96 = 0, \
                RPMAX_48 = 1, \
                RPMAX_24 = 2, \
                RPMAX_12 = 3, \
                RPMAX_6 = 4, \
                RPMAX_3 = 5, \
                RPMAX_1 = 6, \
                RPMAX_0 = 7} RPMAX;

typedef enum {  RPMIN_96 = 0, \
                RPMIN_48 = 1, \
                RPMIN_24 = 2, \
                RPMIN_12 = 3, \
                RPMIN_6 = 4, \
                RPMIN_3 = 5, \
                RPMIN_1 = 6, \
                RPMIN_0 = 7} RPMIN;

/**
* Class for the LDC1101.
* @author Victor Sluiter
* @date 2015-12-09
*/
class LDC1101
{
    public:
        /**
        * @brief Create a new Class to interface to an LDC1101
        **/
        LDC1101(PinName mosi, PinName miso, PinName sck, PinName cs, float capacitor, float f_CLKIN);
        ~LDC1101();

        /**
        * @brief Set power mode.
        * The constructor sets the LDC1101 in Active mode.
        * @param mode choose from LDC_MODE_ACTIVE, LDC_MODE STANDBY or LDC_MODE_SHUTDOWN
        **/
        void func_mode(LDC_MODE mode);
        /**
        * @brief Set LDC1101 to lowest power setting
        **/
        void sleep(void);
        /**
        * @brief Get LDC1101 to work for you again
        **/
        void wakeup(void);
        /**
        * @brief initial configurations
        **/
        void init(void);
        /**
        * @brief initialises LHR mode
        * Also enables shutdown modus
        **/
        void setLHRmode(void);
        void setRPmode(void);

        /**
        * @brief Settings for RP
        * @param RPMAX_DIS [7]
        * 0 - not disabled: RP_MAX is driven
        * 1 - disabled: RP_MAX is ignored, current drive is off.
        * @param RPMIN [2:0]
        * This setting can be calibrated with the target closest to the sensor: R_p(d = 0mm)
        * RPMIN < 0.8 x R_p(d = 0mm)
        * If R_p < 750 Ohm --> increase distance to target
        **/
        void setRPsettings(bool RPMAX_DIS, RPMAX rpmax, RPMIN rpmin);

        /**
        * @brief Sensor divider (p.26)
        * Sensor input divider         (p.35)
        * Because f_CLKIN > 4*f_sensor_max is not realisable for higher frequencies, so there is a divider
        * f_CLKIN > 4 * f_sensor_max / SENSOR_DIV
        * this effectively decreases resolution. Preferable use setLHRoffset instead.
        * @param div
        * - DIVIDER_1
        * - DIVIDER_2
        * - DIVIDER_4
        * - DIVIDER_8
        **/
        void setDivider(DIVIDER div);

        /**
        * @brief Sensor offset (p.26)
        * Sensor offset
        * The sensor might reach a value > 2^24. To prevent this, set an offset.
        * @param offset
        * 32 bit value that should be substracted from the current sensor value
        **/
        void setLHRoffset( uint32_t offset );

        /**
        * @brief Set the Response Time parameters. Does not apply in LHR mode (p.17)
        * @param responsetime
        * Larger value increases accuracy, but slows down the output data rate. Choose one of these values:
        * - LDC_RESPONSE_192
        * - LDC_RESPONSE_384
        * - LDC_RESPONSE_768
        * - LDC_RESPONSE_1536
        * - LDC_RESPONSE_3072
        * - LDC_RESPONSE_6144
        *              ResponseTime
        * t_conv (s) = ------------
        *              3 x f_sensor
        **/
        void setResponseTime(LDC_RESPONSE responsetime);

        /**
        * @brief Set the Reference Count parameter.
        * @param LHR_RCount
        * For LHR mode, the conversion time is set by the reference count LHR_RCOUNT (0x30 & 0x31) (p.34)
        * The conversion time represents the number of clock cycles used to measure the sensor frequency.
        * Higher values for LHR_RCOUNT have a higher effective measurement resolution but a lower sample rate. (p.34)
        * The maximum setting (0xffff) is required for full resolution (p. 35)
        * LHR_RCount = (f_CLKIN/sample rate - 55)/16
        **/
        void setReferenceCount(uint16_t LHR_RCount);

        /**
        * @brief Set the rample rate (indirectly set the reference count)
        **/
        void setSampleRate( float samplerate );

        /**
        * @brief Set the minimum sensor frequency (so without any target)
        * @param f_sensor_min
        * f_sensor_min in MHz
        * value between 0.5 and 8 MHz
        **/
        void set_fsensor_min(float f_sensor_min);

        /**
        * @brief Set the value of the external capacitor
        * This is needed for the calculation of the inductance.
        **/
        void setCapacitor(float c){_cap = c;};
        /**
        * @brief set the value of the external clock
        * If PWMout is used to generate a clock signal, this will update the output frequency.s
        **/
        void setFrequency(float frequency){_fCLKIN = frequency;};





        /**
        * @brief Read LHR_Data, the raw 24-bit inductance value.
        * This is needed for the calculation of the inductance.
        * It reads from addresses 0x38, 0x39 & 0x3A.
        **/
        uint32_t get_LHR_Data(void);
        uint16_t get_RP_Data(void);
        uint16_t get_L_Data(void);
        /**
        * @brief get the set minimum value for f_sensor (0x04[7:5])
        **/
        float get_fsensor_min(void);
        /**
        * @brief get the calculated value for f_sensor
        **/
        float get_fsensor(uint32_t Ldata = 0);
        /**
        * @brief get the calculated inductance value
        **/
        float get_Inductance(uint32_t Ldata = 0);
        float get_RP( uint16_t RPdata = 0);
        /**
        * @brief get the reference frequency (f_CLKIN)
        **/
        float get_fCLKIN(void);
        /**
        * @brief get the responsetime
        **/
        uint16_t get_responsetime(void) { uint16_t resps[] = {0, 0, 192, 384, 768, 1536, 3072, 6144}; uint8_t resp[1]; readSPI(resp, 0x04, 1); return resps[(resp[0] & 0x07)]; };
        /**
        * @brief get RPmin
        **/
        float get_RPmin(void);
        /**
        * @brief get RPmax
        **/
        float get_RPmax(void);
        /**
        * @brief get the reference count
        **/
        uint16_t get_Rcount(void) { uint8_t rcount[2]; readSPI(rcount, 0x30, 2); return ((rcount[1] << 8) | rcount[0]); };
        /**
        * @brief get the divider
        **/
        uint8_t get_divider(void);
        /**
        * @brief get LHR_OFFSET
        **/
        uint32_t get_LHRoffset(void);
        /**
        * @brief get the capacitance
        **/
        float get_cap(void);
        /**
        * @brief get the quality
        **/
        float get_Q(void);

        uint8_t get_status(void);
        uint8_t get_LHR_status(void);
        bool is_New_LHR_data(void);
        bool is_Oscillation_Error(void);




    private:
        void readSPI(uint8_t *data, uint8_t address, uint8_t num_bytes = 1);
        void writeSPI(uint8_t *data, uint8_t address, uint8_t num_bytes = 1);
        void writeSPIregister(uint8_t reg, uint8_t value){writeSPI(&value,reg);}; // VERKEERD OM?!
        void suicide(void *obj) {delete obj;};


        uint16_t _responsetime; // Response Time
        uint8_t _divider;       // divider
        uint32_t _LHRoffset;    // LHR_OFFSET
        float _RPmin;           // RP_MIN
        float _RPmax;           // RP_MAX
        float _fsensor;         // f_sensor: the calculated frequency of the sensor
        float _f_sensor_min;    // f_sensor_min: setting for the lowest expected value for f_sensor
        float _inductance;      // the calculated inductance
        float _fCLKIN;          // f_CLKIN: frequency of external clock: 16MHz
        float _cap;             // capacitor: 120 pF
        uint32_t _L_data;       // The raw measured data for inductance
        uint16_t _Rcount;       // The reference count

        SPI _spiport;
        DigitalOut _cs_pin;

};

#endif
