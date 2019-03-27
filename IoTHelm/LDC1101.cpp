/**
* @file LDC1101.cpp
* @brief this C++ file contains all required
* functions to interface with Texas
* Instruments' LDC1101.
*
* @author Victor Sluiter & Bob Giesberts
*
* @date 2015-12-09
*/

#include "LDC1101.h"


LDC1101::LDC1101(PinName mosi, PinName miso, PinName sck, PinName cs, float capacitor, float f_CLKIN) : _spiport(mosi,miso,sck, NC), _cs_pin(cs)
{
    // settings
    _cap = capacitor;
    _LHRoffset = 0;
    _f_sensor_min = 6.4;    // MHz
    _Rcount = 0xffff;       // max

    _spiport.format(8,3);
    _spiport.frequency(1E6);
    setFrequency(f_CLKIN);

    _cs_pin.write(1);
    wait_us(100);

    init();
}

LDC1101::~LDC1101()
{
    // THERE IS A LEAK HERE!!!
    // _sd_enable is connected to both the SD card and the LDC1101
    // this line should put 0.0 V on the SD card and the LDC1101
    // but in reality there still is 0.8 V on both the SD card and the LDC1101


    // For all SPI communication pinouts:
    // 1) Create a new handle to access the pinout
    // 2) Set it to 0 V
    // 3) delete and remove the class

  // SPI communication with the SD card --> THIS SHOULD BE PART OF THE SDFileSystem CLASS!!!
   /* DigitalOut *sdP2 = new DigitalOut(p4);
        sdP2->write(0);
        delete sdP2;
        sdP2 = NULL;// SPI: cs
    DigitalOut *sdP3 = new DigitalOut(p6);
        sdP3->write(0);
        delete sdP3;
        sdP3 = NULL;// SPI: mosi
    DigitalOut *sdP5 = new DigitalOut(p5);
        sdP5->write(0);
        delete sdP5;
        sdP5 = NULL;// SPI: sck
    DigitalOut *sdP7 = new DigitalOut(p7);
        sdP7->write(0);
        delete sdP7;
        sdP7 = NULL;// SPI: miso*/
/*
    // SPI communication with the LDC1101
    DigitalOut *senP2 = new DigitalOut(p6);
        senP2->write(0);
        delete senP2;
        senP2 = NULL; // SPI: miso
    DigitalOut *senP3 = new DigitalOut(p4);
        senP3->write(0);
        delete senP3;
        senP3 = NULL; // SPI: sck
    DigitalOut *senP4 = new DigitalOut(p5);
        senP4->write(0);
        delete senP4;
        senP4 = NULL; // SPI: mosi
    DigitalOut *senP5 = new DigitalOut(p7);
        senP5->write(0);
        delete senP5;
        senP5 = NULL; // SPI: cs*/
}


void LDC1101::func_mode(LDC_MODE mode)
{
    writeSPI((uint8_t *)(&mode), 0x0B);
    wait_ms(0.8);
}

void LDC1101::sleep(void)
{
    /* stop toggling the CLKIN pin input and drive the CLKIN pin Low */
    func_mode( LDC_MODE_SHUTDOWN );
    suicide( this );
}

void LDC1101::wakeup(void) {
    /* start toggling the clock input on the CLKIN pin */
    init();
    wait(0.5);
}


void LDC1101::init()
{
    /********* SETTINGS *****************
    ** C_sensor     =   120 pF
    ** L_sensor     =     5 uH
    ** Rp_min       =  1500 Ohm
    **
    ** RCount       = 65535     (max)
    ** Samplerate   =    15.3 Hz
    ** t_conv       =    65.5 ms
    **
    ** f_sensor_min =     6.4 MHz (d = inf)
    ** f_sensor_max =    10   MHz (d = 0)
    ** divider      =     1
    ************************************/


    // Set LDC1101 in configuration modus
    func_mode( LDC_MODE_STANDBY );     // STANDBY = 0x01 naar 0x0B

    // - initialise LHR mode & enable SHUTDOWN mode
    // setLHRmode();   // LHR mode
    setRPmode(); // RP+L mode

    // - set ResponseTime to 6144
    setResponseTime( LDC_RESPONSE_6144 );

    // - set Reference Count to highest resolution
    setReferenceCount( _Rcount );

    // - set calibrated value for f_sensor_min (d = inf, no target)
    set_fsensor_min( _f_sensor_min ); // 6.4 MHz

    // - disable RP_MAX
    // - set RP_MIN to 1,5 kOhm (RPMIN_1)
    // setRPsettings( 1, RPMAX_96, RPMIN_1 );   // LHR mode
    setRPsettings( 0, RPMAX_96, RPMIN_1 );   // RP+L mode

    // - set Divider to 1 (for large range / ENOB / resolution)
    setDivider( DIVIDER_1 );

    // - shift the signal down a bit
    setLHRoffset( _LHRoffset );

    // Done configuring settings, set LDC1101 in measuring modus
    func_mode( LDC_MODE_ACTIVE );
}

void LDC1101::setLHRmode( void ){
    writeSPIregister( 0x05, 0x03 ); // ALT_CONFIG:  0000 0011 --> LHR modus + Shutdown enabled
    writeSPIregister( 0x0C, 0x01 ); // D_CONFIG:    Enables LHR modus, disables RP
}

void LDC1101::setRPmode( void ){
    writeSPIregister( 0x05, 0x02 ); // ALT_CONFIG:  0000 0010 --> RP modus + Shutdown enabled
    writeSPIregister( 0x0C, 0x00 ); // D_CONFIG:    Enables LHR modus, disables RP
}

void LDC1101::setRPsettings(bool RP_MAX_DIS, RPMAX rpmax, RPMIN rpmin)
{
    float rps[] = {96, 48, 24, 12, 6, 3, 1.5, 0.75};
    _RPmin = rps[rpmin];
    _RPmax = rps[rpmax];
    writeSPIregister(0x01, (RP_MAX_DIS << 7) | (rpmax << 4) | rpmin);
}

void LDC1101::setDivider(DIVIDER div)
{
    uint8_t divs[] = {1, 2, 4, 8};
    _divider = divs[div];
    writeSPIregister(0x34, div);
}

void LDC1101::setLHRoffset( uint32_t offset )
{
    _LHRoffset = offset;
    uint16_t LHR_OFFSET = offset / 256;
    writeSPIregister(0x32, (uint8_t) (LHR_OFFSET & 0x00ff) );           // LSB
    writeSPIregister(0x33, (uint8_t) ((LHR_OFFSET & 0xff00) >> 8) );    // MSB
}

void LDC1101::setResponseTime(LDC_RESPONSE responsetime)
{
    uint16_t resps[] = {0, 0, 192, 384, 768, 1536, 3072, 6144};
    _responsetime = resps[responsetime];
    uint8_t DIG_CONF[1];
    readSPI(DIG_CONF, 0x04, 1);
    writeSPIregister(0x04, (DIG_CONF[0] & 0xF8) + responsetime);
}

void LDC1101::setReferenceCount(uint16_t rcount)
{
    _Rcount = rcount;
    uint8_t LHR_RCOUNT_LSB = (rcount & 0x00ff);
    uint8_t LHR_RCOUNT_MSB = ((rcount & 0xff00) >> 8);
    writeSPIregister(0x30, LHR_RCOUNT_LSB);   //LSB
    writeSPIregister(0x31, LHR_RCOUNT_MSB);   //MSB
}

void LDC1101::setSampleRate(float samplerate){ setReferenceCount( ((_fCLKIN/samplerate)-55)/16 ); }


void LDC1101::set_fsensor_min(float f_sensor_min)
{
    uint8_t DIG_CONF[1];
    readSPI(DIG_CONF, 0x04, 1);
    uint8_t MIN_FREQ = 16.0f - (8.0f / f_sensor_min);
    writeSPIregister(0x04, ((MIN_FREQ << 4) + (DIG_CONF[0] & 0x0f)));
}

float LDC1101::get_fsensor_min()
{
    uint8_t DIG_CONF[1];
    readSPI(DIG_CONF, 0x04, 1);
    return (float) 8.0f/(16.0f - (float) ((DIG_CONF[0] & 0xf0) >> 4));
}

bool LDC1101::is_New_LHR_data(void) { return(!(get_LHR_status() & 0x01)); }
bool LDC1101::is_Oscillation_Error(void) { return(get_status() & 0x80); }

uint8_t LDC1101::get_status(void)
{
    uint8_t status[1];
    readSPI(status, 0x20, 1);
    return status[0];
}

uint8_t LDC1101::get_LHR_status(void)
{
    uint8_t LHR_status[1];
    readSPI( LHR_status, 0x3B, 1 );

    // ERR_ZC:   (LHR_status & 0x10) >> 4   //
    // ERR_OR:   (LHR_status & 0x08) >> 3   //
    // ERR_UR:   (LHR_status & 0x04) >> 2   // 1 = LHR_DATA < 0 because LHR_OFFSET > LHR_DATA
    // ERR_OF:   (LHR_status & 0x02) >> 1   //
    // LHR_DRDY: (LHR_status & 0x01)        // 1 = Data ready

    return LHR_status[0];
}


/* CALCULATE STUFF WITH SENSOR DATA */

float LDC1101::get_Q(void){ return _RPmin * sqrt(_cap/_inductance*1000000); }


float LDC1101::get_fsensor( uint32_t Ldata )
{
    // LHR mode
    //if( Ldata == 0 ) { Ldata = get_LHR_Data(); }
    //_fsensor = _fCLKIN * _divider * (Ldata + _LHRoffset)/16777216;       // (p.26)

    // RP+L mode
    if( Ldata == 0 ) { Ldata = get_L_Data(); }
    _fsensor = (_fCLKIN * 6144) / (3 * Ldata);       // (p.31)

    return _fsensor;
}

float LDC1101::get_Inductance( uint32_t Ldata )
{
    float fsensor = get_fsensor( Ldata );
    _inductance = 1./(_cap * 4*PI*PI*fsensor*fsensor); // (p.34)
    return _inductance;
}


/* GETTING DATA FROM SENSOR */

float LDC1101::get_RP( uint16_t RPdata )
{
    if( RPdata == 0 )
    {
        RPdata = get_RP_Data();
    }

    return (_RPmax * _RPmin) / ( _RPmax * (1.0f - ((float) RPdata / 65535.0f)) + _RPmin * ((float) RPdata / 65535.0f));
    // return _RPmax * (1.0f - ((float) RPdata / 65535.0f));
}

uint32_t LDC1101::get_LHR_Data(void)
{
    uint8_t LHR_DATA[3];
    readSPI(LHR_DATA, 0x38, 3);     // 0x38 + 0x39 + 0x3A
    return (LHR_DATA[2]<<16) | (LHR_DATA[1]<<8) | LHR_DATA[0];
}

uint16_t LDC1101::get_RP_Data(void)
{
    uint8_t RP_DATA[2];
    readSPI(RP_DATA, 0x21, 2);     // 021 + 0x22
    return (RP_DATA[1]<<8) | RP_DATA[0];
}

uint16_t LDC1101::get_L_Data(void)
{
    uint8_t L_DATA[2];
    readSPI(L_DATA, 0x23, 2);     // 023 + 0x24
    return (L_DATA[1]<<8) | L_DATA[0];
}


void LDC1101::readSPI(uint8_t *data, uint8_t address, uint8_t num_bytes)
{
    // CSB down
    _cs_pin.write(0);
    _spiport.write(address | 0x80); //read flag
    for(int i=0; i < num_bytes ; i++)
    {
        data[i] = _spiport.write(0xFF);
    }
    // CSB up
    _cs_pin.write(1);
}

void LDC1101::writeSPI(uint8_t *data, uint8_t address, uint8_t num_bytes)
{
    // CSB down
    _cs_pin.write(0);

    _spiport.write(address);
    for(int i=0; i < num_bytes ; i++)
    {
        _spiport.write(data[i]);
    }
    // CSB up
    _cs_pin.write(1);
}


// EXTRA test: Get&print values of all variables to verify (to calculate the induction)
// The data will be printed on the screen using RealTerm: baud 9600.
// Begin ***********************************************************
    float LDC1101::get_fCLKIN()             {return _fCLKIN;}
    uint8_t LDC1101::get_divider()          {return _divider;}
    uint32_t LDC1101::get_LHRoffset()       {return _LHRoffset;}
    float LDC1101::get_RPmin()              {return _RPmin;}
    float LDC1101::get_RPmax()              {return _RPmax;}
    float LDC1101::get_cap()                {return _cap;}
// END ***********************************************************
