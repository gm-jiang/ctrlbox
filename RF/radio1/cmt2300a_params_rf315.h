#ifndef __CMT2300A_PARAMS_RF315_H
#define __CMT2300A_PARAMS_RF315_H
//;---------------------------------------
//;  CMT2300A Configuration File
//;  Generated by CMOSTEK RFPDK 1.50
//;  2020.01.19 15:32
//;---------------------------------------
//az; Mode                      = Advanced
//; Part Number               = CMT2300A
//; Frequency                 = 315.000 MHz
//; Xtal Frequency            = 26.0000 MHz
//; Demodulation              = OOK
//; AGC                       = On
//; Data Rate                 = 2.4 kbps
//; Deviation                 = NA
//; Tx Xtal Tol.              = 20 ppm
//; Rx Xtal Tol.              = 20 ppm
//; TRx Matching Network Type = 13 dBm
//; Tx Power                  = +13 dBm
//; Gaussian BT               = NA
//; Bandwidth                 = Auto-Select kHz
//; Demod Method              = Middle
//; Long-Zero Number          = 31
//; CDR Type                  = Counting
//; CDR DR Range              = NA
//; Rx Duty-Cycle             = Off
//; Tx Duty-Cycle             = Off
//; Sleep Timer               = Off
//; Sleep Time                = NA
//; Rx Timer                  = Off
//; Rx Time T1                = NA
//; Rx Time T2                = NA
//; Rx Exit State             = SLEEP
//; Tx Exit State             = SLEEP
//; SLP Mode                  = Disable
//; RSSI Valid Source         = PJD
//; PJD Window                = 8 Jumps
//; LFOSC Calibration         = On
//; Xtal Stable Time          = 155 us
//; RSSI Compare TH           = NA
//; Data Mode                 = Direct
//; Whitening                 = Disable
//; Whiten Type               = NA
//; Whiten Seed Type          = NA
//; Whiten Seed               = NA
//; Manchester                = Disable
//; Manchester Type           = NA
//; FEC                       = Disable
//; FEC Type                  = NA
//; Tx Prefix Type            = 0
//; Tx Packet Number          = 1
//; Tx Packet Gap             = 32
//; Packet Type               = Fixed Length
//; Node-Length Position      = NA
//; Payload Bit Order         = Start from msb
//; Preamble Rx Size          = 2
//; Preamble Tx Size          = 8
//; Preamble Value            = 170
//; Preamble Unit             = 8-bit
//; Sync Size                 = 2-byte
//; Sync Value                = 11732
//; Sync Tolerance            = None
//; Sync Manchester           = Disable
//; Node ID Size              = NA
//; Node ID Value             = NA
//; Node ID Mode              = None
//; Node ID Err Mask          = Disable
//; Node ID Free              = Disable
//; Payload Length            = 32
//; CRC Options               = None
//; CRC Seed                  = NA
//; CRC Range                 = NA
//; CRC Swap                  = NA
//; CRC Bit Invert            = NA
//; CRC Bit Order             = NA
//; Dout Mute                 = Off
//; Dout Adjust Mode          = Disable
//; Dout Adjust Percentage    = NA
//; Collision Detect          = Off
//; Collision Detect Offset   = NA
//; RSSI Detect Mode          = Always
//; RSSI Filter Setting       = No Filtering
//; RF Performance            = High
//; LBD Threshold             = 2.4 V
//; RSSI Offset               = 26
//; RSSI Offset Sign          = 1
//;---------------------------------------
//;  The following are the Register contents
//;---------------------------------------
#include "cmt2300a_defs.h"

/* [CMT Bank] */
const u8 g_cmt2300aCmtBank_RF315[CMT2300A_CMT_BANK_SIZE] = {
/*0x00*/  0x00,
/*0x01*/  0x66,
/*0x02*/  0xEC,
/*0x03*/  0x1C,
/*0x04*/  0xF0,
/*0x05*/  0x80,
/*0x06*/  0x14,
/*0x07*/  0x08,
/*0x08*/  0x91,
/*0x09*/  0x02,
/*0x0A*/  0x02,
/*0x0B*/  0xD0,
};

/* [System Bank] */
const u8 g_cmt2300aSystemBank_RF315[CMT2300A_SYSTEM_BANK_SIZE] = {
/*0x0C*/  0xAE,
/*0x0D*/  0xE0,
/*0x0E*/  0x30,
/*0x0F*/  0x00,
/*0x10*/  0x00,
/*0x11*/  0xF4,
/*0x12*/  0x10,
/*0x13*/  0xE2,
/*0x14*/  0x42,
/*0x15*/  0x20,
/*0x16*/  0x00,
/*0x17*/  0x81,
};

/* [Frequency Bank] */
const u8 g_cmt2300aFrequencyBank_RF315[CMT2300A_FREQUENCY_BANK_SIZE] = {
	
/*0x18*/  0x48,
/*0x19*/  0xD2,
/*0x1A*/  0x1E,
/*0x1B*/  0x5C,
/*0x1C*/  0x48,
/*0x1D*/  0xB1,
/*0x1E*/  0x13,
/*0x1F*/  0x1B,
};

/* [Data Rate Bank] */
const u8 g_cmt2300aDataRateBank_RF315[CMT2300A_DATA_RATE_BANK_SIZE] = {
/*0x20*/  0x32,   
/*0x21*/  0x18,
/*0x22*/  0x80,
/*0x23*/  0xDD,
/*0x24*/  0x00,
/*0x25*/  0x00,
/*0x26*/  0x00,
/*0x27*/  0x00,
/*0x28*/  0x00,
/*0x29*/  0x00,
/*0x2A*/  0x00,
/*0x2B*/  0x29,
/*0x2C*/  0xC0,
/*0x2D*/  0x51,
/*0x2E*/  0x2A,
/*0x2F*/  0x4B,
/*0x30*/  0x05,
/*0x31*/  0x00,
/*0x32*/  0x50,
/*0x33*/  0x2D,
/*0x34*/  0x00,
/*0x35*/  0x01,
/*0x36*/  0x05,
/*0x37*/  0x05,
};

/* [Baseband Bank] */
const u8 g_cmt2300aBasebandBank_RF315[CMT2300A_BASEBAND_BANK_SIZE] = {
/*0x38*/  0x10,
/*0x39*/  0x08,
/*0x3A*/  0x00,
/*0x3B*/  0xAA,
/*0x3C*/  0x02,
/*0x3D*/  0x00,
/*0x3E*/  0x00,
/*0x3F*/  0x00,
/*0x40*/  0x00,
/*0x41*/  0x00,
/*0x42*/  0x00,
/*0x43*/  0xD4,
/*0x44*/  0x2D,
/*0x45*/  0x00,
/*0x46*/  0x1F,
/*0x47*/  0x00,
/*0x48*/  0x00,
/*0x49*/  0x00,
/*0x4A*/  0x00,
/*0x4B*/  0x00,
/*0x4C*/  0x00,
/*0x4D*/  0x00,
/*0x4E*/  0x00,
/*0x4F*/  0x60,
/*0x50*/  0xFF,
/*0x51*/  0x00,
/*0x52*/  0x00,
/*0x53*/  0x1F,
/*0x54*/  0x10,
};

/* [Tx Bank] */
const u8 g_cmt2300aTxBank_RF315[CMT2300A_TX_BANK_SIZE] = {
/*0x55*/  0x55,
/*0x56*/  0xE7,
/*0x57*/  0x12,
/*0x58*/  0x00,
/*0x59*/  0x0F,
/*0x5A*/  0xB0,
/*0x5B*/  0x00,
/*0x5C*/  0x49,
/*0x5D*/  0x12,
/*0x5E*/  0x3F,
/*0x5F*/  0x7F,
};

#endif