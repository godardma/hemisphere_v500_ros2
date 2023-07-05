#ifndef _DEF_HEMISPHERE_GNSS_BINARY_MSG_H_
#define _DEF_HEMISPHERE_GNSS_BINARY_MSG_H_

#ifdef __cplusplus
extern "C" {
#endif

// For fixed size types
#include <stdint.h>

/*
* Copyright (c) 2020 Hemisphere GNSS, Inc.
* All Rights Reserved. *
* Use and copying of this software and preparation of derivative works based
* upon this software are permitted. Any copy
of this software or of any derivative work must include the above copyright
notice, this paragraph and the one after it. Any distribution of this software
or derivative works must comply with all applicable laws. This software is made
available AS IS, and COPYRIGHT OWNERS DISCLAIMS ALL WARRANTIES, EXPRESS OR
IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE, AND NOTWITHSTANDING ANY OTHER PROVISION
CONTAINED HEREIN, ANY LIABILITY FOR DAMAGES RESULTING FROM THE SOFTWARE OR ITS
USE IS EXPRESSLY DISCLAIMED, WHETHER ARISING IN CONTRACT, TORT (INCLUDING
NEGLIGENCE) OR STRICT LIABILITY, EVEN IF COPYRIGHT OWNERS ARE ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.
*/

//xx #if defined(WIN32) || ( ARMCC_VERSION>=300441) // all compilers that we use today
#pragma pack(push)
#pragma pack(4)
//#endif

/****************************************************/
/* SBinaryMsgHeader */
/****************************************************/
typedef struct
{
    char     strSOH[4];  /* start of header ($BIN) */
    uint16_t blockID;    /* ID of message (1,2,99,98,97,96,95,94,93 or 80 ) */
    uint16_t dataLength; /* 52 16,304,68,28,300,128,96,56, or 40 */
} SBinaryMsgHeader;

typedef struct
{
    uint32_t preamble; /* 0x4E494224 = $BIN */
    uint32_t info;     /* 0x00340001 or 0x00100002 or 0x01300063 */
} SBinaryMsgHeaderDW;  /* or 0x00440062 or 0x001C0061 or 0x012C0060 */
                       /* or 0x0080005F or 0x0060005E or 0x0038005D */
                       /* or 0x00280050 */ 

#define BIN_MSG_PREAMBLE   0x4E494224 /* $BIN = 0x4E494224 */
#define BIN_MSG_HEAD_TYPE1 0x00340001 /* 52 = 0x34 */
#define BIN_MSG_HEAD_TYPE2 0x00100002 /* 16 = 0x10 */
#define BIN_MSG_HEAD_TYPE3 0x00740003 /* 116 = 0x74 */
#define BIN_MSG_HEAD_TYPE4 0x00280004 /* 40 = 0x28 */
#define BIN_MSG_HEAD_TYPE5 0x00480005 /* 72 = 0x48 */
#define BIN_MSG_HEAD_TYPE6 0x000C0006 /* 12 = 0x0C */
#define BIN_MSG_HEAD_TYPE99  0x01300063 /* 99 = 0x63, 304 = 0x130 */ //GPS
#define BIN_MSG_HEAD_TYPE108 0x0174006C /* 108 = 0x6C, 372 = 0x174 = total size in bytes -8 -2 -2 */
#define BIN_MSG_HEAD_TYPE104 0x01180068 /* 104 = 0x68, 280 = 0x118 */
#define BIN_MSG_HEAD_TYPE103 0x00400067 /* 103 = 0x67, 64 = 0x158 */
#define BIN_MSG_HEAD_TYPE102 0x01580066 /* 102 = 0x66, 344 = 0x158 */
#define BIN_MSG_HEAD_TYPE101 0x01C00065 /* 101 = 0x65, 448 = 0x1C0 */
#define BIN_MSG_HEAD_TYPE100 0x01040064 /* 100 = 0x64, 260 = 0x104 */
#define BIN_MSG_HEAD_TYPE98 0x00440062 /* 98 = 0x62, 68 = 0x44 */
#define BIN_MSG_HEAD_TYPE97 0x001C0061 /* 97 = 0x61, 28 = 0x1C */
#define BIN_MSG_HEAD_TYPE96 0x012C0060 /* 96 = 0x60, 300 = 0x12C */ //GPS L1CA phase observables
#define BIN_MSG_HEAD_TYPE95 0x0080005F /* 95 = 0x5F, 128 = 0x80 */ //GPS L1CA ephemeris data
#define BIN_MSG_HEAD_TYPE94 0x0060005E /* 94 = 0x5E, 96 = 0x60 */
#define BIN_MSG_HEAD_TYPE93 0x0038005D /* 93 = 0x5D, 56 = 0x38 */
#define BIN_MSG_HEAD_TYPE92 0x0034005C /* 92 = 0x5C, 52 = 0x34 */
#define BIN_MSG_HEAD_TYPE91 0x0198005B /* 91 = 0x5B, 408 = 0x198 = total size in bytes -8 -2 -2*/
#define BIN_MSG_HEAD_TYPE89 0x00500059 /* 89 = 0x59, 80 = 0x50 */
#define BIN_MSG_HEAD_TYPE80 0x00280050 /* 80 = 0x50, 40 = 0x28 */
#define BIN_MSG_HEAD_TYPE76 0x01C0004C /* 76 = 0x4C, 448 = 0x1C0 = total size in bytes -8 -2 -2*/
#define BIN_MSG_HEAD_TYPE71 0x01C00047 /* 71 = 0x47, 448 = 0x1C0 = total size in bytes -8 -2 -2*/
#define BIN_MSG_HEAD_TYPE61 0x0140003D /* 61 = 0x3D, 320 = 0x140 */
#define BIN_MSG_HEAD_TYPE62 0x0028003E /* 62 = 0x3E, 40 = 0x28 */
#define BIN_MSG_HEAD_TYPE65 0x00440041 /* 65 = 0x41, 68 = 0x44 */
#define BIN_MSG_HEAD_TYPE66 0x01600042 /* 66 = 0x42, 352 = 0x160 */
#define BIN_MSG_HEAD_TYPE69 0x012C0045 /* 69 = 0x45, 300 = 0x12C */ //Glonass
#define BIN_MSG_HEAD_TYPE59 0x0100003B /* 59 = 0x3B, 256 = 0x100 */ //GPS L2C
#define BIN_MSG_HEAD_TYPE49 0x012C0031 /* 49 = 0x31, 300 = 0x12C */ //Galileo Channel Data for SLXMON
#define BIN_MSG_HEAD_TYPE45 0x0080002D /* 45 = 0x2D, 128 = 0x80 */ //Galileo subframe words --- similar to GPS
#define BIN_MSG_HEAD_TYPE44 0x0038002C /* 44 = 0x2C, 56 = 0x38 */ //Galileo time offsets
#define BIN_MSG_HEAD_TYPE42 0x0034002A /* 42 = 0x2A, 52 = 0x34 */
#define BIN_MSG_HEAD_TYPE30 0x0080001E /* 30 = 0x1E, 208 = 0xD0 */ //BeiDou subframe words --- similar to GPS
#define BIN_MSG_HEAD_TYPE32 0x00340020 /* 32 = 0x20, 52 = 0x34 */
#define BIN_MSG_HEAD_TYPE34 0x00200022 /* 34 = 0x22, 32 = 0x20 */ //BeiDou time offsets
#define BIN_MSG_HEAD_TYPE35 0x00800023 /* 35 = 0x23, 128 = 0x80 */ //BeiDou subframe words --- similar to GPS
#define BIN_MSG_HEAD_TYPE36 0x01500024 /* 36 = 0x24, 336 = 0x150 */ //BeiDou phase observables
#define BIN_MSG_HEAD_TYPE39 0x019C0027 /* 39 = 0x27, 412 = 0x19C */ //BeiDou Channel Data for SLXMON
#define BIN_MSG_HEAD_TYPE22 0x00340016 /* 22 = 0x16, 52 = 0x34 */
#define BIN_MSG_HEAD_TYPE25 0x00800019 /* 25 = 0x19, 128 = 0x80 */ //QZSS L1CA ephemeris data
#define BIN_MSG_HEAD_TYPE16 0x01380010 /* 16 = 0x10, 312 = 0x138 */ //GNSS phase observables
#define BIN_MSG_HEAD_TYPE19 0x01780013 /* 19 = 0x13, 376 = 0x178 */ //Generic Channel Data for SLXMON
#define BIN_MSG_HEAD_TYPE10 0x0194000A /* 10 = 0xA, 404 = 0x194 = total size in bytes -8 -2 -2*/
//#define BIN_MSG_HEAD_TYPE12 0x0194000C /* 12 = 0xC, 404 = 0x194 = total size in bytes -8 -2 -2*/
#define BIN_MSG_HEAD_TYPE12 0x019A000C /* 12 = 0xC, 410 = 0x19A = total size in bytes -8 -2 -2*/ //RFR_160506 -- added 6 bytes
#define BIN_MSG_HEAD_TYPE13 0x0194000D /* 13 = 0xD, 404 = 0x194 = total size in bytes -8 -2 -2*/
#define BIN_MSG_HEAD_TYPE17 0x02100011 /* 17 = 0x11, 528 = 0x210 = total size in bytes -8 -2 -2*/
//#if defined(_RXAIF_PLOT_MESSAGES_)
#define BIN_MSG_HEAD_TYPE11 0x0064000B /* 11 = 0x0B, 100 = 0x64 = total size(112) in bytes -8 -2 -2*/
//#endif
#define BIN_MSG_HEAD_TYPE209 0x014C00D1 // 209 = 0xD1, 332 = 0x14C
#define BIN_MSG_HEAD_TYPE122 0x0050007A //122 = 0x7A, 80 = 50

// #endif // what is this doing here ?


#define BIN_MSG_CRLF 0x0A0D /* CR LF = 0x0D, 0x0A */
#define CHANNELS_12 12
#define CHANNELS_20 20
#define CHANNELS_gen 16 // CHANNELS FOR 16 and 19 general messages
#define cBPM_SCAT_MEMSIZE 100
#define cBPM_SPEC_AN_MEMSIZE 128 //Must be a power of 2 (for a 4096 FFT we need 32 of these)
//RFR_140829 #define cBPM_STRIP_MEMSIZE 50
#define cBPM_STRIP_MEMSIZE 95
//#if defined(_RXAIF_PLOT_MESSAGES_)
#define cBPM_AIFSCAT_MEMSIZE 16
//#endif


typedef union
{
    SBinaryMsgHeader   bytes;
    SBinaryMsgHeaderDW dWord;
} SUnionMsgHeader;

/****************************************************/
/* SBinaryMsg1 */
/****************************************************/
typedef struct
{
    SUnionMsgHeader head;
    uint8_t  ageOfDiff;     // age of differential, seconds (255 max)
    uint8_t  numOfSats;     // number of satellites used (12 max)
    uint16_t gpsWeek;       // GPS week
    double   gpsTimeOfWeek; // GPS tow
    double   latitude;      // Latitude degrees, -90..90
    double   longitude;     // Longitude degrees, -180..180
    float    height;        // (m), Altitude ellipsoid
    float    vNorth;        // Velocity north m/s
    float    vEast;         // Velocity east m/s
    float    vUp;           // Velocity up m/s
    float    stdDevResid;   // (m), Standard Deviation of Residuals
    uint16_t navMode;
    uint16_t ageOfDiffLong; // age of diff using 16 bits (WHY?)
    uint16_t checkSum;      // sum of all bytes of the header and data
    uint16_t crlf;          // Carriage Return Line Feed
} SBinaryMsg1;              // length = 8 + 52 + 2 + 2 = 64

/****************************************************/
/* SBinaryMsg2 */
/****************************************************/
typedef struct
{
    SUnionMsgHeader head;
    uint32_t maskSatsTracked; // SATS Tracked, bit mapped 0..31
    uint32_t maskSatsUsed;    // SATS Used, bit mapped 0..31
    uint16_t gpsUtcDiff;      // GPS/UTC time difference (GPS minus UTC)
    uint16_t hdopTimes10;     // HDOP (0.1 units)
    uint16_t vdopTimes10;     // VDOP (0.1 units)
    uint16_t waasMask;        // Bits 0-1: tracked sats, Bits 2-3:
                              // used sats, Bits 5-9 WAAS PRN 1 minus
                              // 120, Bits 10-14 WAAS PRN 1 minus 120
    uint16_t checkSum;        // sum of all bytes of the header and data
    uint16_t crlf;            // Carriage Return Line Feed
} SBinaryMsg2;                // length = 8 + 16 + 2 + 2 = 28

//-****************************************************
//-* SBinaryMsg3
//-* Lat/Lon/Hgt, Covariances, RMS, DOPs and COG, Speed, Heading
//-****************************************************
typedef struct
{
    SUnionMsgHeader head;    // [8]
    double   gpsTimeOfWeek;  // GPS tow [8 bytes]
    uint16_t gpsWeek;        // GPS week [2 bytes]
    uint16_t numSatsTracked; // SATS Tracked [2 bytes]
    uint16_t numSatsUsed;    // SATS Used [2 bytes]
    uint8_t  navMode;        // Nav Mode (same as message 1) [1 byte ]
    uint8_t  spare00;        // Spare [1 byte ]
    double   latitude;       // Latitude degrees, -90..90 [8 bytes]
    double   longitude;      // Longitude degrees, -180..180 [8 bytes]
    float    height;         // (m), Altitude ellipsoid [4 bytes]
    float    speed;          // Horizontal Speed m/s [4 bytes]
    float    vUp;            // Vertical Velocity +up m/s [4 bytes]
    float    cog;            // Course over Ground, degrees [4 bytes]
    float    heading;        // Heading (degrees), Zero unless vector[4 bytes]
    float    pitch;          // Pitch (degrees), Zero unless vector [4 bytes]
    float    spare01;        // Spare [4 bytes]
    uint16_t ageOfDiff;      // age of differential, seconds [2 bytes]
    uint16_t attitudeStatus; // Attitude Status, Zero unless vector [2 bytes]
                             // attitudeStatus: bit {0-3} = sStatus.eYaw
                             // bit {4-7} = sStatus.ePitch
                             // bit {8-11} = sStatus.eRoll
                             // where sStatus can be 0 = INVALID, 1 = GNSS, 2 = Inertial, 3= Magnetic
    float    stdevHeading;   // Yaw stdev, degrees, 0 unless vector [4 bytes]
    float    stdevPitch;     // Pitch stdev, degrees, 0 unless vector[4 bytes]
    float    hRMS;           // Horizontal RMS [4 bytes]
    float    vRMS;           // Vertical RMS [4 bytes]
    float    hdop;           // Horizontal DOP [4 bytes]
    float    vdop;           // Vertical DOP [4 bytes]
    float    tdop;           // Time DOP [4 bytes]
    float    covNN;          // Covaraince North-North [4 bytes]
    float    covNE;          // Covaraince North-East [4 bytes]
    float    covNU;          // Covaraince North-Up [4 bytes]
    float    covEE;          // Covaraince East-East [4 bytes]
    float    covEU;          // Covaraince East-Up [4 bytes]
    float    covUU;          // Covaraince Up-Up [4 bytes]
    uint16_t checkSum;       // sum of all bytes of the header and data
    uint16_t crlf;           // Carriage Return Line Feed
} SBinaryMsg3;               // length = 8 + 116 + 2 + 2 = 128 (108 = 74 hex)

//xx #if defined(WIN32) || ( ARMCC_VERSION>=300441) // all compilers that we use today
#pragma pack(pop)
//xx #endif

#ifdef __cplusplus
} // extern "C"

#include <iostream>

inline size_t msg_size(const SBinaryMsgHeader& header)
{
    return header.dataLength + sizeof(SUnionMsgHeader) + 4;
}

inline size_t msg_size(const SUnionMsgHeader& header)
{
    return msg_size(header.bytes);
}

inline std::ostream& operator<<(std::ostream& os, const SBinaryMsgHeader& header)
{
    os << header.strSOH[0]
       << header.strSOH[1]
       << header.strSOH[2]
       << header.strSOH[3]
       << header.blockID
       << " (size : " << header.dataLength << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const SUnionMsgHeader& header)
{
    os << header.bytes;
    return os;
}

#endif

#endif //_DEF_HEMISPHERE_GNSS_BINARY_MSG_H_
