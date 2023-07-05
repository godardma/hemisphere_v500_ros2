#ifndef _DEF_HEMISPHERE_GNSS_CONVERSIONS_H_
#define _DEF_HEMISPHERE_GNSS_CONVERSIONS_H_

#include <hemisphere_gnss/BinaryMsg.h>
#include "hemisphere_v500/msg/binary1.hpp"
#include "hemisphere_v500/msg/binary2.hpp"
#include "hemisphere_v500/msg/binary3.hpp"

namespace hemisphere_v500 {

inline void to_ros(hemisphere_v500::msg::Binary1& dst, const SBinaryMsg1& src)
{
    dst.ageofdiff     = src.ageOfDiff;
    dst.numofsats     = src.numOfSats;
    dst.gpsweek       = src.gpsWeek;
    dst.gpstimeofweek = src.gpsTimeOfWeek;
    dst.latitude      = src.latitude;
    dst.longitude     = src.longitude;
    dst.height        = src.height;
    dst.vnorth        = src.vNorth;
    dst.veast         = src.vEast;
    dst.vup           = src.vUp;
    dst.stddevresid   = src.stdDevResid;
    dst.navmode       = src.navMode;
    dst.ageofdifflong = src.ageOfDiffLong;
}

inline void to_ros(hemisphere_v500::msg::Binary2& dst, const SBinaryMsg2& src)
{
    dst.masksatstracked = src.maskSatsTracked;
    dst.masksatsused    = src.maskSatsUsed;
    dst.gpsutcdiff      = src.gpsUtcDiff;
    dst.hdoptimes10     = src.hdopTimes10;
    dst.vdoptimes10     = src.vdopTimes10;
    dst.waasmask        = src.waasMask;
}

inline void to_ros(hemisphere_v500::msg::Binary3& dst, const SBinaryMsg3& src)
{
    dst.gpstimeofweek  = src.gpsTimeOfWeek;
    dst.gpsweek        = src.gpsWeek;
    dst.numsatstracked = src.numSatsTracked;
    dst.numsatsused    = src.numSatsUsed;
    dst.navmode        = src.navMode;
    dst.spare00        = src.spare00;
    dst.latitude       = src.latitude;
    dst.longitude      = src.longitude;
    dst.height         = src.height;
    dst.speed          = src.speed;
    dst.vup            = src.vUp;
    dst.cog            = src.cog;
    dst.heading        = src.heading;
    dst.pitch          = src.pitch;
    dst.spare01        = src.spare01;
    dst.ageofdiff      = src.ageOfDiff;
    //dst.attitudeStatus = src.attitudeStatus;
    dst.yawstatus      = (0b000000111 & src.attitudeStatus);
    dst.pitchstatus    = (0b000111000 & src.attitudeStatus) >> 3;
    dst.rollstatus     = (0b111000000 & src.attitudeStatus) >> 6;
    dst.spare02        = 0;
    dst.stdevheading   = src.stdevHeading;
    dst.stdevpitch     = src.stdevPitch;
    dst.hrms           = src.hRMS;
    dst.vrms           = src.vRMS;
    dst.hdop           = src.hdop;
    dst.vdop           = src.vdop;
    dst.tdop           = src.tdop;
    dst.covnn          = src.covNN;
    dst.covne          = src.covNE;
    dst.covnu          = src.covNU;
    dst.covee          = src.covEE;
    dst.coveu          = src.covEU;
    dst.covuu          = src.covUU;
}

inline hemisphere_v500::msg::Binary1 to_ros(const SBinaryMsg1& src)
{
    hemisphere_v500::msg::Binary1 dst;
    to_ros(dst, src);
    return dst;
}

inline hemisphere_v500::msg::Binary2 to_ros(const SBinaryMsg2& src)
{
    hemisphere_v500::msg::Binary2 dst;
    to_ros(dst, src);
    return dst;
}

inline hemisphere_v500::msg::Binary3 to_ros(const SBinaryMsg3& src)
{
    hemisphere_v500::msg::Binary3 dst;
    to_ros(dst, src);
    return dst;
}

}

#endif //_DEF_HEMISPHERE_GNSS_CONVERSIONS_H_
