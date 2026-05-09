using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Ardupilot
{
    public enum PX4_CUSTOM_SUB_MODE_AUTO
    {
        PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
        PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
        PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
        PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
        PX4_CUSTOM_SUB_MODE_AUTO_RTL,
        PX4_CUSTOM_SUB_MODE_AUTO_LAND,
        PX4_CUSTOM_SUB_MODE_AUTO_RTGS
    }
}
