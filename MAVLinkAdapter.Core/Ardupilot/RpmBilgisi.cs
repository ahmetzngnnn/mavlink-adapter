using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Ardupilot
{
    public class RpmBilgisi
    {
        public float Rpm1 { get; set; }
        public float Rpm2 { get; set; }

        public RpmBilgisi(float rpm1, float rpm2)
        {
            Rpm1 = rpm1;
            Rpm2 = rpm2;
        }

        public RpmBilgisi()
        {
            Rpm1 = 0;
            Rpm2 = 0;
        }
        

        public void UpdateFromRpmMsg(MAVLink.mavlink_rpm_t msg)
        {
            Rpm1 = msg.rpm1;
            Rpm2 = msg.rpm2;
        }
    }
}
