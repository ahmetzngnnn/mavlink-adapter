using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class HizBilgisi
    {
        public double HizX { get; set; }
        public double HizY { get; set; }
        public double HizZ { get; set; }
        public double UcEksen { get; set; }

        public void UpdateFromPositionMsg(MAVLink.mavlink_global_position_int_t msg)
        {
            HizX = msg.vx * 0.01;
            HizY = msg.vy * 0.01;
            HizZ = msg.vz * 0.01;
        }
    }
}
