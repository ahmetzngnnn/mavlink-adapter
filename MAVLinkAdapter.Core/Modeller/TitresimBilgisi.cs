using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class TitresimBilgisi
    {
        public TitresimBilgisi()
        {
            this.vibex = 0;
            this.vibey = 0;
            this.vibez = 0;
            this.vibeclip0 = 0;
            this.vibeclip1 = 0;
            this.vibeclip2 = 0;
        }

        public uint vibeclip0 { get; set; }

        public uint vibeclip1 { get; set; }

        public uint vibeclip2 { get; set; }

        public float vibex { get; set; }

        public float vibey { get; set; }

        public float vibez { get; set; }
        
        public void UpdateFromVibrationMsg(MAVLink.mavlink_vibration_t msg)
        {
            vibex = msg.vibration_x;
            vibey = msg.vibration_y;
            vibez = msg.vibration_z;
            vibeclip0 = msg.clipping_0;
            vibeclip1 = msg.clipping_1;
            vibeclip2 = msg.clipping_2;
        }
    }
}
