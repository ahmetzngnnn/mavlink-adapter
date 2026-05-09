using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    /// <summary>
    /// Optik akış sensörü bilgisi
    /// </summary>
    public class OpticFlowSensorBilgisi
    {
        public OpticFlowSensorBilgisi()
        {
            Mx = 0;
            My = 0;
            X = 0;
            Y = 0;
            Kalite = 0;
        }

        // X-sensör yönünde akış, açısal hız dengelemesi (m/s)
        public float Mx { get; set; }

        // Y-sensör yönünde akış, açısal hız dengelemesi (m/s)
        public float My { get; set; }

        // X yönündeki akış (dpix)
        public short X { get; set; }

        // Y yönündeki akış (dpix)
        public short Y { get; set; }

        // Sensor kalitesi
        public byte Kalite { get; set; }
        

        public void UpdateFromOptFlowMsg(MAVLink.mavlink_optical_flow_t optflow)
        {
            Mx = optflow.flow_comp_m_x;
            My = optflow.flow_comp_m_y;
            X = optflow.flow_x;
            Y = optflow.flow_y;
            Kalite = optflow.quality;
        }
    }
}
