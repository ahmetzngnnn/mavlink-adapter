using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class ImuBilgisi

    {

        public float IvmeX { get; set; }
        public float IvmeY { get; set; }
        public float IvmeZ { get; set; }

        public float IvmeDegeri
        {
            get
            {
                if (IvmeX != null && IvmeY != null && IvmeZ != null)
                {
                    return (float)Math.Sqrt(IvmeX * IvmeX + IvmeY * IvmeY + IvmeZ * IvmeZ);
                }
                else
                {
                    return 0;
                }
            }
        }

        public float GyroX { get; set; }
        public float GyroY { get; set; }
        public float GyroZ { get; set; }

        public float GyroDegeri
        {
            get
            {
                if (GyroX != null && GyroY != null && GyroZ != null)
                {
                    return (float)Math.Sqrt(GyroX * GyroX + GyroY * GyroY + GyroZ * GyroZ);
                }
                else
                {
                    return 0;
                }
            }
        }

        public float MagX { get; set; }
        public float MagY { get; set; }
        public float MagZ { get; set; }

        public float ManyetikAlan
        {
            get
            {
                if (MagX != null && MagY != null && MagZ != null)
                {
                    return (float)Math.Sqrt(MagX * MagX + MagY * MagY + MagZ * MagZ);
                }
                else
                {
                    return 0;
                }
            }
        }
        

        public void UpdateFromScaledImu3Msg(MAVLink.mavlink_scaled_imu3_t msg)
        {
            IvmeX = msg.xacc;
            IvmeY = msg.yacc;
            IvmeZ = msg.zacc;
            GyroX = msg.xgyro;
            GyroY = msg.ygyro;
            GyroZ = msg.zgyro;
            MagX = msg.xmag;
            MagY = msg.ymag;
            MagZ = msg.zmag;
        }

        public void UpdateFromScaledImu2Msg(MAVLink.mavlink_scaled_imu2_t msg)
        {
            IvmeX = msg.xacc;
            IvmeY = msg.yacc;
            IvmeZ = msg.zacc;
            GyroX = msg.xgyro;
            GyroY = msg.ygyro;
            GyroZ = msg.zgyro;
            MagX = msg.xmag;
            MagY = msg.ymag;
            MagZ = msg.zmag;
        }

        public void UpdateFromScaledImuMsg(MAVLink.mavlink_scaled_imu_t msg)
        {
            IvmeX = msg.xacc;
            IvmeY = msg.yacc;
            IvmeZ = msg.zacc;
            GyroX = msg.xgyro;
            GyroY = msg.ygyro;
            GyroZ = msg.zgyro;
            MagX = msg.xmag;
            MagY = msg.ymag;
            MagZ = msg.zmag;
        }

        public void UpdateFromRawImuMsg(MAVLink.mavlink_raw_imu_t msg)
        {
            IvmeX = msg.xacc;
            IvmeY = msg.yacc;
            IvmeZ = msg.zacc;
            GyroX = msg.xgyro;
            GyroY = msg.ygyro;
            GyroZ = msg.zgyro;
            MagX = msg.xmag;
            MagY = msg.ymag;
            MagZ = msg.zmag;
        }
    }
}
