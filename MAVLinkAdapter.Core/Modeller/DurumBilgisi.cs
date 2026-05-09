using MAVLinkAdapter.Core.Utilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class DurumBilgisi
    {
        private float _yaw;
        
        public DurumBilgisi()
        {
            Roll = 0;
            Pitch = 0;
            Yaw = 0;
            YanKaymaAcisi = 0;
            HucumAcisi = 0;
            YerYonu = 0;
        }

        // Roll (derece)
        public float Roll { get; set; }
        
        // Roll Hızı (derece/saniye)
        public float RollHizi { get; set; }

        // Pitch (derece)
        public float Pitch { get; set; }
        
        // Pitch Hızı (derece/saniye)
        public float PitchHizi { get; set; }

        // Yaw (derece)
        public float Yaw
        {
            get => _yaw;
            set
            {
                if (value < 0)
                    _yaw = value + 360;
                else
                    _yaw = value;
            }
        }

        // Yaw Hızı (derece/saniye)
        public float YawHizi { get; set; }

        // Yan Kayma Açısı (derece)
        public float YanKaymaAcisi { get; set; }

        // Hücum Açısı (derece)
        public float HucumAcisi { get; set; }

        // GroundCourse (derece)
        public float YerYonu { get; set; }

        public override string ToString()
        {
            return string.Format("Roll = {0}°, Pitch = {1}°, Yaw = {2}°, Yan Kayma Açısı = {3}°, Hücum Açısı = {4}°, Yer Yönü = {5}°",
                Roll, Pitch, Yaw, YanKaymaAcisi, HucumAcisi, YerYonu);
        }

        public void UpdateFromAoaSsaMsg(MAVLink.mavlink_aoa_ssa_t msg)
        {
            HucumAcisi = msg.AOA;
            YanKaymaAcisi = msg.SSA;
        }

        public void UpdateFromAttitudeMsg(MAVLink.mavlink_attitude_t msg)
        {
            Roll = (float)(msg.roll * MathHelper.rad2deg);
            RollHizi = (float)(msg.rollspeed * MathHelper.rad2deg);
            Pitch = (float)(msg.pitch * MathHelper.rad2deg);
            PitchHizi = (float)(msg.pitchspeed * MathHelper.rad2deg);
            Yaw = (float)(msg.yaw * MathHelper.rad2deg);
            YawHizi = (float)(msg.yawspeed * MathHelper.rad2deg);
        }

        public void UpdateFromHighLatencyMsg(MAVLink.mavlink_high_latency_t msg)
        {
            Roll = msg.roll / 100f;
            Pitch = msg.pitch / 100f;
            Yaw = msg.heading / 100f;
        }
    }
}
