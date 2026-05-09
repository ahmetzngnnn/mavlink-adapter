using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class NavigasyonKontrolBilgisi
    {
        // NAV_CONTROLLER_OUTPUT mesajı için varsayılan oluşturucu
        public NavigasyonKontrolBilgisi()
        {
            NavRoll = 0;
            NavPitch = 0;
            NavBearing = 0;
            HedefYonelmeAcisi = 0;
            BirSonrakiNoktayaOlanMesafe = 0;
            IrtifaHatasi = 0;
            HavaHiziHatasi = 0;
            CaprazIzlemeHatasi = 0;
        }
        // Mevcut istenen yuvarlanma açısı
        public float NavRoll { get; set; }

        // Mevcut istenen duruş açısı
        public float NavPitch { get; set; }

        // Mevcut istenen baş açısı
        public int NavBearing { get; set; }

        // Mevcut hedefe veya noktaya olan yön
        public int HedefYonelmeAcisi { get; set; }

        // Aktif hedef noktasına olan mesafe
        public ushort BirSonrakiNoktayaOlanMesafe { get; set; }

        // Mevcut yükseklik hatası
        public float IrtifaHatasi { get; set; }

        // Mevcut hava hızı hatası
        public float HavaHiziHatasi { get; set; }

        // X-Y düzlemindeki mevcut çapraz izleme hatası
        public float CaprazIzlemeHatasi { get; set; }

        public ushort SiradakiNokta { get; set; }
        
        public void UpdateFromNavCtrlMsg(MAVLink.mavlink_nav_controller_output_t msg)
        {
            this.NavRoll = msg.nav_roll;
            this.NavPitch = msg.nav_pitch;
            this.NavBearing = msg.nav_bearing;
            this.HedefYonelmeAcisi = msg.target_bearing;
            this.BirSonrakiNoktayaOlanMesafe = msg.wp_dist;
            this.IrtifaHatasi = msg.alt_error;
            this.HavaHiziHatasi = msg.aspd_error / 100.0f;
            this.CaprazIzlemeHatasi = msg.xtrack_error;
        }

        public void UpdateFromMisssionCurrentMsg(MAVLink.mavlink_mission_current_t msg)
        {
            this.SiradakiNokta = msg.seq;
        }

        public void UpdateFromHighLatencyMsg(MAVLink.mavlink_high_latency_t msg)
        {
            SiradakiNokta = msg.wp_num;
            BirSonrakiNoktayaOlanMesafe = msg.wp_distance;
        }
    }
}
