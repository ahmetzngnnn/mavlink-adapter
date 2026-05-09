using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class BataryaBilgisi
    {
        public BataryaBilgisi()
        {
            Voltaj = 0;
            Akim = 0;
            Kalan = 0;
            KullanilanMah = 0;
            Hucre1 = 0;
            Hucre2 = 0;
            Hucre3 = 0;
            Hucre4 = 0;
            Hucre5 = 0;
            Hucre6 = 0;
            Sicaklik = 0;
        }


        // Batarya voltajı (Volt)
        public double Voltaj { get; set; }

        // Batarya akımı (Amper)
        public double Akim { get; set; }

        // Batarya kalan kapasite Yüzdesi (%)
        public int Kalan { get; set; }

        // Batarya gücü (Watt)
        public double Guc => Voltaj * Akim;


        // Batarya verimliliği (mah/km)
        public double BataryaVerimliligiHesapla(float gidilenMesafe)
        {
            if (KullanilanMah <= 0)
                return 0;

            return KullanilanMah / (gidilenMesafe / 1000.0f);
        }

        // Kalan batarya ile gidilebilecek mesafe (km)
        public double KalanMesafeHesapla(float gidilenMesafe)
        {
            var bataryaVerimliligi = BataryaVerimliligiHesapla(gidilenMesafe);

            if (bataryaVerimliligi <= 0)
                return 0;

            return (100.0f / (100.0f - Kalan) * KullanilanMah - KullanilanMah) / bataryaVerimliligi;
        }


        // Tahmini kullanılan batarya kapasitesi (mah)
        public double KullanilanMah { get; set; }

        // Batarya 1.hücre voltajı (V)
        public double Hucre1 { get; set; }

        // Batarya 2.hücre voltajı (V)
        public double Hucre2 { get; set; }

        // Batarya 3.hücre voltajı (V)
        public double Hucre3 { get; set; }

        // Batarya 4.hücre voltajı (V)
        public double Hucre4 { get; set; }

        // Batarya 5.hücre voltajı (V)
        public double Hucre5 { get; set; }

        // Batarya 6.hücre voltajı (V)
        public double Hucre6 { get; set; }

        // Batarya sıcaklığı (°C)
        public double Sicaklik { get; set; }

        public override string ToString() => $"Batarya: Voltaj={Voltaj}V, Akım={Akim}A, Seviye=%{Kalan}";


        public void UpdatefromBattMsg(MAVLink.mavlink_battery2_t msg)
        {
            Voltaj = msg.voltage / 1000.0f;
            Akim = msg.current_battery / 100.0f;
        }

        public void UpdatefromSysStatusMsg(MAVLink.mavlink_sys_status_t msg)
        {
            Voltaj = msg.voltage_battery / 1000.0f;
            Kalan = msg.battery_remaining;
            Akim = msg.current_battery / 100.0f;
        }
    }
}
