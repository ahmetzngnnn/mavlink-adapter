using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MAVLinkAdapter.Core.Ardupilot;
using MAVLinkAdapter.Core.Utilities;
using static MAVLink;

namespace MAVLinkAdapter.Core.Modeller
{
    public class GpsBilgisi
    {
        /// <summary>
        /// GPS verisi sınıfı oluşturur
        /// </summary>
        public GpsBilgisi()
        {

        }

        /// <summary>
        /// GPS verisi sınıfı oluşturur
        /// </summary>
        /// <param name="enlem"> Enlem (m) </param>
        /// <param name="boylam"> Boylam (m) </param>
        /// <param name="irtifa"> İrtifa (m) </param>
        /// <param name="durum"> GPS'in durumu </param>
        /// <param name="hdop"> GPS'in hassasiyeti </param>
        /// <param name="uyduSayisi"> GPS'in bağlı olduğu uydu sayısı </param>
        /// <param name="yerHizi"> Aracın yere göre hareket hızı (m/s) </param>
        /// <param name="yerYonu"> Aracın yeryüzündeki hareket yönü (derece) </param>
        public GpsBilgisi(double enlem, double boylam, float irtifa, float durum, float hdop, float uyduSayisi,
            float yerHizi, float yerYonu)
        {
            Enlem = enlem;
            Boylam = boylam;
            Irtifa = irtifa;
            Durum = (GPSDurum)durum;
            Hdop = hdop;
            UyduSayisi = uyduSayisi;
            YerHizi = yerHizi;
            YerYonu = yerYonu;
        }


        // Enlem (m)
        public double Enlem { get; private set; }

        // Boylam (m)
        public double Boylam { get; private set; }

        // İrtifa (m)
        public float Irtifa { get; private set; }


        // İrtifa (m) (deniz seviyesine göre)
        public float IrtifaDS { get; private set; }


        // GPS'in durumunu ifade eder
        public GPSDurum Durum { get; private set; }

        // GPS'in hassasiyetini ifade eder
        public float Hdop { get; private set; }

        // GPS'in bağlı olduğu uydu sayısını ifade eder
        public float UyduSayisi { get; private set; }

        // Aracın yere göre hareket hızını ifade eder (m/s)
        public float YerHizi { get; private set; }

        //Aracın yeryüzündeki hareket yönünü açısal olarak ifade eder (derece)
        public float YerYonu{ get; private set; }

        public float YatayDogruluk { get; private set; }
        public float DikeyDogruluk { get; private set; }
        public float HizDogrulugu { get; private set; }
        public float YerYonDogrulugu { get; private set; }

        public DateTime Saat { get; private set; }

        public override string ToString() => $"GPS Bilgisi: Durum={DurumToString()}, Uydu Sayısı={UyduSayisi}, Enlem = {Enlem}, Boylam = {Boylam}, Irtifa = {Irtifa}, Hız = {YerHizi}, Yön = {YerYonu}, Saat = {Saat}";


        public PointLatLngAlt ToPointLatLngAlt()
        {
            return new PointLatLngAlt(Enlem, Boylam, Irtifa);
        }

        public void UpdateFromGpsStatMsg(MAVLink.mavlink_gps_status_t msg)
        {
            UyduSayisi = msg.satellites_visible;
        }

        public void UpdateFromGpsRawMsg(MAVLink.mavlink_gps2_raw_t msg)
        {
            Enlem = msg.lat * 1.0e-7;
            Boylam = msg.lon * 1.0e-7;
            Irtifa = msg.alt / 1000.0f;

            Durum = (GPSDurum)msg.fix_type;
            Hdop = (float)Math.Round(msg.eph / 100.0, 2);

            UyduSayisi = msg.satellites_visible;

            YerHizi = msg.vel * 1.0e-2f;
            YerYonu = msg.cog * 1.0e-2f;

        }

        public void UpdateFromGpsRawMsg(MAVLink.mavlink_gps_raw_int_t msg, bool useLocation, bool isMavlink2)
        {
            if (!useLocation)
            {
                Enlem = msg.lat * 1.0e-7;
                Boylam = msg.lon * 1.0e-7;
                Irtifa = msg.alt / 1000.0f;
            }
            Durum = (GPSDurum)msg.fix_type;
            Hdop = (float)Math.Round(msg.eph / 100.0, 2);

            UyduSayisi = msg.satellites_visible;

            YerHizi = msg.vel * 1.0e-2f;

            if (YerHizi > 0.5)
                YerYonu = msg.cog * 1.0e-2f;

            if (isMavlink2)
            {
                YatayDogruluk = msg.h_acc / 1000.0f;
                DikeyDogruluk = msg.v_acc / 1000.0f;
                HizDogrulugu = msg.vel_acc / 1000.0f;
                YerYonDogrulugu = msg.hdg_acc / 1e5f;
            }
            else
            {
                YatayDogruluk = -1;
                DikeyDogruluk = -1;
                HizDogrulugu = -1;
                YerYonDogrulugu = -1;
            }
        }

        public void UpdateFromPositionMsg(mavlink_global_position_int_t msg, out bool useLocation)
        {
            // yeni ahrs ölü hesaplama 0 alt ve 0 uzun gönderebilir. kontrol edin ve geri alın
            Irtifa = msg.relative_alt / 1000.0f;

            useLocation = true;
            if (msg.lat == 0 && msg.lon == 0)
            {
                useLocation = false;
            }
            else
            {
                Enlem = msg.lat / 10000000.0;
                Boylam = msg.lon / 10000000.0;

                IrtifaDS = msg.alt / 1000.0f;
            }
        }

        public void UpdateFromSysTimeMsg(mavlink_system_time_t msg)
        {
            var date1 = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            try
            {
                date1 = date1.AddMilliseconds(msg.time_unix_usec / 1000);
            }
            catch
            {
            }

            Saat = date1;
        }

        /// <summary>
        /// GPSDurum bilgisini string olarak döndürür
        /// </summary>
        /// <returns> metin olarak GPSDurum bilgisi </returns>
        public string DurumToString()
        {
            switch (Durum)
            {
                case GPSDurum.NoGPS:
                    return "No GPS";
                case GPSDurum.NoFix:
                    return "No Fix";
                case GPSDurum.Fix2D:
                    return "2D Fix";
                case GPSDurum.Fix3D:
                    return "3D Fix";
                case GPSDurum.FixDGPS:
                    return "DGPS Fix";
                case GPSDurum.RtfFloat:
                    return "RTK Float";
                case GPSDurum.RtfFixed:
                    return "RTK Fixed";
                default:
                    return "Bilinmeyen GPS Durumu";
            }
        }

        public void UpdateFromHighLatencyMsg(mavlink_high_latency_t msg)
        {
            Enlem = msg.latitude / 1e7;
            Boylam = msg.longitude / 1e7;

            UyduSayisi = msg.gps_nsat;
            Durum = (GPSDurum)msg.gps_fix_type;

            IrtifaDS = msg.altitude_amsl;
            Irtifa = msg.altitude_sp;
        }
    }

    public enum GPSDurum
    {
        NoGPS = 0,
        NoFix = 1,
        Fix2D = 2,
        Fix3D = 3,
        FixDGPS = 4,
        RtfFloat = 5,
        RtfFixed = 6
    }
}
