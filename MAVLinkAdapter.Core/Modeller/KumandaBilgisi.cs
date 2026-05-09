using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MAVLinkAdapter.Core.Ardupilot;
using static MAVLink;

namespace MAVLinkAdapter.Core.Modeller
{
    public class KumandaBilgisi
    {

        public KumandaBilgisi()
        {
            ch1in = 0;
            ch2in = 0;
            ch3in = 0;
            ch4in = 0;
            ch5in = 0;
            ch6in = 0;
            ch7in = 0;
            ch8in = 0;
            ch9in = 0;
            ch10in = 0;
            ch11in = 0;
            ch12in = 0;
            ch13in = 0;
            ch14in = 0;
            ch15in = 0;
            ch16in = 0;

            rssi = 0;

            AlinanHatalar = 0;
            DuzeltilenPaketSayisi = 0;
            SinyalGucu = 0;
            UzakSinyalGucu = 0;
            KalanArabellek = 0;
            GurultuSeviyesi = 0;
            UzakGurultuSeviyesi = 0;


        }

        public float ch1in { get; set; }
        public float ch2in { get; set; }
        public float ch3in { get; set; }
        public float ch4in { get; set; }
        public float ch5in { get; set; }
        public float ch6in { get; set; }
        public float ch7in { get; set; }
        public float ch8in { get; set; }

        public float ch9in { get; set; }
        public float ch10in { get; set; }
        public float ch11in { get; set; }
        public float ch12in { get; set; }
        public float ch13in { get; set; }
        public float ch14in { get; set; }
        public float ch15in { get; set; }
        public float ch16in { get; set; }

        // kumandanın % sinyal kalitesi (rssi)
        public float rssi { get; set; }

        /// <summary>
        /// Alınan hatalar
        /// </summary>
        public ushort AlinanHatalar { get; set; }
        /// <summary>
        /// Hataları düzeltilmiş paketlerin sayısı
        /// </summary>
        public ushort DuzeltilenPaketSayisi { get; set; }

        /// <summary>
        /// Yerel sinyal gücü
        /// </summary>
        public byte SinyalGucu { get; set; }

        /// <summary>
        /// Uzak sinyal gücü
        /// </summary>
        public byte UzakSinyalGucu { get; set; }

        /// <summary>
        /// Kalan boş arabellek alanı.  [%]
        /// </summary>
        public byte KalanArabellek { get; set; }

        /// <summary>
        /// Arka plan gürültü seviyesi
        /// </summary>
        public byte GurultuSeviyesi { get; set; }

        /// <summary>
        /// Uzak arka plan gürültü seviyesi
        /// </summary>
        public byte UzakGurultuSeviyesi { get; set; }

        public void UpdateFromRadioStatMsg(MAVLink.mavlink_radio_status_t msg)
        {
            AlinanHatalar = msg.rxerrors;
            DuzeltilenPaketSayisi = msg.@fixed;
            SinyalGucu = msg.rssi;
            UzakSinyalGucu = msg.remrssi;
            KalanArabellek = msg.txbuf;
            GurultuSeviyesi = msg.noise;
            UzakGurultuSeviyesi = msg.remnoise;
        }

        public void UpdateFromRadioMsg(MAVLink.mavlink_radio_t msg)
        {
            AlinanHatalar = msg.rxerrors;
            DuzeltilenPaketSayisi = msg.@fixed;
            SinyalGucu = msg.rssi;
            UzakSinyalGucu = msg.remrssi;
            KalanArabellek = msg.txbuf;
            GurultuSeviyesi = msg.noise;
            UzakGurultuSeviyesi = msg.remnoise;
        }

        public void UpdateFromRcChannelsMsg(mavlink_rc_channels_t msg)
        {
            ch1in = msg.chan1_raw;
            ch2in = msg.chan2_raw;
            ch3in = msg.chan3_raw;
            ch4in = msg.chan4_raw;
            ch5in = msg.chan5_raw;
            ch6in = msg.chan6_raw;
            ch7in = msg.chan7_raw;
            ch8in = msg.chan8_raw;
            ch9in = msg.chan9_raw;
            ch10in = msg.chan10_raw;
            ch11in = msg.chan11_raw;
            ch12in = msg.chan12_raw;
            ch13in = msg.chan13_raw;
            ch14in = msg.chan14_raw;
            ch15in = msg.chan15_raw;
            ch16in = msg.chan16_raw;
            rssi = (int)(msg.rssi / 255.0 * 100.0);
        }

        public void UpdateFromRcChannelsRawMsg(mavlink_rc_channels_raw_t msg)
        {
            ch1in = msg.chan1_raw;
            ch2in = msg.chan2_raw;
            ch3in = msg.chan3_raw;
            ch4in = msg.chan4_raw;
            ch5in = msg.chan5_raw;
            ch6in = msg.chan6_raw;
            ch7in = msg.chan7_raw;
            ch8in = msg.chan8_raw;
            rssi = (int)(msg.rssi / 255.0 * 100.0);
        }

        public void UpdateFromHighLatencyMsg(mavlink_high_latency_t msg)
        {
            ch3in = msg.throttle;
        }
    }
}
