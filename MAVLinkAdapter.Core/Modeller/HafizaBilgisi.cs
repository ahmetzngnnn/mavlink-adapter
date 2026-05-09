using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class HafizaBilgisi
    {
        
        // Serbest bellek miktarı
        public float KalanKapasite { get; set; }

        // Serbest bellek miktarı (32 bit)
        public float KalanKapasite32 { get; set; }

        //Ayrılmış heap alanının en üst sınırı
        public float AyrilmisBellekAlani { get; set; }
        

        public void UpdateFromMemInfoMsg(MAVLink.mavlink_meminfo_t msg)
        {
            KalanKapasite = msg.freemem;
            KalanKapasite32 = msg.freemem32;
            AyrilmisBellekAlani = msg.brkval;
        }
    }
}
