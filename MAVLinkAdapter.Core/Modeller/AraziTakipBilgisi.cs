using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static MAVLink;

namespace MAVLinkAdapter.Core.Modeller
{
    public class AraziTakipBilgisi
    {

        public bool Aktif { get; set; }


        /// <summary>
        /// Ortalama Deniz Seviyesinin Üstündeki Arazi Yüksekliği [m]
        /// </summary>
        public float Irtifa { get; set; }

        /// <summary>
        /// Enlem/boylam arazi yüksekliği üzerindeki mevcut araç yüksekliği [m]
        /// </summary>
        public float GuncelIrtifa { get; set; }

        /// <summary>
        /// ızgara aralığı (bu konumda arazi mevcut değilse sıfır)
        /// </summary>
        public ushort IzgaraAraligi {get; set; }

        /// <summary>
        /// Bellekten alınmayı veya okunmayı bekleyen 4x4 arazi bloklarının sayısı
        /// </summary>
        /// 
        public ushort BekleyenBlokSayisi;


        /// <summary>
        /// Bellekteki 4x4 arazi bloklarının sayısı
        /// </summary>
        public ushort BellektekiBlokSayisi;
        

        public void UpdateFromTerrainReportMsg(mavlink_terrain_report_t msg)
        {
            Irtifa = msg.terrain_height;
            GuncelIrtifa = msg.current_height;
            IzgaraAraligi = msg.spacing;
            BekleyenBlokSayisi = msg.pending;
            BellektekiBlokSayisi = msg.loaded;
        }
    }
}
