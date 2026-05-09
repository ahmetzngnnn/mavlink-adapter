using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    /// <summary>
    /// Barometre bilgisini tutan sınıf
    /// </summary>
    public class BarometreBilgisi
    {
        public BarometreBilgisi()
        {
            YaklasikBasinc = 0;
            Sicaklik = 0;
        }

        // Yaklaşık Basınç [hPa]
        public float YaklasikBasinc { get; set; }

        // Sıcaklık [°C]
        public int Sicaklik { get; set; }
        

        public void UpdateFromScaledPressure2Msg(MAVLink.mavlink_scaled_pressure2_t msg)
        {
            YaklasikBasinc = msg.press_abs;
            Sicaklik = msg.temperature;
        }

        public void UpdateFromScaledPressureMsg(MAVLink.mavlink_scaled_pressure_t msg)
        {
            YaklasikBasinc = msg.press_abs;
            Sicaklik = msg.temperature;
        }
    }
}
