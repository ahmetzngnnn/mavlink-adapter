using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    /// <summary>
    /// PID Tuning bilgilerini tutan sınıf
    /// </summary>
    public class PidTuningBilgisi
    {
        public PidTuningBilgisi()
        {
            FF = 0;
            P = 0;
            I = 0;
            D = 0;
            Eksenler = 0;
            IstenenOran = 0;
            EldeEdilmeOrani = 0;

        }

        //FF bileşeni.
        public float FF { get; set; }

        //P bileşeni.
        public float P { get; set; }

        //I bileşeni.
        public float I { get; set; }

        //D bileşeni.
        public float D { get; set; }

        //Eksenler. PID_TUNING_AXIS
        public byte Eksenler { get; set; }

        // İstenen oran.  [deg/s]
        public float IstenenOran { get; set; }

        // Elde edilme oranı. [deg/s]
        public float EldeEdilmeOrani { get; set; }
    }
}
