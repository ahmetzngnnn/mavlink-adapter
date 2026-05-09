using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class SonarBilgisi
    {
        public SonarBilgisi()
        {
            Uzaklik = 0;
            Voltaj = 0;
        }

        // Sonar ile ölçülen mesafe
        public float Uzaklik { get; set; }

        // Sonarın ölçüm yaptığı anki voltaj değeri
        public float Voltaj { get; set; }
        

        public void UpdateFromRangefinderMsg(MAVLink.mavlink_rangefinder_t msg)
        {
            Uzaklik = msg.distance;
            Voltaj = msg.voltage;
        }
    }
}
