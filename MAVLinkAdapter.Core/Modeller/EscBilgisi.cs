using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class EscBilgisi
    {
        public float Volt { get; set; }
        public float Akim { get; set; }
        public float Rpm { get; set; }
        public float Sicaklik { get; set; }
    }
}
