using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller
{
    public class MotorBilgisi
    {
        public MotorBilgisi()
        {
            ch1out = 0;
            ch2out = 0;
            ch3out = 0;
            ch4out = 0;
            ch5out = 0;
            ch6out = 0;
            ch7out = 0;
            ch8out = 0;
            ch9out = 0;
            ch10out = 0;
            ch11out = 0;
            ch12out = 0;
            ch13out = 0;
            ch14out = 0;
            ch15out = 0;
            ch16out = 0;
        }
        public float ch1out { get; set; }
        public float ch2out { get; set; }
        public float ch3out { get; set; }
        public float ch4out { get; set; }
        public float ch5out { get; set; }
        public float ch6out { get; set; }
        public float ch7out { get; set; }
        public float ch8out { get; set; }

        public float ch9out { get; set; }
        public float ch10out { get; set; }
        public float ch11out { get; set; }
        public float ch12out { get; set; }
        public float ch13out { get; set; }
        public float ch14out { get; set; }
        public float ch15out { get; set; }
        public float ch16out { get; set; }
        
        public void UpdateFromServoOutRawMsg(MAVLink.mavlink_servo_output_raw_t msg)
        {
            ch1out = msg.servo1_raw;
            ch2out = msg.servo2_raw;
            ch3out = msg.servo3_raw;
            ch4out = msg.servo4_raw;
            ch5out = msg.servo5_raw;
            ch6out = msg.servo6_raw;
            ch7out = msg.servo7_raw;
            ch8out = msg.servo8_raw;

            // mavlink 2 uzatma
            ch9out = msg.servo9_raw;
            ch10out = msg.servo10_raw;
            ch11out = msg.servo11_raw;
            ch12out = msg.servo12_raw;
            ch13out = msg.servo13_raw;
            ch14out = msg.servo14_raw;
            ch15out = msg.servo15_raw;
            ch16out = msg.servo16_raw;
        }
    }
}
