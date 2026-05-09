using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Ardupilot
{
    public class Serial
    {
        public static void print(object v)
        {
            if (v.GetType().IsArray)
                Console.Write(ASCIIEncoding.ASCII.GetString((byte[])v));
            else
                Console.Write(v);
        }
        public static void println(object v)
        {
            if (v.GetType().IsArray)
                Console.WriteLine(ASCIIEncoding.ASCII.GetString((byte[])v));
            else
                Console.WriteLine(v);
        }
    }
}
