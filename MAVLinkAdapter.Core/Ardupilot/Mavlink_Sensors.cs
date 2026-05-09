using System.Collections;

namespace MAVLinkAdapter.Core.Ardupilot;

public partial class CurrentState
{
    /// <summary>
    /// derived from MAV_SYS_STATUS_SENSOR
    /// </summary>
    public class Mavlink_Sensors
    {
        BitArray bitArray = new BitArray(32);

        public bool seen = false;

        public Mavlink_Sensors()
        {
            //var item = MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL;
        }

        public Mavlink_Sensors(uint p)
        {
            seen = true;
            bitArray = new BitArray(new int[] { (int)p });
        }

        public bool gyro
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO)] = value; }
        }

        public bool accelerometer
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL)] = value; }
        }

        public bool compass
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG)] = value; }
        }

        public bool barometer
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE)] = value; }
        }

        public bool differential_pressure
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.DIFFERENTIAL_PRESSURE)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.DIFFERENTIAL_PRESSURE)] = value; }
        }

        public bool gps
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.GPS)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.GPS)] = value; }
        }

        public bool optical_flow
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.OPTICAL_FLOW)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.OPTICAL_FLOW)] = value; }
        }

        public bool VISION_POSITION
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.VISION_POSITION)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.VISION_POSITION)] = value; }
        }

        public bool LASER_POSITION
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.LASER_POSITION)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.LASER_POSITION)] = value; }
        }

        public bool GROUND_TRUTH
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.EXTERNAL_GROUND_TRUTH)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.EXTERNAL_GROUND_TRUTH)] = value; }
        }

        public bool rate_control
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.ANGULAR_RATE_CONTROL)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.ANGULAR_RATE_CONTROL)] = value; }
        }

        public bool attitude_stabilization
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.ATTITUDE_STABILIZATION)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.ATTITUDE_STABILIZATION)] = value; }
        }

        public bool yaw_position
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.YAW_POSITION)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.YAW_POSITION)] = value; }
        }

        public bool altitude_control
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.Z_ALTITUDE_CONTROL)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.Z_ALTITUDE_CONTROL)] = value; }
        }

        public bool xy_position_control
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.XY_POSITION_CONTROL)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.XY_POSITION_CONTROL)] = value; }
        }

        public bool motor_control
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MOTOR_OUTPUTS)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MOTOR_OUTPUTS)] = value; }
        }

        public bool rc_receiver
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.RC_RECEIVER)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.RC_RECEIVER)] = value; }
        }

        public bool gyro2
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO2)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO2)] = value; }
        }

        public bool accel2
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL2)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL2)] = value; }
        }

        public bool mag2
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG2)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG2)] = value; }
        }

        public bool geofence
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE)] = value; }
        }

        public bool ahrs
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS)] = value; }
        }

        public bool terrain
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN)] = value; }
        }

        public bool revthrottle
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR)] = value; }
        }

        public bool logging
        {
            get { return bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING)]; }
            set { bitArray[ConvertValuetoBitmaskOffset((int)MAVLink.MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING)] = value; }
        }

        int ConvertValuetoBitmaskOffset(int input)
        {
            int offset = 0;
            for (int a = 0; a < sizeof(int) * 8; a++)
            {
                offset = 1 << a;
                if (input == offset)
                    return a;
            }
            return 0;
        }

        public uint Value
        {
            get
            {
                int[] array = new int[1];
                bitArray.CopyTo(array, 0);
                return (uint)array[0];
            }
            set
            {
                seen = true;
                bitArray = new BitArray(new int[] { (int)value });
            }
        }

        public override string ToString()
        {
            return Convert.ToString(Value, 2);
        }
    }
}