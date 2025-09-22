namespace Hub.Models
{
    public class ImuSample
    {
        public int Id { get; set; }
        public DateTime ReceivedAt { get; set; }   // hub receive time
        public ulong RoverTimeNs { get; set; }     // rover timestamp

        // Quaternion
        public float QuaternionW { get; set; }
        public float QuaternionX { get; set; }
        public float QuaternionY { get; set; }
        public float QuaternionZ { get; set; }

        // Acceleration (m/s²)
        public float AccelX { get; set; }
        public float AccelY { get; set; }
        public float AccelZ { get; set; }

        // Gyroscope (deg/s)
        public float GyroX { get; set; }
        public float GyroY { get; set; }
        public float GyroZ { get; set; }

        // Magnetometer (µT)
        public float MagX { get; set; }
        public float MagY { get; set; }
        public float MagZ { get; set; }

        // Euler angles (degrees)
        public float Heading { get; set; } // eX
        public float Roll { get; set; }    // eY
        public float Pitch { get; set; }   // eZ

        // Linear acceleration (m/s²)
        public float LinearAccelX { get; set; }
        public float LinearAccelY { get; set; }
        public float LinearAccelZ { get; set; }

        public float TemperatureC { get; set; }

        // Calibration status (0..3)
        public uint CalibSys { get; set; }
        public uint CalibGyro { get; set; }
        public uint CalibAccel { get; set; }
        public uint CalibMag { get; set; }
    }
}