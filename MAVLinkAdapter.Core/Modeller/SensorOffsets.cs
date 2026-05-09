namespace MAVLinkAdapter.Core.Modeller
{
    /// <summary>
    ///  Sensor kalibrasyon değerlerini tutan sınıf
    /// </summary>
    public class SensorOffsets
    {
        /// <summary>
        /// Boş bir SensorOffsets nesnesi oluşturur
        /// </summary>
        public SensorOffsets()
        {
            this.PusulaOffsetX = 0;
            this.PusulaOffsetY = 0;
            this.PusulaOffsetZ = 0;

            this.PusulaSapma = 0;

            this.HamBasinc = 0;
            this.HamSicaklik = 0;

            this.JiroskopOffsetX = 0;
            this.JiroskopOffsetY = 0;
            this.JiroskopOffsetZ = 0;

            this.IvmeolcerOffsetX = 0;
            this.IvmeolcerOffsetY = 0;
            this.IvmeolcerOffsetZ = 0;
        }

        //Pusula x eksen kalibrasyon değeri
        public int PusulaOffsetX { get; set; }

        //Pusula y eksen kalibrasyon değeri
        public int PusulaOffsetY { get; set; }

        //Pusula z eksen kalibrasyon değeri
        public int PusulaOffsetZ { get; set; }

        //Manyetik sapma açısı  [radyan]
        public float PusulaSapma { get; set; }

        //Ham basınç değeri 
        public int HamBasinc { get; set; }

        //Ham sıcaklık değeri
        public int HamSicaklik { get; set; }

        //Jiroskop x eksen kalibrasyon değeri
        public float JiroskopOffsetX { get; set; }

        //Jiroskop y eksen kalibrasyon değeri
        public float JiroskopOffsetY { get; set; }

        //Jiroskop z eksen kalibrasyon değeri
        public float JiroskopOffsetZ { get; set; }

        //İvmeölçer x eksen kalibrasyon değeri
        public float IvmeolcerOffsetX { get; set; }

        //İvmeölçer y eksen kalibrasyon değeri
        public float IvmeolcerOffsetY { get; set; }

        //İvmeölçer z eksen kalibrasyon değeri
        public float IvmeolcerOffsetZ { get; set; }


        /// <summary>
        /// MAVLink mesajından SensorOffsets nesnesini günceller
        /// </summary>
        /// <param name="msg"> MAVLink mesajı </param>
        public void UpdateFromSensorOffsetsMsg(MAVLink.mavlink_sensor_offsets_t msg)
        {
            this.PusulaOffsetX = msg.mag_ofs_x;
            this.PusulaOffsetY = msg.mag_ofs_y;
            this.PusulaOffsetZ = msg.mag_ofs_z;

            this.PusulaSapma = msg.mag_declination;

            this.HamBasinc = msg.raw_press;
            this.HamSicaklik = msg.raw_temp;

            this.JiroskopOffsetX = msg.gyro_cal_x;
            this.JiroskopOffsetY = msg.gyro_cal_y;
            this.JiroskopOffsetZ = msg.gyro_cal_z;

            this.IvmeolcerOffsetX = msg.accel_cal_x;
            this.IvmeolcerOffsetY = msg.accel_cal_y;
            this.IvmeolcerOffsetZ = msg.accel_cal_z;

        }
    }
}