using log4net;
using System.Drawing;
using System.Net.NetworkInformation;

namespace MAVLinkAdapter.Core.Modeller
{
    /// <summary>
    /// EKF bilgilerini tutar
    /// </summary>
    public class EkfBilgisi
    {

        // EKF'nin yatay pozisyon (mutlak) tahmininin iyi olduğunu gösteren boolean
        private bool _ekfYatayPozTahminiIyi = false;
        // EKF'nin sabit pozisyon modunda olduğunu ve mutlak veya göreceli pozisyonunu bilmediğini gösteren boolean
        private bool _ekfSabitPozisyonModu = false;
        // EKF'nin tahmini yatay pozisyon (mutlak) tahmininin iyi olduğunu gösteren boolean
        private bool _ekfTahminiYatayPozTahminiIyi = false;


        /// <summary>
        /// Yeni bir EKF bilgisi nesnesi oluşturur
        /// </summary>
        public EkfBilgisi()
        {
            YatayHizVaryansi = 0;
            DikeyHizVaryansi = 0;
            Durum = 0;
            PusulaVaryansi = 0;
            HizVaryansi = 0;
            AraziVaryansi = 0;
            Bayrak = 0;
        }

        /// <summary>
        /// EKF durumu. (0.5 ve 0.5'ten küçükse iyi, 0.5'ten büyükse riskli, 0.8'den büyükse kötü)
        /// </summary>
        public float Durum { get; set; }

        /// <summary>
        /// Ekf'nin iyi durumda olup olmadığını gösterir. true ise iyi, false ise kötü ekf verisi olduğu anlamına gelir.
        /// </summary>
        /// <param name="armDurumu">Aracın arm durumu (değerlendirme için gerekli)</param>
        /// <returns> EKF'nin iyi durumda olup olmadığını gösterir. true ise iyi, false ise kötü ekf verisi olduğu anlamına gelir.</returns>
        public bool EkfOkMu(bool armDurumu)
        {
            if (armDurumu)
            {
                return _ekfYatayPozTahminiIyi && !_ekfSabitPozisyonModu && Durum <= 0.5;
            }
            else
            {
                return _ekfYatayPozTahminiIyi && _ekfTahminiYatayPozTahminiIyi && Durum <= 0.5;
            }
        }

        /// <summary>
        /// EKF bayrağı  (ekf flag)
        /// </summary>
        public int Bayrak { get; set; }

        /// <summary>
        /// EKF hız varyansı
        /// </summary>
        public float HizVaryansi { get; set; }

        /// <summary>
        /// EKF pusula varyansı
        /// </summary>
        public float PusulaVaryansi { get; set; }

        /// <summary>
        /// EKF yatay hız varyansı
        /// </summary>
        public float YatayHizVaryansi { get; set; }

        /// <summary>
        /// EKF dikey hız varyansı
        /// </summary>
        public float DikeyHizVaryansi { get; set; }

        /// <summary>
        /// EKF arazi varyansı (terrarin)
        /// </summary>
        public float AraziVaryansi { get; set; }



        /// <summary>
        /// MAVLink ekf_status_report_t mesajından EKF bilgisi nesnesi oluşturur
        /// </summary>
        /// <param name="ekfstatusm"> MAVLink ekf_status_report_t mesajı</param>
        /// <returns> EKF bilgisi nesnesi</returns>
        public void UpdateFromEkfStatusReport(MAVLink.mavlink_ekf_status_report_t ekfstatusm, GpsBilgisi gps)
        {
            Bayrak = ekfstatusm.flags;
            HizVaryansi = ekfstatusm.velocity_variance;
            PusulaVaryansi = ekfstatusm.compass_variance;
            YatayHizVaryansi = ekfstatusm.pos_horiz_variance;
            DikeyHizVaryansi = ekfstatusm.pos_vert_variance;
            AraziVaryansi = ekfstatusm.terrain_alt_variance;

            Durum = Math.Max(HizVaryansi,
                Math.Max(PusulaVaryansi, Math.Max(YatayHizVaryansi, Math.Max(DikeyHizVaryansi, AraziVaryansi))));


            for (var a = 1; a <= (int)MAVLink.EKF_STATUS_FLAGS.EKF_UNINITIALIZED; a = a << 1)
            {
                var currentbit = ekfstatusm.flags & a;
                var currentflag = (MAVLink.EKF_STATUS_FLAGS)Enum.Parse(typeof(MAVLink.EKF_STATUS_FLAGS), a.ToString());

                if (currentbit == 0)
                {
                    switch (currentflag)
                    {
                        case MAVLink.EKF_STATUS_FLAGS.EKF_ATTITUDE: // step 1
                            Durum = 1;
                            break;
                        case MAVLink.EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ: // with pos
                            if (gps.Durum > 0)
                            {
                                // we have gps and dont have vel_hoz
                                Durum = 1;
                            }

                            break;
                        case MAVLink.EKF_STATUS_FLAGS.EKF_VELOCITY_VERT: // with pos
                                                                         //case MAVLink.EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL: // optical flow
                        case MAVLink.EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS: // step 1
                        case MAVLink.EKF_STATUS_FLAGS.EKF_POS_VERT_ABS: // step 1
                                                                        //case MAVLink.EKF_STATUS_FLAGS.EKF_POS_VERT_AGL: //  range finder
                                                                        //case MAVLink.EKF_STATUS_FLAGS.EKF_CONST_POS_MODE:  // never true when absolute - non gps
                                                                        //case MAVLink.EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL: // optical flow
                        case MAVLink.EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS: // ekf has origin - post arm
                                                                              //messageHigh = Strings.ERROR + " " + currentflag.ToString().Replace("_", " ");
                                                                              //messageHighTime = DateTime.Now;
                            break;

                    }
                }
                else
                {

                    if (a == (int)MAVLink.EKF_STATUS_FLAGS.EKF_UNINITIALIZED)
                    {
                        Durum = 1;
                    }
                    else
                    {
                        switch (currentflag)
                        {
                            case MAVLink.EKF_STATUS_FLAGS.EKF_POS_VERT_ABS:
                                _ekfYatayPozTahminiIyi = true;
                                break;
                            case MAVLink.EKF_STATUS_FLAGS.EKF_CONST_POS_MODE:
                                _ekfSabitPozisyonModu = true;
                                break;
                            case MAVLink.EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS:
                                _ekfTahminiYatayPozTahminiIyi = true;
                                break;

                        }
                    }
                }
            }

        }
    }
}