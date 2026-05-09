using MAVLinkAdapter.Core.Ardupilot;
using MAVLinkAdapter.Core.Communication;
using MAVLinkAdapter.Core.Modeller;
using MAVLinkAdapter.Core.Utilities;

namespace MAVLinkAdapter.Core;

/// <summary>
///     Mavlink bağlantısını sağlayan sınıf
/// </summary>
public class MavlinkAdapter
{
    /// <summary>
    ///     Pasif bağlantılar
    /// </summary>
    public static List<MavLinkArayuzu> Comports = new();

    #region ### public özellikler (properties) ###
    /// <summary>
    ///     Aktif ComPort Arayüzü
    /// </summary>
    public MavLinkArayuzu ComPort
    {
        get => _comPort;
        set
        {
            if (_comPort != null && _comPort == value)
                return;
            _comPort = value;
        }
    }

    /// <summary>
    /// Aracın 'arm' edilebilir olup olmadığını belirtir. 
    /// </summary>
    public bool ArmEdilebilirMi =>
        Mod != "INITIALISING" &&
        Gps != null && (Gps.Durum > (GPSDurum)1)
        && ComPort.MAV.cs.Ekf.EkfOkMu(ArmEdildi);


    /// <summary>
    /// Aracın modunu verir. (örn: STABILIZE, GUIDED, vs.)
    /// </summary>
    public string Mod => ComPort.MAV.cs.Mod;

    /// <summary>
    /// Aracın 'arm' edilip edilmediğini verir
    /// </summary>
    public bool ArmEdildi => ComPort.MAV.cs.Armed;

    /// <summary>
    /// Aracın EKF durumu ile ilgili bilgileri tutar.(EKF durumu, vs.)
    /// </summary>
    public EkfBilgisi Ekf => ComPort.MAV.cs.Ekf;

    /// <summary>
    /// Aracın GPS durumu ile ilgili bilgileri tutar.(enlem, boylam, irtifa, vs.)
    /// </summary>
    public GpsBilgisi Gps => ComPort.MAV.cs.GpsBilgisi;

    /// <summary>
    /// Aracın durumu ile ilgili bilgileri tutar.(pitch, roll, yaw, irtifa, hız, vs.)
    /// </summary>
    public DurumBilgisi Durum => ComPort.MAV.cs.Durum;

    /// <summary>
    /// Aracın batarya durumu ile ilgili bilgileri tutar.(voltaj, akım, vs.)
    /// </summary>
    public BataryaBilgisi Batarya => ComPort.MAV.cs.Batarya;

    /// <summary>
    /// Aracın yer hızını verir. (m/s). Set edilirse araca yeni yer hızı gönderir.
    /// </summary>
    public float YerHizi
    {
        get => ComPort.MAV.cs.YerHizi;
        set
        {
            try
            {
                ComPort?.KomutGonder(MAVLink.MAV_CMD.DO_CHANGE_SPEED, 1, value, 0, 0, 0, 0, 0);
            }
            catch
            {
                // ignored
            }
        }
    }

    /// <summary>
    /// Aracın hava hızını verir. (m/s). Set edilirse araca yeni hava hızı gönderir.
    /// </summary>
    public float HavaHizi
    {
        get => ComPort.MAV.cs.HavaHizi;
        set
        {
            try
            {
                ComPort?.KomutGonder(MAVLink.MAV_CMD.DO_CHANGE_SPEED, 0, value, 0, 0, 0, 0, 0);
            }
            catch
            {
                // ignored
            }
        }
    }

    /// <summary>
    /// Kumanda kanal baskılamalarını tutar. Kanal baskılamaları değiştirildiğinde KanalBaskilamaGonder event'i tetiklenir.
    /// </summary>
    public KumandaKanalBaskilama KanalBaskilama
    {
        get => _baskilamalar;
        set
        {
            _baskilamalar.Aktif = false;
            _baskilamalar.Clear();
            foreach (var kvp in value)
            {
                _baskilamalar[kvp.Key] = kvp.Value;
            }

            _baskilamalar.Aktif = true;
            KanalBaskilamaGonder(this, EventArgs.Empty);
        }
    }

    public RotaBilgisi rotaBilgisi { get; set; }

    #endregion

    #region ### private değişkenler (fields) ###
    /// <summary>
    ///     ilk bağlandığımız zamanı tutar
    /// </summary>
    private readonly DateTime _connecttime = DateTime.Now;

    private readonly DateTime _nodatawarning = DateTime.Now;

    private readonly ManualResetEvent _serialThreadrunner = new(false);

    private MavLinkArayuzu _comPort = new();

    /// <summary>
    ///     en son nabız gönderilme zamanını tutar
    /// </summary>
    private DateTime _heatbeatSend = DateTime.Now;

    /// <summary>
    ///     ana serial okuyucu thread'ini kontrol eder
    /// </summary>
    private bool _serialThread;

    /// <summary>
    ///    Kumanda kanal baskılamalarını tutar
    /// </summary>
    private KumandaKanalBaskilama _baskilamalar;

    #endregion

    /// <summary>
    ///  Mavlink Araçlarıyla iletişim kurmak için kullanılan sınıf oluşturur.
    /// </summary>
    public MavlinkAdapter()
    {
        NabizGonder();

        MavLinkArayuzu.YkiSistemId = MavLinkArayuzu.YkiSistemId;

        // define default basestream
        ComPort.BaseStream = new SerialPort();
        ComPort.BaseStream.BaudRate = 115200;

        //MissionPlanner.Utilities.Tracking.cid = new Guid(Settings.Instance["guid"].ToString());
        ComPort.MAV.cs.ResetInternals();

        _baskilamalar = new KumandaKanalBaskilama();
        _baskilamalar.KanalBaskilamaGonder += KanalBaskilamaGonder;

        rotaBilgisi = new RotaBilgisi(this);

        Comports.Add(ComPort);
        ComPort.MavChanged += comPort_MavChanged;


        ComPort.MAV.cs.HomeLocation = new PointLatLngAlt();

        var serialreaderthread = new Thread(SerialReader)
        {
            IsBackground = true,
            Name = "Main Serial reader",
            Priority = ThreadPriority.AboveNormal
        };
        serialreaderthread.Start();
    }


    #region ### public metotlar ###

    /// <summary>
    /// Verilen bağlantı bilgilerine göre araca bağlanır.
    /// </summary>
    /// <param name="baglantiMetni"> Cihaza bağlanılacak bağlantı metni (örn: COM3, tcp:127.0.0.1:5760 , vs.) </param>
    /// <param name="baud"> Bağlantı hızı </param>
    /// <param name="parametreleriAl"> Bağlantı kurulduktan sonra aracın parametrelerini alıp almayacağını belirler </param>
    /// <returns> Bağlantı başarılı ise true, aksi halde false döner </returns>
    public bool Baglan(string baglantiMetni, string baud = "9600", bool parametreleriAl = true)
    {
        Console.WriteLine("Araca {0} üzerinden bağlanılıyor...", baglantiMetni);

        ComPort.giveComport = false;
        ComPort.kayitDosyasi = null;
        ComPort.hamKayitDosyasi = null;

        Baglan(ComPort, baglantiMetni, baud, parametreleriAl);

        return ComPort.BaseStream.IsOpen;
    }

    /// <summary>
    /// Verilen bağlantı bilgilerine göre araca bağlanır.
    /// </summary>
    /// <param name="mavLink"> Bağlantı kurulacak MavLinkArayuzu nesnesi </param>
    /// <param name="baglantiMetni"> Cihaza bağlanılacak bağlantı metni (örn: COM3, tcp:127.0.0.1:5760 , vs.) </param>
    /// <param name="baud"> Bağlantı hızı </param>
    /// <param name="parametreleriAl"> Bağlantı kurulduktan sonra aracın parametrelerini alıp almayacağını belirler </param>
    public void Baglan(MavLinkArayuzu mavLink, string baglantiMetni, string baud="9600", bool parametreleriAl = true)
    {
        try
        {
            if (baglantiMetni.StartsWith("udpcl:")) // eğer baglantiMetni udpin: ile başlıyorsa
            {
                var hostandport = baglantiMetni.Substring(6).Split(':');

                if (hostandport.Length != 2)
                {
                    throw new Exception("UDP Client bağlantı metni host:port şeklinde olmalıdır.");
                }

                mavLink.BaseStream = new UdpSerialConnect(hostandport[0], hostandport[1]);
            }
            else if (baglantiMetni.StartsWith("udp:")) // eğer baglantiMetni udp: ile başlıyorsa
            {
                var hostandport = baglantiMetni.Substring(4).Split(':');

                if (hostandport.Length != 2)
                {
                    throw new Exception("UDP bağlantı metni host:port şeklinde olmalıdır.");
                }

                mavLink.BaseStream = new UdpSerial();
            }
            else if (baglantiMetni.StartsWith("tcp:")) // eğer baglantiMetni tcp: ile başlıyorsa
            {
                var hostandport = baglantiMetni.Substring(4).Split(':');

                if (hostandport.Length != 2)
                    throw new Exception("TCP bağlantı metni host:port şeklinde olmalıdır.");

                mavLink.BaseStream = new TcpSerial(hostandport[0], hostandport[1]);
            }
            else // hiçbiri değilse serial baglantiMetni oluştur
            {
                mavLink.BaseStream = new SerialPort();
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine(ex.Message);
            return;
        }


        mavLink.MAV.cs.ResetInternals();

        //oynatılan herhangi bir günlüğü temizleyin
        mavLink.logreadmode = false;
        mavLink.logplaybackfile = null;

        try
        {
            // portu ayarla
            if (baglantiMetni.ToLower() != "preset")
                mavLink.BaseStream.PortName = baglantiMetni;

            try
            {
                if (baud != "" && baud != "0" && !baglantiMetni.StartsWith("tcp:"))
                    mavLink.BaseStream.BaudRate = int.Parse(baud);
            }
            catch
            {
                // ignored
            }

            // serialreader'ın herhangi bir şey yapmasını engeller
            mavLink.giveComport = true;


            mavLink.giveComport = false;

            LogKaydiOlustur(mavLink);


            // bağlantıyı yap
            mavLink.Ac(parametreleriAl);

            if (!mavLink.BaseStream.IsOpen)
            {
                mavLink.Kapat();
                return;
            }

            if (parametreleriAl)
            {
                Console.WriteLine("Parametreler alınıyor...");
                mavLink.parametreListesiniGetir();
                Console.WriteLine("Parametreler alındı");
            }
        }
        catch (Exception)
        {
            mavLink.Kapat();
        }
    }

    /// <summary>
    ///  Bağlantıyı kapatır
    /// </summary>
    public void BaglantiyiKes()
    {
        try
        {
            ComPort.BaseStream.DtrEnable = false;
            ComPort.Kapat();
        }
        catch
        {
            // ignored
        }
    }

    /// <summary>
    /// Aracın 'arm' edilmesini sağlar. (araç arm edilebilir değilse arm edilmesini bekler sonra arm eder.)
    /// </summary>
    /// <param name="zorla"> Aracın 'arm' edilmesi için zorlama yapılıp yapılmayacağını belirler </param>
    /// <param name="bekle"> Aracın 'arm' edilmesini bekleyip beklemeyeceğini belirler </param>
    /// <param name="zamanAsimi"> Aracın 'arm' edilmesi için belirlenen zaman aşımı süresi </param>
    /// <returns> Aracın 'arm' edilip edilmediğini döner </returns>
    public bool ArmEt(bool zorla = false, bool bekle = false, int? zamanAsimi = 30)
    {
        if (!ArmEdilebilirMi)
            ArmEdilebilirOlmasiniBekle(zamanAsimi);

        var durum = ComPort.ArmEt(true, zorla);

        if (bekle)
        {
            Bekle(() => ArmEdildi, zamanAsimi,
                hataMesaji: "Aracı 'arm' etme zaman aşımına uğradı");
        }

        return durum;
    }

    /// <summary>
    /// Aracın 'disarm' edilmesini sağlar
    /// </summary>
    /// <param name="zorla"></param>
    /// <param name="bekle"></param>
    /// <param name="zamanAsimi"></param>
    /// <returns></returns>
    public bool DisarmEt(bool zorla = false, bool bekle = false, int? zamanAsimi = 30)
    {
        var durum = ComPort.ArmEt(false, zorla);

        if (bekle)
        {
            Bekle(() => !ArmEdildi, zamanAsimi,
                hataMesaji: "Aracı 'disarm' etme zaman aşımına uğradı");
        }

        return durum;
    }

    /// <summary>
    /// Araca Kalkış komutu gönderir.
    /// Eğer öncesinde araç Arm edilmediyse arm eder ve modu GUIDED değilse GUIDED yapar.
    /// (Eğer bekle parametresi true ise kalkışın yapılarak belirlenen irtifaya ulaşılmasını bekler)
    /// </summary>
    /// <param name="irtifa"> Kalkış yapılacak irtifa (m) </param>
    /// <param name="bekle"> Kalkışın yapıldıktan sonra belirlenen irtifaya ulaşılmasını bekleme durumu </param>
    /// <param name="zamanAsimi"> Kalkışın yapıldıktan sonra belirlenen irtifaya ulaşılması için belirlenen zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    /// <param name="epsilon"> Belirlenen irtifa için kabul edilebilir hata payı (m) ( örn: irtifa = 5, epsilon = 0.1 ise arac 4.9 ile 5.1 arasındaki irtifaya ulaştığında kalkış tamamlanmış sayılır) </param>
    public void KalkisYap(float irtifa, bool bekle = false, int? zamanAsimi = null, double epsilon = 0.1)
    {
        // Aracı arm et ve arm edilmesini bekle
        if (!ArmEdildi)
            ArmEt(true, true);

        // Aracın modunu GUIDED yapıp modun güncellenmesini bekle
        if (Mod.ToLower() != "GUIDED".ToLower())
            ModGuncelle("GUIDED", true, zamanAsimi);

        // Araca kalkis komutunu gönder
        ComPort.KalkisYap(irtifa);

        // Aracın belirtilen irtifaya ulaşmasını bekle
        if (bekle)
        {
            IrtifayiBekle(irtifa, epsilon, zamanAsimi);
        }
    }

    /// <summary>
    ///  Aracı LAND moduna geçirir.
    ///  (Eğer bekle parametresi true ise inişin yapılarak aracın 0 metre irtifaya ulaşmasını bekler)
    /// </summary>
    /// <param name="bekle"> İnişin yapıldıktan sonra aracın 0 metre irtifaya ulaşmasını bekleme durumu </param>
    /// <param name="zamanAsimi"> İnişin yapıldıktan sonra aracın 0 metre irtifaya ulaşması için belirlenen zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    /// <param name="epsilon"> Aracın 0 metre irtifaya ulaşması için kabul edilebilir hata payı (m) ( örn: irtifa = 0, epsilon = 0.1 ise arac -0.1 ile 0.1 arasındaki irtifaya ulaştığında iniş tamamlanmış sayılır) </param>
    public void InisYap(bool bekle = false, int? zamanAsimi = null, double epsilon = 0.1)
    {
        if (Mod.ToLower() != "LAND".ToLower())
            ModGuncelle("LAND", true, zamanAsimi);

        // Aracın belirtilen irtifaya ulaşmasını bekle
        if (bekle)
        {
            IrtifayiBekle(0, epsilon, zamanAsimi);
        }
    }


    /// <summary>
    /// Aracın belirli bir yönde döndürmek için kullanılır.
    /// </summary>
    /// <param name="derece"> Dönülecek açı (derece) </param>
    /// <param name="aracaGore"> Dönüşün araca göre mi yoksa dünyaya göre mi yapılacağını belirler. (araca göre: true, dünyaya göre: false) </param>
    /// <param name="yawHizi"> Dönüş hızı (derece/saniye) </param>
    /// <param name="cw"> Dönüş yönü (saat yönü: true, saat yönünün tersi: false) </param>
    public void YawDegistir(float derece, bool aracaGore, float yawHizi, bool cw)
    {
        ComPort.KomutGonder(MAVLink.MAV_CMD.CONDITION_YAW,
            derece, // dönülecek açı (derece)
            yawHizi,    // dönüş hızı (derece/saniye)
            cw ? 1 : -1, // dönüş yönü (saat yönü: 1, saat yönünün tersi: -1)
            aracaGore ? 1 : 0, // dönüşün araca göre mi yoksa dünyaya göre mi yapılacağını belirler (araca göre: 1, dünyaya göre: 0)
            0, 0, 0);
    }


    /// <summary>
    /// Aracın modunu günceller ( ve bekle parametresi true ise yeni mod ile güncellenmesini bekler)
    /// </summary>
    /// <param name="yeniMod"> Güncellenecek mod </param>
    /// <param name="bekle"> Aracın modunun yeni mod ile güncellenmesini bekleyip beklemeyeceğini belirler </param>
    /// <param name="zamanAsimi"> Aracın modunun yeni mod ile güncellenmesi için belirlenen zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    public void ModGuncelle(string yeniMod, bool bekle = false, int? zamanAsimi = null)
    {
        //modu güncelle
        ComPort.ModGuncelle(yeniMod);


        // aracın modunun yeni mod ile güncelenmesini bekle
        Bekle(() => Mod.ToLower() == yeniMod.ToLower(), zamanAsimi,
            hataMesaji: "Uçuş modunu ayarlama başarısız oldu");
    }

    /// <summary>
    /// Aracın belirli bir irtifaya ulaşmasını bekleyen metot
    /// </summary>
    /// <param name="irtifa"> Ulaşılması beklenen irtifa (m) </param>
    /// <param name="epsilon"> Ulaşılması beklenen irtifa için kabul edilebilir hata payı (m) </param>
    /// <param name="zamanAsimi"> Ulaşılması beklenen irtifaya ulaşılması için belirlenen zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    public void IrtifayiBekle(double irtifa, double epsilon = 0.1, int? zamanAsimi = null)
    {
        var oncekiIrtifa = ComPort.MAV.cs.GpsBilgisi.Irtifa;

        // Belirtilen irtifaya ulaşıp ulaşılmadığını kontrol eden metot
        bool IrtifaKontrol()
        {
            double suankiIrtifa = ComPort.MAV.cs.GpsBilgisi.Irtifa;
            double delta = Math.Abs(irtifa - suankiIrtifa); // istenen irtifa ile şuanki irtifa arasındaki fark

            return (
                (delta < epsilon) ||
                (suankiIrtifa > irtifa && irtifa > oncekiIrtifa) ||
                (suankiIrtifa < irtifa && irtifa < oncekiIrtifa)
            );
        }


        // Bekle metodunu kullanarak, belirtilen irtifaya ulaşılıp ulaşılmadığını kontrol et ve
        // belirtilen zaman aşımı süresi içinde irtifaya ulaşılmazsa hata fırlat
        Bekle(IrtifaKontrol, zamanAsimi, hataMesaji: "Belirtilen irtifaya ulaşılamadı");
    }

    /// <summary>
    /// Aracın belirtilen konuma gitmesini sağlar.
    /// (Eğer bekle parametresi true ise aracın belirtilen konuma ulaşmasını bekler)
    /// </summary>
    /// <param name="konum"> Aracın gideceği konum </param>
    /// <param name="bekle"> Aracın belirtilen konuma ulaşana kadar bekleyip beklememe durumu</param>
    /// <param name="zamanAsimi"> Aracın belirtilen konuma ulaşması için belirlenen zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    /// <param name="epsilon"> Aracın belirtilen konuma ulaşması için kabul edilebilir hata payı (m) ( örn: istenen konum ile aracın şuanki konumu arasındaki mesafe epsilon'dan küçükse aracın istenen konuma ulaştığı kabul edilir) </param>
    public void KonumaGit(PointLatLngAlt konum, bool bekle = false, int? zamanAsimi = null, double epsilon = 1)
    {
        // Aracın modunu GUIDED yapıp modun güncellenmesini bekle
        if (Mod.ToLower() != "GUIDED".ToLower())
            ModGuncelle("GUIDED", true, zamanAsimi);

        // Araca kalkis komutunu gönder
        ComPort.KonumaGit(konum);

        // Aracın belirtilen konuma ulaşıp ulaşmadığını kontrol eden metot
        bool KonumaUlasildiMi()
        {
            //aracın şuanki konumunu al
            PointLatLngAlt suankiKonum = ComPort.MAV.cs.GpsBilgisi.ToPointLatLngAlt();

            //aradaki mesafeyi hesapla (metre)
            var mesafe = suankiKonum.MesafeGetir(konum);

            //mesafe epsilon'dan küçükse istenen konuma ulaşılmış sayılır
            return mesafe <= epsilon;
        }


        // Aracın belirtilen konuma ulaşmasını bekle
        if (bekle)
        {
            Bekle(KonumaUlasildiMi, zamanAsimi, hataMesaji: "Konuma ulaşma işlemi zaman aşımına uğradı");
        }
    }


    /// <summary>
    /// Aracın 'arm' edilebilir olmasını bekleyen metod. (araç arm edilmez)
    /// </summary>
    /// <param name="zamanAsimi"> Aracın 'arm' edilebilir olmasını beklemek için belirlenen zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    public void ArmEdilebilirOlmasiniBekle(int? zamanAsimi = null)
    {
        // Daha önce tanımlanmış 'Bekle' metodunu kullanarak, belirtilen koşul sağlanana kadar bekler
        Bekle(() => ArmEdilebilirMi, zamanAsimi, "Araç ARM edilmesini bekleme süresi zaman aşımına uğradı.");
    }

    /// <summary>
    /// Bir koşulun gerçekleşmesini belirli bir süre bekleyen metod
    /// </summary>
    /// <param name="kosul"> Gerçekleşmesi beklenen koşul </param>
    /// <param name="zamanAsimi"> Zaman aşımı süresi (saniye). null ise süre sınırı olmadan bekler </param>
    /// <param name="aralik"> Kontrol aralığı (milisaniye) </param>
    /// <param name="hataMesaji"> Zaman aşımı durumunda fırlatılacak hata mesajı </param>
    /// <exception cref="TimeoutException"> Zaman aşımı durumunda fırlatılır </exception>
    public static void Bekle(Func<bool> kosul, int? zamanAsimi = null, string hataMesaji = null, int? aralik = null)
    {
        // Başlangıç zamanını kaydediyoruz
        var baslangicZamani = DateTime.UtcNow;
        while (!kosul())  // Koşul sağlanana kadar döngüyü sürdür
        {
            // Zaman aşımı kontrolü. Eğer belirlenen süre geçtiyse hata fırlat
            if (zamanAsimi != null && (DateTime.UtcNow - baslangicZamani).Seconds >= zamanAsimi)
            {
                throw new TimeoutException(hataMesaji);
            }

            // Belirlenen aralıklarla bekleme işlemi (eğer aralık belirtilmemişse 100 ms bekle)
            Thread.Sleep(aralik ?? 100);
        }
    }

    #endregion

    #region ### private metotlar ###
    private static void NabizGonder()
    {
        var tmp = new MAVLink.MavlinkParse();
        var hb = new MAVLink.mavlink_heartbeat_t
        {
            autopilot = 1,
            base_mode = 2,
            custom_mode = 3,
            mavlink_version = 2,
            system_status = 6,
            type = 7
        };

        tmp.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
        tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
        tmp.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
        tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

        tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb, true);
        tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb, true);
    }

    /// <summary>
    /// Log kaydı oluşturur
    /// </summary>
    /// <param name="comPort"></param>
    private void LogKaydiOlustur(MavLinkArayuzu comPort)
    {
        try
        {
            var logDir = Path.Combine(Directory.GetCurrentDirectory(), "logs");
            Directory.CreateDirectory(logDir);
            lock (this)
            {
                // günlük adları oluştur
                var dt = DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss");
                var tlog = logDir + Path.DirectorySeparatorChar +
                           dt + ".tlog";
                var rlog = logDir + Path.DirectorySeparatorChar +
                           dt + ".rlog";

                // bu günlük adının zaten var olup olmadığını kontrol et
                var a = 1;
                while (File.Exists(tlog))
                {
                    Thread.Sleep(1000);
                    // create new names with a as an index
                    dt = DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss") + "-" + a;
                    tlog = logDir + Path.DirectorySeparatorChar +
                           dt + ".tlog";
                    rlog = logDir + Path.DirectorySeparatorChar +
                           dt + ".rlog";
                }

                //günlükleri yazmak için aç
                comPort.kayitDosyasi =
                    new BufferedStream(
                        File.Open(tlog, FileMode.CreateNew, FileAccess.ReadWrite, FileShare.None));
                comPort.hamKayitDosyasi =
                    new BufferedStream(
                        File.Open(rlog, FileMode.CreateNew, FileAccess.ReadWrite, FileShare.None));
            }
        }
        catch
        {
            // ignored
        }
    }

    /// <summary>
    /// ComPort üzerindeki Mavlink nesnesi değiştiğinde tetiklenen metot
    /// </summary>
    /// <param name="sender"></param>s
    /// <param name="e"></param>
    private void comPort_MavChanged(object sender, EventArgs e)
    {
    }


    /// <summary>
    /// Kumanda kanal baskılamaları değiştiğinde tetiklenen metot
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void KanalBaskilamaGonder(object? sender, EventArgs e)
    {
        // baskılama değerlerini göndermek için bir dizi oluştur
        ushort[] overrides = new ushort[8];

        // her kanal için baskılama değerini al
        for (int i = 0; i < _baskilamalar._count; i++)
        {
            // ilgili kanal için baskılama değeri varsa onu kullan, yoksa 0
            if (_baskilamalar.ContainsKey((i + 1).ToString()))
                overrides[i] = (ushort)(_baskilamalar[(i + 1).ToString()] ?? 0);
            else
                overrides[i] = 0;
        }

        // baskılama değerlerini araca gönder
        ComPort.KumandaBaskilamaGonder((byte)ComPort.SeciliSistemId, (byte)ComPort.SeciliBilesenId, overrides[0], overrides[1], overrides[2], overrides[3],
            overrides[4], overrides[5], overrides[6], overrides[7]);
    }

    /// <summary>
    ///     ana serial Okuyucu
    ///     controls
    ///     serial reading
    ///     link quality stats
    ///     speech voltage - custom - alt warning - data lost
    ///     heartbeat packet sending
    ///     and can't fall out
    /// </summary>
    private void SerialReader()
    {
        if (_serialThread)
            return;
        _serialThread = true;

        _serialThreadrunner.Reset();

        var minbytes = 0;

        var armedstatus = false;

        var linkqualitytime = DateTime.Now;

        while (_serialThread)
            try
            {
                Thread.Sleep(1); // was 5

                // bağlantı kalitesini zaman içinde yenile
                if ((DateTime.Now - ComPort.MAV.lastvalidpacket).TotalSeconds >= 1)
                    if (linkqualitytime.Second != DateTime.Now.Second)
                    {
                        ComPort.MAV.cs.YkiLinkKalitesi = (ushort)(ComPort.MAV.cs.YkiLinkKalitesi * 0.8f);
                        linkqualitytime = DateTime.Now;
                    }

                // veri̇ kaybi uyarisi - en az 10 saniye bekle, bağlantının ilk 30 saniyesini göz ardı et, 5 saniye aralıklarla tekrarla
                if ((DateTime.Now - ComPort.MAV.lastvalidpacket).TotalSeconds > 10
                    && (DateTime.Now - _connecttime).TotalSeconds > 30
                    && (DateTime.Now - _nodatawarning).TotalSeconds > 5
                    && ComPort.BaseStream.IsOpen
                    && ComPort.MAV.cs.Armed)
                {
                }

                // arm durumu değişince home konumunu güncelle
                if (armedstatus != ComPort.MAV.cs.Armed && ComPort.BaseStream.IsOpen)
                {
                    armedstatus = ComPort.MAV.cs.Armed;

                    // durum arm olarak değiştiğinde home konumunu güncelle
                    if (ComPort.MAV.cs.Armed &&
                        ComPort.MAV.apname != MAVLink.MAV_AUTOPILOT.INVALID &&
                        ComPort.MAV.aptype != MAVLink.MAV_TYPE.GIMBAL)
                        ThreadPool.QueueUserWorkItem(state =>
                        {
                            try
                            {
                                ComPort.MAV.cs.HomeLocation = new PointLatLngAlt(ComPort.WpGetir(0));
                            }
                            catch
                            {
                                // ignored
                            }
                        });
                }

                // her saniye yki'den otopilot'a nabiz gönder
                if (_heatbeatSend.Second != DateTime.Now.Second)
                {
                    var htb = new MAVLink.mavlink_heartbeat_t
                    {
                        type = (byte)MAVLink.MAV_TYPE.GCS,
                        autopilot = (byte)MAVLink.MAV_AUTOPILOT.INVALID,
                        mavlink_version = 3 // MAVLink.MAVLINK_VERSION
                    };

                    // her linki kontrol et
                    foreach (var port in Comports.ToArray())
                    {
                        if (!port.BaseStream.IsOpen)
                            continue;

                        // kalp atışı aralığında paramlar için yokla - bu baglantiMetni sadece birincil mavlink bağlantısı ise
                        if (!port.giveComport)
                            try
                            {
                                // arm degilken sorgula
                                if (!port.MAV.cs.Armed)
                                {
                                    port.getParamPoll();
                                    port.getParamPoll();
                                }
                            }
                            catch
                            {
                                // ignored
                            }

                        // gönderebilen 3 tür nabiz var, mavlink1, mavlink2 imzalı ve imzasız
                        var sentsigned = false;
                        var sentmavlink1 = false;
                        var sentmavlink2 = false;

                        // her mav'ı kontrol et
                        foreach (var mav in port.MAVlist)
                            try
                            {
                                // yoksa versiyon için sorgula - her mav her baglantiMetni
                                if (!mav.cs.Armed && DateTime.Now.Second % 20 == 0 &&
                                    mav.cs.Surum < new Version(0, 1))
                                    port.getVersion(mav.SistemId, mav.BilesenId, false);

                                // mavlink2 aygıtı ile bağlantı kurulduysa
                                if (mav.mavlinkv2)
                                {
                                    // imzalama aktif mi?
                                    if (mav.signing)
                                    {
                                        // gönderip göndermediğimizi kontrol et
                                        if (sentsigned)
                                            continue;
                                        sentsigned = true;
                                    }
                                    else
                                    {
                                        // gönderip göndermediğimizi kontrol et
                                        if (sentmavlink2)
                                            continue;
                                        sentmavlink2 = true;
                                    }
                                }
                                else
                                {
                                    // gönderip göndermediğimizi kontrol et
                                    if (sentmavlink1)
                                        continue;
                                    sentmavlink1 = true;
                                }

                                port.paketGonder(htb, mav.SistemId, mav.BilesenId);
                            }
                            catch (Exception)
                            {
                                // kötü baglantiMetni kapat
                                try
                                {
                                    port.Kapat();
                                }
                                catch
                                {
                                    // ignored
                                }
                            }
                    }

                    _heatbeatSend = DateTime.Now;
                }


                // bağlantı arayüzlerini kontrol et
                foreach (var port in Comports.ToArray())
                {
                    if (!port.BaseStream.IsOpen)
                    {
                        // ilk portu atla
                        if (port == ComPort)
                            continue;

                        // diziyi değiştir ve bırakın
                        Comports.Remove(port);
                        port.Dispose();
                        break;
                    }

                    while (port.BaseStream.IsOpen && port.BaseStream.BytesToRead > minbytes &&
                           port.giveComport == false && _serialThread)
                        try
                        {
                            port.paketOku();
                        }
                        catch
                        {
                            // ignored
                        }

                    // baglantiMetni üzerindeki sysid'lerin mevcut durumunu güncelle
                    foreach (var mav in port.MAVlist)
                        try
                        {
                            mav.cs.UpdateCurrentSettings(null, false, port, mav);
                        }
                        catch
                        {
                            // ignored
                        }
                }
            }
            catch (Exception)
            {
                try
                {
                    ComPort.Kapat();
                }
                catch
                {
                    // ignored
                }
            }

        _serialThreadrunner.Set();
    }
    #endregion

}