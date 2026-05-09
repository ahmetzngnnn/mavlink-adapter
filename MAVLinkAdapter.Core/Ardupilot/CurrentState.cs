using System.Net.NetworkInformation;
using System.Reflection;
using System.Runtime.Serialization;
using System.Text.Json.Serialization;
using log4net;
using MAVLinkAdapter.Core.Modeller;
using MAVLinkAdapter.Core.Utilities;

namespace MAVLinkAdapter.Core.Ardupilot;

public partial class CurrentState : ICloneable
{
    private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

    private static PointLatLngAlt _homelocation = new();

    private static PointLatLngAlt _trackerloc = new();

    public static int rateattitudebackup;
    public static int ratepositionbackup;
    public static int ratestatusbackup;
    public static int ratesensorsbackup;
    public static int ratercbackup;

    public static int KIndexstatic = -1;


    private float _ch3percent = -1;
    private float _climbrate;
    private float _crit_aoa = 25;

    private DateTime _lastcurrent = DateTime.MinValue;

    private DateTime _lastcurrent2 = DateTime.MinValue;
    private float _localsnrdb;

    private uint _mode = 99999;

    private PointLatLngAlt _movingbase = new();
    private float _remotesnrdb;

    private float _sonarrange;

    private float _ter_alt;

    private float _ter_curalt;

    private float _yaw;
    internal bool batterymonitoring = false;

    // current firmware
    public Firmwares firmware = Firmwares.ArduCopter2;
    private bool gotwind;

    // HIL
    public int hilch1; // { get; set; }
    public int hilch2; // { get; set; }
    public int hilch3; // { get; set; }
    public int hilch4; // { get; set; }
    public int hilch5;
    public int hilch6;
    public int hilch7;
    public int hilch8;
    internal double imutime;

    private DateTime lastalt = DateTime.MinValue;

    [JsonIgnore][IgnoreDataMember] public int lastautowp = -1;

    private DateTime lastdata = DateTime.MinValue;

    // for calc of sitl speedup
    internal DateTime lastimutime = DateTime.MinValue;
    private PointLatLngAlt lastpos = new();
    private DateTime lastremrssi = DateTime.Now;
    private DateTime lastrssi = DateTime.Now;

    private DateTime lastsecondcounter = DateTime.Now;

    private DateTime lastupdate = DateTime.Now;

    internal bool MONO;
    private float oldalt;

    [JsonIgnore][IgnoreDataMember] public MAVState parent;

    // rc override
    public short rcoverridech1; //{ get; set; }
    public short rcoverridech10; // { get; set; }
    public short rcoverridech11; //{ get; set; }
    public short rcoverridech12; //{ get; set; }
    public short rcoverridech13; // { get; set; }
    public short rcoverridech14; // { get; set; }
    public short rcoverridech15; // { get; set; }
    public short rcoverridech16; // { get; set; }
    public short rcoverridech17; // { get; set; }
    public short rcoverridech18; // { get; set; }
    public short rcoverridech2; // { get; set; }
    public short rcoverridech3; //{ get; set; }
    public short rcoverridech4; //{ get; set; }
    public short rcoverridech5; // { get; set; }
    public short rcoverridech6; // { get; set; }
    public short rcoverridech7; // { get; set; }
    public short rcoverridech8; // { get; set; }
    public short rcoverridech9; //{ get; set; }

    public Mavlink_Sensors sensors_enabled = new();
    public Mavlink_Sensors sensors_health = new();
    public Mavlink_Sensors sensors_present = new();

    private bool useLocation;

    /// <summary>
    ///     used for wind calc
    /// </summary>
    private double We_fgo;

    /// <summary>
    ///     used in wind calc
    /// </summary>
    private double Wn_fgo;


    static CurrentState()
    {
        // set default telemrates
        rateattitudebackup = 4;
        ratepositionbackup = 2;
        ratestatusbackup = 2;
        ratesensorsbackup = 2;
        ratercbackup = 2;
    }

    public CurrentState()
    {
        ResetInternals();

        var t = Type.GetType("Mono.Runtime");
        MONO = t != null;
    }

    public RpmBilgisi rpmBilgisi { get; set; }
    
    public DurumBilgisi Durum { get; private set; }

    public HizBilgisi Hiz { get; set; }

    // Ev İrtifa Ofseti (mesafe)
    public float altoffsethome { get; set; }

    public GpsBilgisi GpsBilgisi { get; set; }

    public GpsBilgisi GpsBilgisi2 { get; set; }

    public DateTime GpsZamani { get; set; }

    // Hava Hızı (m/s)
    public float HavaHizi { get; set; }

    // Hedef Hava Hızı (m/s)
    public float HedefHavaHizi { get; private set; }

    // Hava hızı Oranı
    public float asratio { get; set; }

    // Yer Hızı (m/s)
    public float YerHizi { get; set; }

    //IMU
    public ImuBilgisi imu1 { get; set; }
    public ImuBilgisi imu2 { get; set; }
    public ImuBilgisi imu3 { get; set; }

    //radio

    public KumandaBilgisi Kumanda { get; set; }

    // motors

    public MotorBilgisi motorpwm { get; set; }

    public EscBilgisi esc1 { get; set; }
    public EscBilgisi esc2 { get; set; }
    public EscBilgisi esc3 { get; set; }
    public EscBilgisi esc4 { get; set; }
    public EscBilgisi esc5 { get; set; }
    public EscBilgisi esc6 { get; set; }
    public EscBilgisi esc7 { get; set; }
    public EscBilgisi esc8 { get; set; }


    // Failsafe Durumu
    public bool FailSafe { get; set; }

    public NavigasyonKontrolBilgisi NavKontrol { get; set; }

    // Mod (Auto, RTL, vs)
    public string Mod { get; private set; }

    // Dikey yönde Tırmanma Hızı (m/s)
    public float TirmanmaHizi { get; set; }


    // Gidilen Mesafe (m)
    public float GidilenMesafe { get; set; }

    // Havadaki Zaman (saniye)    
    public float HavadakiZaman { get; set; }


    // Dönüş Hızı
    public float DonusHizi
    {
        get
        {
            if (YerHizi <= 1) return 0;
            return (float)(Durum.Roll * 9.80665 / YerHizi);
        }
    }


    // Dönüş Yarıçapı
    public float DonusYaricapi
    {
        get
        {
            if (YerHizi <= 1) return 0;
            return (float)(YerHizi * YerHizi / (9.80665 * Math.Tan(Durum.Roll * MathHelper.deg2rad)));
        }
    }

    // Rüzgar Yönü (Deg)
    public float RuzgarYonu { get; set; }

    // Rüzgar Hızı (m/s)
    public float RuzgarHizi { get; set; }

    //message
    public List<string> messages { get; set; }

    //Yüksek Öncelikli Mesaj
    public string messageHigh { get; set; } = "";

    //Yüksek Öncelikli Mesaj Zamanı
    public DateTime messageHighTime { get; set; }

    //battery

    public BataryaBilgisi Batarya { get; set; }
    public BataryaBilgisi batarya2 { get; set; }

    public double HomeAlt
    {
        get => HomeLocation.Alt;
        set { }
    }

    public PointLatLngAlt HomeLocation
    {
        get => _homelocation;
        set => _homelocation = value;
    }


    public PointLatLngAlt TrackerLocation
    {
        get
        {
            if (_trackerloc.Lng != 0) return _trackerloc;
            return HomeLocation;
        }
        set => _trackerloc = value;
    }

    public PointLatLngAlt Konum => GpsBilgisi.ToPointLatLngAlt();

    public PointLatLngAlt TargetLocation { get; set; } = PointLatLngAlt.Zero;

    // Eve olan Mesafe (m)
    public float EveOlanUzaklik
    {
        get
        {
            if ((GpsBilgisi.Enlem == 0 && GpsBilgisi.Boylam == 0) || TrackerLocation.Lat == 0)
                return 0;

            // shrinking factor for longitude going to poles direction
            var rads = Math.Abs(TrackerLocation.Lat) * 0.0174532925;
            var scaleLongDown = Math.Cos(rads);
            var scaleLongUp = 1.0f / Math.Cos(rads);

            //DST to Home
            var dstlat = Math.Abs(TrackerLocation.Lat - GpsBilgisi.Enlem) * 111319.5;
            var dstlon = Math.Abs(TrackerLocation.Lng - GpsBilgisi.Boylam) * 111319.5 * scaleLongDown;
            return (float)Math.Sqrt(dstlat * dstlat + dstlon * dstlon);
        }
    }


    public SonarBilgisi Sonar { get; set; }


    public HafizaBilgisi Hafiza { get; set; }

    // Sistem Yükü
    public float SysLoad { get; set; }

    //Arm edilme durumu
    public bool Armed { get; private set; }

    // stats
    public ushort KaybedilenPaketOrani { get; set; }
    public ushort YkiLinkKalitesi { get; set; }

    // Donanım voltajı (volt)
    public float hwvoltage { get; set; }

    // otopilot voltajı (volt)
    public float boardvoltage { get; set; }

    // Servo voltajı (volt)
    public float servovoltage { get; set; }

    // Voltaj bayrağı
    public uint voltageflag { get; set; }

    // I2C hataları
    public ushort i2cerrors { get; set; }


    // pressure
    public BarometreBilgisi barometre { get; set; }
    public BarometreBilgisi barometre2 { get; set; }

    // sensor offsets
    public SensorOffsets sensor_offsets { get; set; }

    // requested stream rates
    public int rateattitude { get; set; }
    public int rateposition { get; set; }
    public int ratestatus { get; set; }
    public int ratesensors { get; set; }
    public int raterc { get; set; }

    // reference
    public DateTime datetime { get; set; }

    public float campointa { get; set; }

    public float campointb { get; set; }

    public float campointc { get; set; }


    public bool InisYapildi { get; set; }

    public AraziTakipBilgisi AraziTakip { get; set; }

    public OpticFlowSensorBilgisi OpticFlow { get; set; }

    public EkfBilgisi Ekf { get; set; }

    public PidTuningBilgisi PidTuning { get; set; }

    public TitresimBilgisi Titresim { get; set; }

    public Version Surum { get; set; }

    public uint capabilities { get; set; }

    public float speedup { get; set; }

    public byte vtol_state { get; private set; }
    public byte landed_state { get; private set; }

    // Hava Sıcaklığı (derece) (Hava hızı sensöründe gelen deger)
    public byte HavaSicakligi { get; set; }

    // Otopilot Sıcaklığı (derece)
    public byte OtopilotSicakligi { get; set; }


    public event EventHandler csCallBack;

    #region ### public metotlar ###
    public object Clone()
    {
        return MemberwiseClone();
    }

    /// <summary>
    /// Tüm iç verileri sıfırlar
    /// </summary>
    public void ResetInternals()
    {
        lock (this)
        {
            Mod = "Unknown";
            _mode = 99999;
            messages = new List<string>();
            useLocation = false;
            rateattitude = rateattitudebackup;
            rateposition = ratepositionbackup;
            ratestatus = ratestatusbackup;
            ratesensors = ratesensorsbackup;
            raterc = ratercbackup;
            datetime = DateTime.MinValue;

            Durum = new DurumBilgisi();
            Batarya = new BataryaBilgisi();
            batarya2 = new BataryaBilgisi();
            Ekf = new EkfBilgisi();
            AraziTakip = new AraziTakipBilgisi();
            OpticFlow = new OpticFlowSensorBilgisi();
            PidTuning = new PidTuningBilgisi();
            Titresim = new TitresimBilgisi();
            rpmBilgisi = new RpmBilgisi();
            Hiz = new HizBilgisi();
            GpsBilgisi = new GpsBilgisi();
            GpsBilgisi2 = new GpsBilgisi();
            barometre = new BarometreBilgisi();
            barometre2 = new BarometreBilgisi();
            imu1 = new ImuBilgisi();
            imu2 = new ImuBilgisi();
            imu3 = new ImuBilgisi();

            Kumanda = new KumandaBilgisi();

            motorpwm = new MotorBilgisi();
            esc1 = new EscBilgisi();
            esc2 = new EscBilgisi();
            esc3 = new EscBilgisi();
            esc4 = new EscBilgisi();
            esc5 = new EscBilgisi();
            esc6 = new EscBilgisi();
            esc7 = new EscBilgisi();
            esc8 = new EscBilgisi();
            Sonar = new SonarBilgisi();
            NavKontrol = new NavigasyonKontrolBilgisi();
            Hafiza = new HafizaBilgisi();
            sensor_offsets = new SensorOffsets();

            Batarya.KullanilanMah = 0;
            _lastcurrent = DateTime.MinValue;
            GidilenMesafe = 0;
            HavadakiZaman = 0;
            Surum = new Version();
            voltageflag = (uint)MAVLink.MAV_POWER_STATUS.USB_CONNECTED;
            capabilities = (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT;
        }
    }
    
    /// <summary>
    ///     use for main serial port only
    /// </summary>
    /// <param name="bs"></param>
    public void UpdateCurrentSettings(Action<CurrentState> bs)
    {
        UpdateCurrentSettings(bs, false, parent.parent, parent.parent.MAV);
    }

    /// <summary>
    ///     Use the default sysid
    /// </summary>
    /// <param name="bs"></param>
    /// <param name="updatenow"></param>
    /// <param name="mavinterface"></param>
    public void UpdateCurrentSettings(Action<CurrentState> bs, bool updatenow,
        MavLinkArayuzu mavinterface)
    {
        UpdateCurrentSettings(bs, updatenow, mavinterface, mavinterface.MAV);
    }

    public void UpdateCurrentSettings(Action<CurrentState> bs, bool updatenow,
        MavLinkArayuzu mavinterface, MAVState MAV)
    {
        lock (this)
        {
            if (DateTime.Now > lastupdate.AddMilliseconds(50) || updatenow) // 20 hz
            {
                lastupdate = DateTime.Now;

                ProcessLinkQuality(MAV);

                ProcessUpdatingCounter(mavinterface);

                ProcessMavReRequestAllMsgs(mavinterface, MAV);

                ProcessMavMsgRcChannelsScaled(MAV);

                ProcessMavMsgLocalPosNed(MAV);

                ProcessMavMsgPosTarget(MAV);

                ProcessMavMsgOtoPilotVersion(MAV);

                ProcessMavMsgFenceStatus(MAV);

                ProcessMavMsgHighLatency(MAV);

                ProcessMavMsgHilCtrls(MAV);

                ProcessMavMsgOptFlow(MAV);

                ProcessMavMsgMntStatus(MAV);

                ProcessMavMsgVibration(MAV);

                ProcessMavMsgASAC(MAV);

                ProcessMavMsgSysTime(MAV);

                ProcessMavMsgHWStatus(MAV);

                ProcessMavMsgEKFStatus(MAV);

                ProcessMavMsgRngFinder(MAV);

                ProcessMavMsgPwrStat(MAV);

                ProcessMavMsgWind(MAV);

                ProcessMavMsgHb(MAV);

                ProcessMavMsgSysStatus(MAV);

                ProcessMavMsgExtSysSt(MAV);

                ProcessMavMsgBatt2(MAV);

                ProcessMavMsgBattStat(MAV);

                ProcessMavMsgSclPres(MAV);

                ProcessMavMsgSclPres2(MAV);

                ProcessMavMsgTerRep(MAV);

                ProcessMavMsgSensOff(MAV);

                ProcessMavMsgAttitude(MAV);

                ProcessMavMsgPosition(MAV);

                ProcessMavMsgGpsRaw(MAV);

                ProcessMavMsgGpsRaw2(MAV);

                ProcessMavMsgGpsStat(MAV);

                ProcessMavMsgRadio(MAV);

                ProcessMavMsgRadioStat(MAV);

                ProcessMavMsgMissionCurrent(MAV);

                ProcessMavMsgNavCtrlOut(MAV);

                ProcessMavMsgRpm(MAV);

                ProcessMavMsgRcChannelsRaw(MAV);

                ProcessMavMsgRcChannels(MAV);

                ProcessMavMsgEscTelem1to4(MAV);

                ProcessMavMsgEscTelem5to8(MAV);

                ProcessMavMsgServoOutRaw(MAV);

                ProcessMavMsgRawImu(MAV);

                ProcessMavMsgSclImu(MAV);

                ProcessMavMsgSclImu2(MAV);

                ProcessMavMsgSclImu3(MAV);

                ProcessMavMsgPidTuning(MAV);

                ProcessMavMsgVfrHud(MAV);

                ProcessMavMsgMemInfo(MAV);

                ProcessMavMsgAoaSsa(MAV);
            }

            try
            {
                if (csCallBack != null)
                    csCallBack(this, null);
            }
            catch
            {
            }

            //Console.Write(DateTime.Now.Millisecond + " start ");

            // update form
            try
            {
                if (bs != null) bs?.Invoke(this);
            }
            catch
            {
            }
        }
    }

    public void UpdateCurrentSettings()
    {
        UpdateCurrentSettings(false, parent.parent, parent.parent.MAV);
    }

    public void UpdateCurrentSettings(bool updatenow,
        MavLinkArayuzu mavinterface, MAVState MAV)
    {
        lock (this)
        {
            if (DateTime.Now > lastupdate.AddMilliseconds(50) || updatenow) // 20 hz
            {
                lastupdate = DateTime.Now;

                ProcessLinkQuality(MAV);

                ProcessUpdatingCounter(mavinterface);

                ProcessMavReRequestAllMsgs(mavinterface, MAV);

                ProcessMavMsgRcChannelsScaled(MAV);

                ProcessMavMsgLocalPosNed(MAV);

                ProcessMavMsgPosTarget(MAV);

                ProcessMavMsgOtoPilotVersion(MAV);

                ProcessMavMsgFenceStatus(MAV);

                ProcessMavMsgHighLatency(MAV);

                ProcessMavMsgHilCtrls(MAV);

                ProcessMavMsgOptFlow(MAV);

                ProcessMavMsgMntStatus(MAV);

                ProcessMavMsgVibration(MAV);

                ProcessMavMsgASAC(MAV);

                ProcessMavMsgSysTime(MAV);

                ProcessMavMsgHWStatus(MAV);

                ProcessMavMsgEKFStatus(MAV);

                ProcessMavMsgRngFinder(MAV);

                ProcessMavMsgPwrStat(MAV);

                ProcessMavMsgWind(MAV);

                ProcessMavMsgHb(MAV);

                ProcessMavMsgSysStatus(MAV);

                ProcessMavMsgExtSysSt(MAV);

                ProcessMavMsgBatt2(MAV);

                ProcessMavMsgBattStat(MAV);

                ProcessMavMsgSclPres(MAV);

                ProcessMavMsgSclPres2(MAV);

                ProcessMavMsgTerRep(MAV);

                ProcessMavMsgSensOff(MAV);

                ProcessMavMsgAttitude(MAV);

                ProcessMavMsgPosition(MAV);

                ProcessMavMsgGpsRaw(MAV);

                ProcessMavMsgGpsRaw2(MAV);

                ProcessMavMsgGpsStat(MAV);

                ProcessMavMsgRadio(MAV);

                ProcessMavMsgRadioStat(MAV);

                ProcessMavMsgMissionCurrent(MAV);

                ProcessMavMsgNavCtrlOut(MAV);

                ProcessMavMsgRpm(MAV);

                ProcessMavMsgRcChannelsRaw(MAV);

                ProcessMavMsgRcChannels(MAV);

                ProcessMavMsgEscTelem1to4(MAV);

                ProcessMavMsgEscTelem5to8(MAV);

                ProcessMavMsgServoOutRaw(MAV);

                ProcessMavMsgRawImu(MAV);

                ProcessMavMsgSclImu(MAV);

                ProcessMavMsgSclImu2(MAV);

                ProcessMavMsgSclImu3(MAV);

                ProcessMavMsgPidTuning(MAV);

                ProcessMavMsgVfrHud(MAV);

                ProcessMavMsgMemInfo(MAV);

                ProcessMavMsgAoaSsa(MAV);
            }

            try
            {
                if (csCallBack != null)
                    csCallBack(this, null);
            }
            catch
            {
            }
        }
    }
    
    #endregion


    #region ### private metotlar ###

    private void ProcessUpdatingCounter(MavLinkArayuzu mavinterface)
    {
        if (datetime.Second != lastsecondcounter.Second)
        {
            lastsecondcounter = datetime;

            if (lastpos.Lat != 0 && lastpos.Lng != 0 && Armed)
            {
                if (mavinterface.BaseStream != null && !mavinterface.BaseStream.IsOpen && !mavinterface.logreadmode)
                    GidilenMesafe = 0;

                GidilenMesafe += (float)lastpos.MesafeGetir(new PointLatLngAlt(GpsBilgisi.Enlem, GpsBilgisi.Boylam, 0, ""));
                lastpos = new PointLatLngAlt(GpsBilgisi.Enlem, GpsBilgisi.Boylam, 0, "");
            }
            else
            {
                lastpos = new PointLatLngAlt(GpsBilgisi.Enlem, GpsBilgisi.Boylam, 0, "");
            }

            // throttle is up, or groundspeed is > 3 m/s
            if ((Kumanda.ch3in > 12 || YerHizi > 3.0) && Armed)
            {
                HavadakiZaman++;
            }
        }
    }

    private void ProcessLinkQuality(MAVState stateMav)
    {
        //check if valid mavinterface
        if (parent != null && parent.packetsnotlost != 0)
        {
            if ((DateTime.Now - stateMav.lastvalidpacket).TotalSeconds > 10)
                YkiLinkKalitesi = 0;
            else
                YkiLinkKalitesi =
                    (ushort)(parent.packetsnotlost / (parent.packetsnotlost + parent.packetslost) * 100.0);

            if (YkiLinkKalitesi > 100)
                YkiLinkKalitesi = 100;
        }
    }

    private void ProcessMavReRequestAllMsgs(MavLinkArayuzu mavinterface, MAVState stateMav)
    {
        // re-request streams
        if (!(lastdata.AddSeconds(8) > DateTime.Now) && mavinterface.BaseStream != null &&
            mavinterface.BaseStream.IsOpen)
        {
            try
            {
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS, stateMav.cs.ratestatus,
                    stateMav.SistemId, stateMav.BilesenId); // mode
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.POSITION, stateMav.cs.rateposition,
                    stateMav.SistemId, stateMav.BilesenId); // request gps
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.EXTRA1, stateMav.cs.rateattitude,
                    stateMav.SistemId, stateMav.BilesenId); // request attitude
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.EXTRA2, stateMav.cs.rateattitude,
                    stateMav.SistemId, stateMav.BilesenId); // request vfr
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.EXTRA3, stateMav.cs.ratesensors, stateMav.SistemId,
                    stateMav.BilesenId); // request extra stuff - tridge
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.RAW_SENSORS, stateMav.cs.ratesensors,
                    stateMav.SistemId, stateMav.BilesenId); // request raw sensor
                mavinterface.VeriAkisiIste(MAVLink.MAV_DATA_STREAM.RC_CHANNELS, stateMav.cs.raterc, stateMav.SistemId,
                    stateMav.BilesenId); // request rc info
            }
            catch
            {
                // istekte bulunurken hata oluşursa
            }

            lastdata = DateTime.Now.AddSeconds(30); // prevent flooding
        }
    }

    private void ProcessMavMsgAoaSsa(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.AOA_SSA);
        if (mavLinkMessage != null)
        {
            var aoa_ssa = mavLinkMessage.ToStructure<MAVLink.mavlink_aoa_ssa_t>();
            this.Durum.UpdateFromAoaSsaMsg(aoa_ssa);

        }
    }

    private void ProcessMavMsgMemInfo(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.MEMINFO);
        if (mavLinkMessage != null)
        {
            var mem = mavLinkMessage.ToStructure<MAVLink.mavlink_meminfo_t>();
            Hafiza.UpdateFromMemInfoMsg(mem);
        }
    }

    private void ProcessMavMsgVfrHud(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.VFR_HUD);
        if (mavLinkMessage != null)
        {
            var vfr = mavLinkMessage.ToStructure<MAVLink.mavlink_vfr_hud_t>();

            YerHizi = vfr.groundspeed;

            HavaHizi = vfr.airspeed;

            Kumanda.ch3in = vfr.throttle;

            if (sensors_present.revthrottle && sensors_enabled.revthrottle && sensors_health.revthrottle)
                if (Kumanda.ch3in > 0)
                    Kumanda.ch3in *= -1;
        }
    }

    private void ProcessMavMsgPidTuning(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.PID_TUNING);
        if (mavLinkMessage != null)
        {
            var pid = mavLinkMessage.ToStructure<MAVLink.mavlink_pid_tuning_t>();

            PidTuning = new PidTuningBilgisi();
            PidTuning.FF = pid.FF;
            PidTuning.P = pid.P;
            PidTuning.I = pid.I;
            PidTuning.D = pid.D;
            PidTuning.Eksenler = pid.axis;
            PidTuning.IstenenOran = pid.desired;
            PidTuning.EldeEdilmeOrani = pid.achieved;
        }
    }

    private void ProcessMavMsgSclImu3(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SCALED_IMU3);
        if (mavLinkMessage != null)
        {
            var imu3 = mavLinkMessage.ToStructure<MAVLink.mavlink_scaled_imu3_t>();

            this.imu3.UpdateFromScaledImu3Msg(imu3);
        }
    }

    private void ProcessMavMsgSclImu2(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SCALED_IMU2);
        if (mavLinkMessage != null)
        {
            var imu2 = mavLinkMessage.ToStructure<MAVLink.mavlink_scaled_imu2_t>();

            this.imu2.UpdateFromScaledImu2Msg(imu2);
        }
    }

    private void ProcessMavMsgSclImu(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SCALED_IMU);
        if (mavLinkMessage != null)
        {
            var imu = mavLinkMessage.ToStructure<MAVLink.mavlink_scaled_imu_t>();

            imu1.UpdateFromScaledImuMsg(imu);

            //MAVLink.packets[(byte)MAVLink.MSG_NAMES.RAW_IMU);
        }
    }

    private void ProcessMavMsgRawImu(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RAW_IMU);
        if (mavLinkMessage != null)
        {
            var imu = mavLinkMessage.ToStructure<MAVLink.mavlink_raw_imu_t>();

            imu1.UpdateFromRawImuMsg(imu);

            var timesec = imu.time_usec * 1.0e-6;

            var deltawall = (DateTime.Now - lastimutime).TotalSeconds;

            var deltaimu = timesec - imutime;

            if (deltaimu > 0 && deltaimu < 10)
            {
                speedup = (float)(speedup * 0.95 + deltaimu / deltawall * 0.05);

                imutime = timesec;
                lastimutime = DateTime.Now;
            }
        }
    }

    private void ProcessMavMsgServoOutRaw(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW);
        if (mavLinkMessage != null)
        {
            var servoout = mavLinkMessage.ToStructure<MAVLink.mavlink_servo_output_raw_t>();

            motorpwm.UpdateFromServoOutRawMsg(servoout);

            stateMav.clearPacket((uint)MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW);
        }
    }

    private void ProcessMavMsgEscTelem5to8(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_5_TO_8);
        if (mavLinkMessage != null)
        {
            var esc = mavLinkMessage.ToStructure<MAVLink.mavlink_esc_telemetry_5_to_8_t>();

            esc5.Volt = esc.voltage[0] / 100.0f;
            esc5.Akim = esc.current[0] / 100.0f;
            esc5.Rpm = esc.rpm[0];
            esc5.Sicaklik = esc.temperature[0];

            esc6.Volt = esc.voltage[1] / 100.0f;
            esc6.Akim = esc.current[1] / 100.0f;
            esc6.Rpm = esc.rpm[1];
            esc6.Sicaklik = esc.temperature[1];

            esc7.Volt = esc.voltage[2] / 100.0f;
            esc7.Akim = esc.current[2] / 100.0f;
            esc7.Rpm = esc.rpm[2];
            esc7.Sicaklik = esc.temperature[2];

            esc8.Volt = esc.voltage[3] / 100.0f;
            esc8.Akim = esc.current[3] / 100.0f;
            esc8.Rpm = esc.rpm[3];
            esc8.Sicaklik = esc.temperature[3];
        }
    }

    private void ProcessMavMsgEscTelem1to4(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_1_TO_4);
        if (mavLinkMessage != null)
        {
            var esc = mavLinkMessage.ToStructure<MAVLink.mavlink_esc_telemetry_1_to_4_t>();
            esc1.Volt = esc.voltage[0] / 100.0f;
            esc1.Akim = esc.current[0] / 100.0f;
            esc1.Rpm = esc.rpm[0];
            esc1.Sicaklik = esc.temperature[0];

            esc2.Volt = esc.voltage[1] / 100.0f;
            esc2.Akim = esc.current[1] / 100.0f;
            esc2.Rpm = esc.rpm[1];
            esc2.Sicaklik = esc.temperature[1];

            esc3.Volt = esc.voltage[2] / 100.0f;
            esc3.Akim = esc.current[2] / 100.0f;
            esc3.Rpm = esc.rpm[2];
            esc3.Sicaklik = esc.temperature[2];

            esc4.Volt = esc.voltage[3] / 100.0f;
            esc4.Akim = esc.current[3] / 100.0f;
            esc4.Rpm = esc.rpm[3];
            esc4.Sicaklik = esc.temperature[3];
        }
    }

    private void ProcessMavMsgRcChannels(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS);
        if (mavLinkMessage != null)
        {
            var rcin = mavLinkMessage.ToStructure<MAVLink.mavlink_rc_channels_t>();

            Kumanda.UpdateFromRcChannelsMsg(rcin);
        }
    }

    private void ProcessMavMsgRcChannelsRaw(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW);
        if (mavLinkMessage != null)
        {
            var rcin = mavLinkMessage.ToStructure<MAVLink.mavlink_rc_channels_raw_t>();

            Kumanda.UpdateFromRcChannelsRawMsg(rcin);
        }
    }

    private void ProcessMavMsgRpm(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RPM);

        if (mavLinkMessage != null)
        {
            var rpm = mavLinkMessage.ToStructure<MAVLink.mavlink_rpm_t>();

            this.rpmBilgisi.UpdateFromRpmMsg(rpm);
        }
    }

    private void ProcessMavMsgNavCtrlOut(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.NAV_CONTROLLER_OUTPUT);

        if (mavLinkMessage != null)
        {
            var nav = mavLinkMessage.ToStructure<MAVLink.mavlink_nav_controller_output_t>();

            this.NavKontrol.UpdateFromNavCtrlMsg(nav);

        }
    }

    private void ProcessMavMsgMissionCurrent(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.MISSION_CURRENT);
        if (mavLinkMessage != null)
        {
            var wpcur = mavLinkMessage.ToStructure<MAVLink.mavlink_mission_current_t>();

            NavKontrol.UpdateFromMisssionCurrentMsg(wpcur);

        }
    }

    private void ProcessMavMsgRadioStat(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RADIO_STATUS);
        if (mavLinkMessage != null)
        {
            var radio = mavLinkMessage.ToStructure<MAVLink.mavlink_radio_status_t>();
            Kumanda.UpdateFromRadioStatMsg(radio);
        }
    }

    private void ProcessMavMsgRadio(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RADIO);
        if (mavLinkMessage != null)
        {
            var radio = mavLinkMessage.ToStructure<MAVLink.mavlink_radio_t>();
            Kumanda.UpdateFromRadioMsg(radio);
        }
    }

    private void ProcessMavMsgGpsStat(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.GPS_STATUS);
        if (mavLinkMessage != null)
        {
            var gps = mavLinkMessage.ToStructure<MAVLink.mavlink_gps_status_t>();
            GpsBilgisi.UpdateFromGpsStatMsg(gps);
        }
    }

    private void ProcessMavMsgGpsRaw2(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.GPS2_RAW);
        if (mavLinkMessage != null)
        {
            var gps = mavLinkMessage.ToStructure<MAVLink.mavlink_gps2_raw_t>();

            GpsBilgisi.UpdateFromGpsRawMsg(gps);
        }
    }

    private void ProcessMavMsgGpsRaw(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT);
        if (mavLinkMessage != null)
        {
            var gps = mavLinkMessage.ToStructure<MAVLink.mavlink_gps_raw_int_t>();
            GpsBilgisi.UpdateFromGpsRawMsg(gps, useLocation, mavLinkMessage.ismavlink2);
        }
    }

    private void ProcessMavMsgPosition(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT);
        if (mavLinkMessage != null)
        {
            var loc = mavLinkMessage.ToStructure<MAVLink.mavlink_global_position_int_t>();
            GpsBilgisi.UpdateFromPositionMsg(loc, out useLocation);
            Hiz.UpdateFromPositionMsg(loc);
        }
    }

    private void ProcessMavMsgAttitude(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE);

        if (mavLinkMessage != null)
        {
            var att = mavLinkMessage.ToStructure<MAVLink.mavlink_attitude_t>();
            this.Durum.UpdateFromAttitudeMsg(att);
        }
    }

    private void ProcessMavMsgSensOff(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SENSOR_OFFSETS);
        if (mavLinkMessage != null)
        {
            var sensofs = mavLinkMessage.ToStructure<MAVLink.mavlink_sensor_offsets_t>();

            sensor_offsets.UpdateFromSensorOffsetsMsg(sensofs);
        }
    }

    private void ProcessMavMsgTerRep(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.TERRAIN_REPORT);
        if (mavLinkMessage != null)
        {
            var terrainrep = mavLinkMessage.ToStructure<MAVLink.mavlink_terrain_report_t>();
            this.AraziTakip.UpdateFromTerrainReportMsg(terrainrep);
        }
    }

    private void ProcessMavMsgSclPres2(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SCALED_PRESSURE2);
        if (mavLinkMessage != null)
        {
            var pres = mavLinkMessage.ToStructure<MAVLink.mavlink_scaled_pressure2_t>();
            barometre2.UpdateFromScaledPressure2Msg(pres);
        }
    }

    private void ProcessMavMsgSclPres(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SCALED_PRESSURE);
        if (mavLinkMessage != null)
        {
            var pres = mavLinkMessage.ToStructure<MAVLink.mavlink_scaled_pressure_t>();
            barometre2.UpdateFromScaledPressureMsg(pres);
        }
    }

    private void ProcessMavMsgBattStat(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.BATTERY_STATUS);
        if (mavLinkMessage != null)
        {
            var bats = mavLinkMessage.ToStructure<MAVLink.mavlink_battery_status_t>();

            if (bats.id == 0)
            {
                if (bats.voltages[0] != ushort.MaxValue)
                {
                    Batarya.Hucre1 = bats.voltages[0] / 1000.0;
                    if (bats.voltages[1] != ushort.MaxValue) Batarya.Hucre2 = bats.voltages[1] / 1000.0;
                    else Batarya.Hucre2 = 0.0;
                    if (bats.voltages[2] != ushort.MaxValue) Batarya.Hucre3 = bats.voltages[2] / 1000.0;
                    else Batarya.Hucre3 = 0.0;
                    if (bats.voltages[3] != ushort.MaxValue) Batarya.Hucre4 = bats.voltages[3] / 1000.0;
                    else Batarya.Hucre4 = 0.0;
                    if (bats.voltages[4] != ushort.MaxValue) Batarya.Hucre5 = bats.voltages[4] / 1000.0;
                    else Batarya.Hucre5 = 0.0;
                    if (bats.voltages[5] != ushort.MaxValue) Batarya.Hucre6 = bats.voltages[5] / 1000.0;
                    else Batarya.Hucre6 = 0.0;
                }

                Batarya.KullanilanMah = bats.current_consumed;
                Batarya.Kalan = bats.battery_remaining;
                Batarya.Akim = bats.current_battery / 100.0f;
                if (bats.temperature != short.MaxValue)
                    Batarya.Sicaklik = bats.temperature / 100.0;
            }
            else if (bats.id == 1)
            {
                batarya2.Akim = bats.current_battery / 100.0f;
            }
        }
    }

    private void ProcessMavMsgBatt2(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.BATTERY2);
        if (mavLinkMessage != null)
        {
            var bat = mavLinkMessage.ToStructure<MAVLink.mavlink_battery2_t>();
            batarya2.UpdatefromBattMsg(bat);
        }
    }

    private void ProcessMavMsgExtSysSt(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.EXTENDED_SYS_STATE);
        if (mavLinkMessage != null)
        {
            var extsysstatus = mavLinkMessage.ToStructure<MAVLink.mavlink_extended_sys_state_t>();

            vtol_state = extsysstatus.vtol_state;
            landed_state = extsysstatus.landed_state;
        }
    }

    private void ProcessMavMsgSysStatus(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SYS_STATUS);
        if (mavLinkMessage != null)
        {
            var sysstatus = mavLinkMessage.ToStructure<MAVLink.mavlink_sys_status_t>();

            SysLoad = sysstatus.load / 10.0f;

            Batarya.UpdatefromSysStatusMsg(sysstatus);

            KaybedilenPaketOrani = sysstatus.drop_rate_comm;

            sensors_enabled.Value = sysstatus.onboard_control_sensors_enabled;
            sensors_health.Value = sysstatus.onboard_control_sensors_health;
            sensors_present.Value = sysstatus.onboard_control_sensors_present;

            AraziTakip.Aktif = sensors_health.terrain && sensors_enabled.terrain && sensors_present.terrain;

            UpdateMsgHighAndTimeFromSysStat();


            stateMav.clearPacket((uint)MAVLink.MAVLINK_MSG_ID.SYS_STATUS);
        }
    }

    private void UpdateMsgHighAndTimeFromSysStat()
    {
        if (!sensors_health.gps && sensors_enabled.gps && sensors_present.gps)
        {
            messageHigh = "Kötü GPS Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.gyro && sensors_enabled.gyro && sensors_present.gyro)
        {
            messageHigh = "Kötü GYRO verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.accelerometer && sensors_enabled.accelerometer &&
                 sensors_present.accelerometer)
        {
            messageHigh = "Kötü İvmeölçer verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.compass && sensors_enabled.compass && sensors_present.compass)
        {
            messageHigh = "Kötü Pusula Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.barometer && sensors_enabled.barometer && sensors_present.barometer)
        {
            messageHigh = "Kötü Barometre Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.LASER_POSITION && sensors_enabled.LASER_POSITION &&
                 sensors_present.LASER_POSITION)
        {
            messageHigh = "Kötü Lidar Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.optical_flow && sensors_enabled.optical_flow &&
                 sensors_present.optical_flow)
        {
            messageHigh = "Kötü Optik Akış Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.VISION_POSITION && sensors_enabled.VISION_POSITION &&
                 sensors_present.VISION_POSITION)
        {
            messageHigh = "Kötü Görüntü Pozisyonu Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.terrain && sensors_enabled.terrain && sensors_present.terrain)
        {
            messageHigh = "Kötü Yer Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.geofence && sensors_enabled.geofence &&
                 sensors_present.geofence)
        {
            messageHigh = "Kötü Geofence Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.ahrs && sensors_enabled.ahrs && sensors_present.ahrs)
        {
            messageHigh = "Kötü AHRS Verisi";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.rc_receiver && sensors_enabled.rc_receiver &&
                 sensors_present.rc_receiver)
        {
            messageHigh = "RC Alıcı Verisi Yok";
            messageHighTime = DateTime.Now;
        }
        else if (!sensors_health.logging && sensors_enabled.logging && sensors_present.logging)
        {
            messageHigh = "Kötü Log Verisi";
            messageHighTime = DateTime.Now;
        }
    }

    private void ProcessMavMsgHb(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT);
        if (mavLinkMessage != null)
        {
            var hb = mavLinkMessage.ToStructure<MAVLink.mavlink_heartbeat_t>();

            if (hb.type == (byte)MAVLink.MAV_TYPE.GCS)
            {
                // yki nabiz verisi
                // sadece kayıt oynatımında olur - ve onları burada almamalısınız
            }
            else
            {
                Armed = (hb.base_mode & (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED) ==
                        (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED;

                // safety switch
                if (Armed && sensors_enabled.motor_control == false && sensors_enabled.seen)
                {
                    messageHigh = "(SAFE)";
                    messageHighTime = DateTime.Now;
                }

                // for future use
                InisYapildi = hb.system_status == (byte)MAVLink.MAV_STATE.STANDBY;

                FailSafe = hb.system_status == (byte)MAVLink.MAV_STATE.CRITICAL;

                if ((hb.base_mode & (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED) != 0)
                    if (_mode != hb.custom_mode)
                    {
                        var modelist = Common.getModesList(firmware);

                        if (modelist != null)
                        {
                            var found = false;

                            foreach (var pair in modelist)
                                if (pair.Key == hb.custom_mode)
                                {
                                    Mod = pair.Value;
                                    _mode = hb.custom_mode;
                                    found = true;
                                    break;
                                }

                            if (!found)
                            {
                                Mod = "Bilinmeyen Mod";
                            }
                        }

                        _mode = hb.custom_mode;
                    }
            }
        }
    }

    private void ProcessMavMsgWind(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.WIND);
        if (mavLinkMessage != null)
        {
            var wind = mavLinkMessage.ToStructure<MAVLink.mavlink_wind_t>();

            gotwind = true;

            RuzgarYonu = (wind.direction + 360) % 360;
            RuzgarHizi = wind.speed;
        }
    }

    private void ProcessMavMsgPwrStat(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.POWER_STATUS);
        if (mavLinkMessage != null)
        {
            var power = mavLinkMessage.ToStructure<MAVLink.mavlink_power_status_t>();

            boardvoltage = power.Vcc;
            servovoltage = power.Vservo;

            try
            {
                voltageflag = (uint)(MAVLink.MAV_POWER_STATUS)power.flags;
            }
            catch
            {
            }
        }
    }

    private void ProcessMavMsgRngFinder(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RANGEFINDER);
        if (mavLinkMessage != null)
        {
            var sonar = mavLinkMessage.ToStructure<MAVLink.mavlink_rangefinder_t>();
            Sonar.UpdateFromRangefinderMsg(sonar);
        }
    }

    private void ProcessMavMsgEKFStatus(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.EKF_STATUS_REPORT);
        if (mavLinkMessage != null)
        {
            var ekfstatusm = mavLinkMessage.ToStructure<MAVLink.mavlink_ekf_status_report_t>();

            Ekf.UpdateFromEkfStatusReport(ekfstatusm, GpsBilgisi);


            if (Ekf.HizVaryansi >= 1)
            {
                messageHigh = "HATA: Hız Varyansı";
                messageHighTime = DateTime.Now;
            }

            if (Ekf.PusulaVaryansi >= 1)
            {
                messageHigh = "HATA: Pusula Varyansı";
                messageHighTime = DateTime.Now;
            }

            if (Ekf.YatayHizVaryansi >= 1)
            {
                messageHigh = "HATA: Yatay Pozisyon Varyansı";
                messageHighTime = DateTime.Now;
            }

            if (Ekf.DikeyHizVaryansi >= 1)
            {
                messageHigh = "HATA: Dikey Pozisyon Varyansı";
                messageHighTime = DateTime.Now;
            }

            if (Ekf.AraziVaryansi >= 1)
            {
                messageHigh = "HATA: Terrain Varyansı";
                messageHighTime = DateTime.Now;
            }

            for (var a = 1; a <= (int)MAVLink.EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS; a = a << 1)
            {
                var currentbit = ekfstatusm.flags & a;
                if (currentbit == 0)
                {
                    var currentflag =
                        (MAVLink.EKF_STATUS_FLAGS)
                        Enum.Parse(typeof(MAVLink.EKF_STATUS_FLAGS), a.ToString());

                    switch (currentflag)
                    {
                        case MAVLink.EKF_STATUS_FLAGS.EKF_ATTITUDE: // step 1
                        case MAVLink.EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ: // with pos
                            if (GpsBilgisi.Durum > 0) // we have gps and dont have vel_hoz
                                Ekf.Durum = 1;
                            break;
                        case MAVLink.EKF_STATUS_FLAGS.EKF_VELOCITY_VERT: // with pos
                        //case MAVLink.EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL: // optical flow
                        case MAVLink.EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS: // step 1
                        case MAVLink.EKF_STATUS_FLAGS.EKF_POS_VERT_ABS: // step 1
                        //case MAVLink.EKF_STATUS_FLAGS.EKF_POS_VERT_AGL: //  range finder
                        //case MAVLink.EKF_STATUS_FLAGS.EKF_CONST_POS_MODE:  // never true when absolute - non gps
                        //case MAVLink.EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL: // optical flow
                        case MAVLink.EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS: // ekf has origin - post arm
                            //messageHigh = "HATA" + " " + currentflag.ToString().Replace("_", " ");
                            //messageHighTime = DateTime.Now;
                            break;
                    }
                }
            }

            
        }
    }

    private void ProcessMavMsgHWStatus(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.HWSTATUS);

        if (mavLinkMessage != null)
        {
            var hwstatus = mavLinkMessage.ToStructure<MAVLink.mavlink_hwstatus_t>();

            hwvoltage = hwstatus.Vcc / 1000.0f;
            i2cerrors = hwstatus.I2Cerr;
        }
    }

    private void ProcessMavMsgSysTime(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME);

        if (mavLinkMessage != null)
        {
            var systime = mavLinkMessage.ToStructure<MAVLink.mavlink_system_time_t>();
            
            GpsBilgisi.UpdateFromSysTimeMsg(systime);
        }
    }

    private void ProcessMavMsgASAC(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.AIRSPEED_AUTOCAL);

        if (mavLinkMessage != null)
        {
            var asac = mavLinkMessage.ToStructure<MAVLink.mavlink_airspeed_autocal_t>();

            asratio = asac.ratio;
        }
    }

    private void ProcessMavMsgVibration(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.VIBRATION);

        if (mavLinkMessage != null)
        {
            var vibe = mavLinkMessage.ToStructure<MAVLink.mavlink_vibration_t>();

            Titresim.UpdateFromVibrationMsg(vibe);
        }
    }

    private void ProcessMavMsgMntStatus(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.MOUNT_STATUS);

        if (mavLinkMessage != null)
        {
            var status = mavLinkMessage.ToStructure<MAVLink.mavlink_mount_status_t>();

            campointa = status.pointing_a / 100.0f;
            campointb = status.pointing_b / 100.0f;
            campointc = status.pointing_c / 100.0f;
        }
    }

    private void ProcessMavMsgOptFlow(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.OPTICAL_FLOW);

        if (mavLinkMessage != null)
        {
            var optflow = mavLinkMessage.ToStructure<MAVLink.mavlink_optical_flow_t>();
            OpticFlow.UpdateFromOptFlowMsg(optflow);
        }
    }

    private void ProcessMavMsgHilCtrls(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.HIL_CONTROLS);

        if (mavLinkMessage != null) // hil mavlink 0.9 and 1.0
        {
            var hil = mavLinkMessage.ToStructure<MAVLink.mavlink_hil_controls_t>();

            hilch1 = (int)(hil.roll_ailerons * 10000);
            hilch2 = (int)(hil.pitch_elevator * 10000);
            hilch3 = (int)(hil.throttle * 10000);
            hilch4 = (int)(hil.yaw_rudder * 10000);

            //MAVLink.packets[(byte)MAVLink.MSG_NAMES.HIL_CONTROLS);
        }
    }

    private void ProcessMavMsgRcChannelsScaled(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED);

        if (mavLinkMessage != null) // hil mavlink 0.9
        {
            var hil = mavLinkMessage.ToStructure<MAVLink.mavlink_rc_channels_scaled_t>();

            hilch1 = hil.chan1_scaled;
            hilch2 = hil.chan2_scaled;
            hilch3 = hil.chan3_scaled;
            hilch4 = hil.chan4_scaled;
            hilch5 = hil.chan5_scaled;
            hilch6 = hil.chan6_scaled;
            hilch7 = hil.chan7_scaled;
            hilch8 = hil.chan8_scaled;

            // Console.WriteLine("RC_CHANNELS_SCALED Packet");

            stateMav.clearPacket((uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED);
        }
    }

    private void ProcessMavMsgHighLatency(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY);

        if (mavLinkMessage != null)
        {
            var highlatency = mavLinkMessage.ToStructure<MAVLink.mavlink_high_latency_t>();

            InisYapildi = highlatency.landed_state == 1;

            if ((highlatency.base_mode & (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED) != 0)
            {
                var modelist = Common.getModesList(firmware);

                if (modelist != null)
                {
                    var found = false;

                    foreach (var pair in modelist)
                        if (pair.Key == highlatency.custom_mode)
                        {
                            Mod = pair.Value;
                            _mode = highlatency.custom_mode;
                            found = true;
                            break;
                        }

                    if (!found) Mod = "Unknown";
                }
            }

            Durum.UpdateFromHighLatencyMsg(highlatency);
            Kumanda.UpdateFromHighLatencyMsg(highlatency);

            GpsBilgisi.UpdateFromHighLatencyMsg(highlatency);

            HavaHizi = highlatency.airspeed;
            YerHizi = highlatency.groundspeed;
            TirmanmaHizi = highlatency.climb_rate;

            HedefHavaHizi = highlatency.airspeed_sp;
            Batarya.Kalan = highlatency.battery_remaining;

            OtopilotSicakligi = (byte)highlatency.temperature;
            HavaSicakligi = (byte)highlatency.temperature_air;
            FailSafe = highlatency.failsafe > 0;

            NavKontrol.UpdateFromHighLatencyMsg(highlatency);
        }
    }

    private void ProcessMavMsgFenceStatus(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.FENCE_STATUS);

        if (mavLinkMessage != null)
        {
            var fence = mavLinkMessage.ToStructure<MAVLink.mavlink_fence_status_t>();

            if (fence.breach_status != (byte)MAVLink.FENCE_BREACH.NONE)
            {
                // fence breached
                messageHigh = "Fence Breach";
                messageHighTime = DateTime.Now;
            }

            stateMav.clearPacket((uint)MAVLink.MAVLINK_MSG_ID.FENCE_STATUS);
        }
    }

    private void ProcessMavMsgOtoPilotVersion(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION);

        if (mavLinkMessage != null)
        {
            var version = mavLinkMessage.ToStructure<MAVLink.mavlink_autopilot_version_t>();
            //#define FIRMWARE_VERSION 3,4,0,FIRMWARE_VERSION_TYPE_DEV
            //		flight_sw_version	0x03040000	uint

            var main = (byte)(version.flight_sw_version >> 24);
            var sub = (byte)((version.flight_sw_version >> 16) & 0xff);
            var rev = (byte)((version.flight_sw_version >> 8) & 0xff);
            var type =
                (MAVLink.FIRMWARE_VERSION_TYPE)(version.flight_sw_version & 0xff);

            this.Surum = new Version(main, sub, rev, (int)type);

            try
            {
                capabilities = (uint)(MAVLink.MAV_PROTOCOL_CAPABILITY)version.capabilities;
            }
            catch
            {
            }


            EkranaYazdir("Flight SW Version: " + version.flight_sw_version);
            EkranaYazdir("Middleware SW: " + version.middleware_sw_version);
            EkranaYazdir("OS Custom: " + version.os_custom_version);
            EkranaYazdir("OS SW: " + version.os_sw_version);
            EkranaYazdir("board_version: " + version.board_version);
            EkranaYazdir("Vendor ID: " + version.vendor_id);
            EkranaYazdir("Product ID: " + version.product_id);
            EkranaYazdir("Board Version: " + version.board_version);

            stateMav.clearPacket((uint)MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION);
        }
    }

    private void ProcessMavMsgPosTarget(MAVState stateMav)
    {
        var mavLinkMessage = stateMav.getPacket((uint)MAVLink.MAVLINK_MSG_ID.POSITION_TARGET_GLOBAL_INT);

        if (mavLinkMessage != null)
        {
            var postraget = mavLinkMessage.ToStructure<MAVLink.mavlink_position_target_global_int_t>();

            if (postraget.coordinate_frame == (byte)MAVLink.MAV_FRAME.GLOBAL_INT)
                TargetLocation = new PointLatLngAlt(postraget.lat_int / 1e7, postraget.lon_int / 1e7, postraget.alt,
                    postraget.type_mask.ToString());

            if (postraget.coordinate_frame == (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT)
                TargetLocation = new PointLatLngAlt(postraget.lat_int / 1e7, postraget.lon_int / 1e7,
                    postraget.alt + HomeAlt,
                    postraget.type_mask.ToString());
        }
    }

    private void ProcessMavMsgLocalPosNed(MAVState mavState)
    {
        var mavLinkMessage = mavState.getPacket((uint)MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED);

        if (mavLinkMessage != null)
        {
            var lpned = mavLinkMessage.ToStructure<MAVLink.mavlink_local_position_ned_t>();

            var loc = HomeLocation.gps_offset(lpned.y, lpned.x);
        }
    }


    private static void EkranaYazdir(string text)
    {
        if (log.Logger.Repository.Configured)
            log.Info(text);
        else
            Console.WriteLine(text);
    }

    #endregion
}