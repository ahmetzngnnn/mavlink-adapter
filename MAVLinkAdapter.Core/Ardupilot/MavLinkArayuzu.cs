using System.Collections;
using System.Reactive.Subjects;
using System.Reflection;
using System.Runtime.ConstrainedExecution;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Xml;
using log4net;
using MAVLinkAdapter.Core.Communication;
using MAVLinkAdapter.Core.Utilities;
using Timer = System.Timers.Timer;

namespace MAVLinkAdapter.Core.Ardupilot;

public partial class MavLinkArayuzu : MAVLink, IDisposable, IMAVLinkArayuzu, IMAVLinkArayuzuKayitOkuma
{
    #region ### private değişkenler (Fields) ###
    /// <summary>
    /// log kaydını tutmak için kullanılır
    /// </summary>
    private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

    private readonly Subject<int> _alinanNesneBaytlari = new();
    private readonly Subject<int> _gonderilenNesneBaytlari = new();
    private ICommsSerial _baseStream;
    private int _bps1;
    private int _bps2;

    private volatile bool _giveComport;

    private int _mavlink1count;
    private int _mavlink2count;
    private int _mavlink2signed;

    private int _parampoll;

    /// <summary>
    ///     Secilen Bilesen Id'sini tutar
    /// </summary>
    private int _seciliBilesenId;

    /// <summary>
    ///     Secilen Sistem Id'sini tutar
    /// </summary>
    private int _seciliSistemId;

    private string buildplaintxtline = "";

    private readonly bool debug = false;

    private int inject_seq_no;

    private DateTime lastparamset = DateTime.MinValue;

    /// <summary>
    ///     mavlink surumu
    /// </summary>
    private byte mavlinkSurum;


    /// <summary>
    ///     seri port okuma kilidi olarak kullanılır
    /// </summary>
    private volatile object okumaKilidi = new();


    private Dictionary<Stream, Tuple<string, long>> streamfncache = new();

    private readonly List<KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>>> Subscriptions = new();

    private readonly double t7 = 1.0e7;

    /// <summary>
    ///     seri port yazma kilidi olarak kullanılır
    /// </summary>
    private volatile object yazmaKilidi = new();

    private DateTime _bpstime { get; set; }

    #endregion

    #region ### public değişkenler (Properties) ###

    /// <summary>
    ///     Baglanti zaman asimi
    /// </summary>
    public double BaglantiZamanAsimi = 30;

    public MAVList MAVlist;


    internal string plaintxtline = "";

    public bool SaltOkunur = false;


    public ICommsSerial BaseStream
    {
        get => _baseStream;
        set
        {
            // Kullanıcı port seçimini her değiştirdiğinde bu çağrılır,
            // bu nedenle önceki nesneleri temizlediğimizden emin olmalıyız,
            // böylece sistem kaynaklarının temizlenmesini çöp toplayıcıya bırakmayız.
            if (_baseStream != null)
            {
                try
                {
                    if (_baseStream.IsOpen) _baseStream.Close();
                }
                catch
                {
                }

                IDisposable dsp = _baseStream;
                if (dsp != null)
                    try
                    {
                        dsp.Dispose();
                    }
                    catch
                    {
                    }
            }

            _baseStream = value;
        }
    }

    public ICommsSerial MirrorStream { get; set; }
    public bool MirrorStreamWrite { get; set; }

    /// <summary>
    ///     Yer Kontrol İstasyonu sistem ID'si
    /// </summary>
    public static byte YkiSistemId { get; set; } = 255;

    /// <summary>
    ///     özel kullanım için comport erişimini önlemek için kullanılır
    /// </summary>
    public bool giveComport
    {
        get => _giveComport;
        set => _giveComport = value;
    }

    /// <summary>
    ///     giden paket gönderimi için kullanılır
    /// </summary>
    public int PaketSayisi { get; internal set; }

    /// <summary>
    ///     Alınan bayt sayısı gözlemlenebilir, baytların kendileri alındığında bildirilir
    /// </summary>
    public IObservable<int> AlinanBaytlar => _alinanNesneBaytlari;

    /// <summary>
    ///     Gönderilen bayt sayısı gözlemlenebilir, baytların kendileri alındığında bildirilir
    /// </summary>
    public IObservable<int> GonderilenBaytlar => _gonderilenNesneBaytlari;

    /// <summary>
    ///     Observable of the count of packets skipped (on reception),
    ///     calculated from periods where received packet sequence is not
    ///     contiguous
    /// </summary>
    public Subject<int> PaketKaybiOldugunda { get; set; }

    public Subject<int> PaketAlindiginda { get; set; }

    /// <summary>
    ///     konsol paket ekranını açar
    /// </summary>
    public bool debugmavlink { get; set; }


    public BufferedStream kayitDosyasi { get; set; }
    public BufferedStream hamKayitDosyasi { get; set; }

    public MAVState MAV
    {
        get => MAVlist[SeciliSistemId, SeciliBilesenId];
        set => MAVlist[SeciliSistemId, SeciliBilesenId] = value;
    }


    /// <summary>
    ///     Bağlantı kurulan sistem ID'si
    /// </summary>
    public int SeciliSistemId
    {
        get => _seciliSistemId;
        set
        {
            if (_seciliSistemId == value)
                return;
            _seciliSistemId = value;
            MavChanged?.Invoke(this, null);
        }
    }

    /// <summary>
    ///     Bağlantı kurulan sistemin bileşen ID'si
    /// </summary>
    public int SeciliBilesenId
    {
        get => _seciliBilesenId;
        set
        {
            if (_seciliBilesenId == value)
                return;
            _seciliBilesenId = value;
            MavChanged?.Invoke(this, null);
        }
    }

    public DateTime sonKayitOkuma { get; set; }

    public BinaryReader logplaybackfile { get; set; }
    public bool logreadmode { get; set; }
    #endregion

    #region ### Oluşturucular (Constructors) ###

    public MavLinkArayuzu()
    {
        log4net.Config.XmlConfigurator.Configure();


        MAVlist = new MAVList(this);

        // init fields
        PaketSayisi = 0;
        _alinanNesneBaytlari = new Subject<int>();
        _gonderilenNesneBaytlari = new Subject<int>();
        PaketKaybiOldugunda = new Subject<int>();
        PaketAlindiginda = new Subject<int>();
        okumaKilidi = new object();

        mavlinkSurum = 0;

        debugmavlink = false;
        sonKayitOkuma = DateTime.MinValue;
        kayitDosyasi = null;
        hamKayitDosyasi = null;
        _bps1 = 0;
        _bps2 = 0;
        _bpstime = DateTime.MinValue;
        _mavlink1count = 0;
        _mavlink2count = 0;
        _mavlink2signed = 0;
    }


    #endregion

    #region ### Yıkıcılar (Destructors) ###

    // yıkıcı metod
    ~MavLinkArayuzu()
    {
        Dispose();
    }
    public void Dispose()
    {
        _alinanNesneBaytlari?.Dispose();
        _gonderilenNesneBaytlari?.Dispose();
        MAVlist?.Dispose();

        Kapat();


        MirrorStream = null;

        logreadmode = false;
        logplaybackfile = null;
    }

    #endregion

    #region ### public olaylar (Events) ###

    public event EventHandler<MAVLinkMessage> OnPacketReceived;

    public event EventHandler ParamListChanged;

    public event EventHandler MavChanged;

    public event EventHandler CommsClose;

    public event ProgressEventHandler Progress;

    #endregion

    #region ### public metotlar ###

    public MAVLinkMessage nabizAl()
    {
        giveComport = true;
        var start = DateTime.Now;
        var readcount = 0;
        while (true)
        {
            var buffer = paketOku();
            readcount++;
            if (buffer.Length > 5)
                //log.Info("getHB packet received: " + buffer.Length + " btr " + BaseStream.BytesToRead + " type " + buffer.msgid );
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.HEARTBEAT)
                {
                    var hb = buffer.ToStructure<mavlink_heartbeat_t>();

                    if (hb.type != (byte)MAV_TYPE.GCS)
                    {
                        SetupMavConnect(buffer, hb);

                        giveComport = false;
                        return buffer;
                    }
                }

            if (DateTime.Now > start.AddMilliseconds(2200) || readcount > 200) // was 1200 , now 2.2 sec
            {
                giveComport = false;
                return MAVLinkMessage.Invalid;
            }
        }
    }

    public void Kapat()
    {
        try
        {
            kayitDosyasi?.Close();
        }
        catch
        {
        }

        try
        {
            hamKayitDosyasi?.Close();
        }
        catch
        {
        }


        try
        {
            if (BaseStream != null && BaseStream.IsOpen)
                BaseStream.Close();
        }
        catch
        {
        }

        try
        {
            CommsClose?.Invoke(this, null);
        }
        catch
        {
        }
    }

    public void Ac(bool parametreleriAl = false)
    {
        if (BaseStream == null)
            return;

        if (parametreleriAl)
            AcAp(true);
        else
            AcAp(false);


        ParamListChanged?.Invoke(this, null);
    }


    /// <summary>
    ///     Mavlink paketlerini okumak için Seri Okuyucu. POLL yöntemi
    /// </summary>
    /// <returns> MAVLink paketi </returns>
    public MAVLinkMessage paketOku()
    {
        var buffer = new byte[MAVLINK_MAX_PACKET_LEN + 25];
        var count = 0;
        var length = 0;
        var readcount = 0;
        MAVLinkMessage message = null;

        var start = DateTime.Now;


        if (debug)
            EkranaYazdir(DateTime.Now.Millisecond + " SR0 " + BaseStream?.BytesToRead);

        lock (okumaKilidi)
        {
            if (debug)
                EkranaYazdir(DateTime.Now.Millisecond + " SR1 " + BaseStream?.BytesToRead);

            while (BaseStream != null && BaseStream.IsOpen)
            {
                try
                {
                    if (readcount > 300) break;

                    readcount++;


                    if (BaseStream.ReadTimeout != 1200)
                        BaseStream.ReadTimeout =
                            1200; // Karakterler arasında 1200 ms var - gps algılaması bunu gerektirir.

                    // Dahili başvuru için zaman güncellemesi
                    MAV.cs.datetime = DateTime.Now;

                    var to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                    if (debug)
                        EkranaYazdir(DateTime.Now.Millisecond + " SR1a " + BaseStream?.BytesToRead);

                    while (BaseStream.IsOpen && BaseStream.BytesToRead <= 0)
                    {
                        if (DateTime.Now > to)
                        {
                            log.InfoFormat("MAVLINK: 1 wait time out btr {0} len {1}", BaseStream?.BytesToRead,
                                length);
                            throw new TimeoutException("Timeout");
                        }

                        Thread.Sleep(1);
                        if (debug)
                            EkranaYazdir(DateTime.Now.Millisecond + " SR0b " + BaseStream?.BytesToRead);
                    }

                    if (debug)
                        EkranaYazdir(DateTime.Now.Millisecond + " SR1a " + BaseStream?.BytesToRead);
                    if (BaseStream.IsOpen)
                    {
                        BaseStream.Read(buffer, count, 1);
                        if (hamKayitDosyasi != null && hamKayitDosyasi.CanWrite)
                            hamKayitDosyasi.WriteByte(buffer[count]);
                    }

                    if (debug)
                        EkranaYazdir(DateTime.Now.Millisecond + " SR1b " + BaseStream?.BytesToRead);
                }
                catch (Exception e)
                {
                    log.Info("MAVLink paket okuma hatası: " + e);
                    break;
                }

                // mavlink paketine benzeyip benzemediğini kontrol et ayrıca istisnaları kontrol et ve konsola yaz
                if (buffer[0] != 0xfe && buffer[0] != 'U' && buffer[0] != 0xfd)
                {
                    if ((buffer[0] >= 0x20 && buffer[0] <= 127) || buffer[0] == '\n' || buffer[0] == '\r')
                    {
                        // satır sonlandırmasını kontrol et
                        if (buffer[0] == '\r' || buffer[0] == '\n')
                        {
                            // yeni satır doğru mu kontrol et
                            if (buildplaintxtline.Length > 3)
                                plaintxtline = buildplaintxtline;

                            log.Info(plaintxtline);

                            // yeni satır için sıfırla
                            buildplaintxtline = "";
                        }

                        TCPConsole.Write(buffer[0]);
                        EkranaYazdir(((char)buffer[0]).ToString());
                        buildplaintxtline += (char)buffer[0];
                    }

                    _alinanNesneBaytlari.OnNext(1);
                    count = 0;
                    buffer[1] = 0;
                    continue;
                }

                // geçerli pakette sayımı sıfırla
                readcount = 0;

                if (debug)
                    EkranaYazdir(DateTime.Now.Millisecond + " SR2 " + BaseStream?.BytesToRead);

                // başlık için kontrol et
                if (buffer[0] == 0xfe || buffer[0] == 0xfd || buffer[0] == 'U')
                {
                    var mavlinkv2 = buffer[0] == MAVLINK_STX ? true : false;

                    int headerlength = mavlinkv2 ? MAVLINK_CORE_HEADER_LEN : MAVLINK_CORE_HEADER_MAVLINK1_LEN;
                    var headerlengthstx = headerlength + 1;

                    // eğer başlık varsa ve başka karakter yoksa, uzunluğu ve paket tanımlayıcılarını al
                    if (count == 0)
                    {
                        var to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                        while (BaseStream.IsOpen && BaseStream.BytesToRead < headerlength)
                        {
                            if (DateTime.Now > to)
                            {
                                log.InfoFormat("MAVLINK: 2 wait time out btr {0} len {1}", BaseStream.BytesToRead,
                                    length);
                                throw new TimeoutException("Timeout");
                            }

                            Thread.Sleep(1);
                        }

                        var read = BaseStream.Read(buffer, 1, headerlength);
                        count = read;
                        if (hamKayitDosyasi != null && hamKayitDosyasi.CanWrite)
                            hamKayitDosyasi.Write(buffer, 1, read);
                    }

                    // paket uzunluğu
                    if (buffer[0] == MAVLINK_STX)
                    {
                        length = buffer[1] + headerlengthstx +
                                 MAVLINK_NUM_CHECKSUM_BYTES; // data + header + checksum - magic - length
                        if ((buffer[2] & MAVLINK_IFLAG_SIGNED) > 0) length += MAVLINK_SIGNATURE_BLOCK_LEN;
                    }
                    else
                    {
                        length = buffer[1] + headerlengthstx +
                                 MAVLINK_NUM_CHECKSUM_BYTES; // data + header + checksum - U - length    
                    }

                    if (count >= headerlength)
                    {
                        try
                        {
                            var to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                            while (BaseStream.IsOpen && BaseStream.BytesToRead < length - headerlengthstx)
                            {
                                if (DateTime.Now > to)
                                {
                                    log.InfoFormat("MAVLINK: 3 wait time out btr {0} len {1}",
                                        BaseStream.BytesToRead, length);
                                    break;
                                }

                                Thread.Sleep(1);
                            }

                            if (BaseStream.IsOpen)
                            {
                                var read = BaseStream.Read(buffer, headerlengthstx, length - headerlengthstx);
                                if (read != length - headerlengthstx)
                                    log.InfoFormat("MAVLINK: bad read {0}, {1}, {2}", headerlengthstx, length,
                                        count);
                                if (hamKayitDosyasi != null && hamKayitDosyasi.CanWrite)
                                    // sadece okunan şeyi yaz, temp tüm pakettir, bu yüzden 6-son
                                    hamKayitDosyasi.Write(buffer, headerlengthstx, read);
                            }


                            count = length;
                        }
                        catch
                        {
                        }

                        break;
                    }
                }

                count++;
                if (count == 299)
                    break;
            }

            if (debug)
                Console.WriteLine(DateTime.Now.Millisecond + " SR3 " + BaseStream?.BytesToRead);
        } // okuma kilidini serbest bırak

        // paketi doğru uzunlukta yeniden boyutlandır
        Array.Resize(ref buffer, count);

        // bayt sayısı ekle
        _alinanNesneBaytlari.OnNext(buffer.Length);

        // nabiz (bps) i̇stati̇sti̇kleri̇ni̇ güncelle
        if (_bpstime.Second != DateTime.Now.Second)
        {
            long btr = 0;
            if (BaseStream != null && BaseStream.IsOpen) btr = BaseStream.BytesToRead;

            var bpsStat = string.Format(
                "bps {0} loss {1} left {2} mem {3} mav2 {4} sign {5} mav1 {6} mav2 {7} signed {8}",
                _bps1,
                MAV.synclost,
                btr,
                GC.GetTotalMemory(false) / 1024 / 1024.0,
                MAV.mavlinkv2,
                MAV.signing,
                _mavlink1count,
                _mavlink2count,
                _mavlink2signed);

            EkranaYazdir(bpsStat);

            _bps2 = _bps1; // prev sec
            _bps1 = 0; // current sec
            _bpstime = DateTime.Now;
            _mavlink1count = 0;
            _mavlink2count = 0;
            _mavlink2signed = 0;
        }

        _bps1 += buffer.Length;

        if (buffer.Length == 0)
            return MAVLinkMessage.Invalid;

        if (message == null)
            message = new MAVLinkMessage(buffer, DateTime.UtcNow);

        var msgid = message.msgid;

        var msginfo = MAVLINK_MESSAGE_INFOS.GetMessageInfo(msgid);

        // crc hesapla
        var sigsize = message.sig != null ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
        var crc = MavlinkCRC.crc_calculate(buffer, message.Length - sigsize - MAVLINK_NUM_CHECKSUM_BYTES);

        // mavlink 1.0/2.0 için ekstra crc biti hesapla
        if (message.header == 0xfe || message.header == 0xfd) crc = MavlinkCRC.crc_accumulate(msginfo.crc, crc);

        // mesaj uzunluğu boyutunu tablo ile kontrol edin (mavlink1 belirgin boyut kontrolü | mavlink2 tümüne izin ver, küçük boyut 0 kırpılmış ve büyük boyut bilinmeyen uzantı)
        if (!message.ismavlink2 && message.payloadlength != msginfo.minlength)
        {
            if (msginfo.length == 0) // bilinmeyen paketleri atla
            {
                log.InfoFormat("bilinmeyen paket türü {0}", message.msgid);
            }
            else
            {
                log.InfoFormat("Mavlink Kötü Paket (Uzunluk Hatası) len {0} pkno {1}", buffer.Length, message.msgid);
                return MAVLinkMessage.Invalid;
            }
        }

        // crc kontrol et
        if (message.crc16 >> 8 != crc >> 8 ||
            (message.crc16 & 0xff) != (crc & 0xff))
        {
            if (buffer.Length > 5 && msginfo.name != null)
                log.InfoFormat("Mavlink Kötü Paket (crc hatası) len {0} crc {1} vs {4} pkno {2} {3}", buffer.Length,
                    crc, message.msgid, msginfo.name,
                    message.crc16);
            return MAVLinkMessage.Invalid;
        }

        var sysid = message.sysid;
        var compid = message.compid;
        var packetSeqNo = message.seq;

        // günlük oynatmada gcs içeren herhangi bir sysid/compid için bir durum oluştur
        if (!MAVlist.Contains(sysid, compid))
        {
            // öğe oluştur - gizli
            MAVlist.AddHiddenList(sysid, compid);
            // bağlantıda paket kaybı sayacını önle
            MAVlist[sysid, compid].recvpacketcount = unchecked(packetSeqNo - 1);
        }

        // bir kez ayarlandığında geri döndürülemez
        if (!MAVlist[sysid, compid].mavlinkv2)
            MAVlist[sysid, compid].mavlinkv2 = message.buffer[0] == MAVLINK_STX ? true : false;

        // istatistik sayısı güncelle
        if (message.buffer[0] == MAVLINK_STX)
            _mavlink2count++;
        else if (message.buffer[0] == MAVLINK_STX_MAVLINK1)
            _mavlink1count++;

        //sig'in pakete dahil edilip edilmediğini kontrol edin ve imzayı yok saymıyoruz (imzalama kontrol edilmedi, aksi takdirde imzalamayı etkinleştirmeyeceğiz) 
        //logreadmode imzalamayı her zaman yok sayarız, çünkü imzalamada başarısız olurlarsa günlükte yer almazlar
        if (message.sig != null && !MAVlist[sysid, compid].signingignore)
        {
            _mavlink2signed++;

            var valid = true;

            foreach (var AuthKey in MAVAuthKeys.Keys.Values)
                using (var signit = new SHA256Managed())
                {
                    signit.TransformBlock(AuthKey.Key, 0, AuthKey.Key.Length, null, 0);
                    signit.TransformFinalBlock(message.buffer, 0, message.Length - MAVLINK_SIGNATURE_BLOCK_LEN + 7);
                    var ctx = signit.Hash;

                    // 48'e kadar kırp
                    Array.Resize(ref ctx, 6);

                    for (var i = 0; i < ctx.Length; i++)
                        if (ctx[i] != message.sig[7 + i])
                        {
                            // bu anahtar değil, sıradakini kontrol et
                            valid = false;
                            break;
                        }

                    if (!valid)
                        continue;

                    // geçerli anahtar alındı
                    MAVlist[sysid, compid].linkid = message.sig[0];

                    MAVlist[sysid, compid].signingKey = AuthKey.Key;

                    imzalamaAc(sysid, compid);

                    break;
                }

            if (!valid)
            {
                log.InfoFormat("Paket imza hatası verdi ancak crc'yi geçti");
                return MAVLinkMessage.Invalid;
            }
        }

        // Paket şimdi doğrulandı

        // oynatma sırasında gcs paketleri de dahil olmak üzere akıştan wp'leri/rallileri/çitleri/kamera geri bildirimlerini/parametrelerini çıkarın
        if (buffer.Length >= 5) getInfoFromStream(ref message, sysid, compid);


        // paket kaybı istatistiklerini güncelle
        if (MAVlist[sysid, compid].packetlosttimer.AddSeconds(5) < DateTime.Now)
        {
            MAVlist[sysid, compid].packetlosttimer = DateTime.Now;
            MAVlist[sysid, compid].packetslost = MAVlist[sysid, compid].packetslost * 0.8f;
            MAVlist[sysid, compid].packetsnotlost = MAVlist[sysid, compid].packetsnotlost * 0.8f;
        }

        try
        {
            if ((message.header == 'U' || message.header == 0xfe || message.header == 0xfd) &&
                buffer.Length >= message.payloadlength)
            {
                // seqno'ya göre paketleri kaybedip kaybetmediğimizi kontrol et
                var expectedPacketSeqNo = (MAVlist[sysid, compid].recvpacketcount + 1) % 0x100;

                {
                    // ikinci kısım, çift seqno'lar gönderen bir 3dr radyo hatasının etrafında çalışmaktır
                    if (packetSeqNo != expectedPacketSeqNo && packetSeqNo != MAVlist[sysid, compid].recvpacketcount)
                    {
                        MAVlist[sysid, compid].synclost++; // Gerçek senkronizasyon kaybı
                        var numLost = 0;

                        if (packetSeqNo < MAVlist[sysid, compid].recvpacketcount + 1)
                            //recvpacketcount = 255 olduğunda 10 < 256 = true ise % 0x100 bu başarısız olur
                            numLost = 0x100 - expectedPacketSeqNo + packetSeqNo;
                        else
                            numLost = packetSeqNo - expectedPacketSeqNo;

                        MAVlist[sysid, compid].packetslost += numLost;
                        PaketKaybiOldugunda.OnNext(numLost);


                        log.InfoFormat("mav {2}-{4} seqno {0} exp {3} pkts lost {1}", packetSeqNo, numLost,
                            sysid,
                            expectedPacketSeqNo, compid);
                    }

                    MAVlist[sysid, compid].packetsnotlost++;

                    //Console.WriteLine("{0} {1}", sysid, packetSeqNo);

                    MAVlist[sysid, compid].recvpacketcount = packetSeqNo;
                }
                PaketAlindiginda.OnNext(1);

                // mav başına paket istatistikleri
                if (!MAVlist[sysid, compid].packetspersecond.ContainsKey(msgid) ||
                    double.IsInfinity(MAVlist[sysid, compid].packetspersecond[msgid]))
                    MAVlist[sysid, compid].packetspersecond[msgid] = 0;
                if (!MAVlist[sysid, compid].packetspersecondbuild.ContainsKey(msgid))
                    MAVlist[sysid, compid].packetspersecondbuild[msgid] = DateTime.Now;

                MAVlist[sysid, compid].packetspersecond[msgid] = (1000 /
                                                                  (DateTime.Now -
                                                                   MAVlist[sysid, compid]
                                                                       .packetspersecondbuild[msgid])
                                                                  .TotalMilliseconds +
                                                                  MAVlist[sysid, compid].packetspersecond[
                                                                      msgid]) / 2;

                MAVlist[sysid, compid].packetspersecondbuild[msgid] = DateTime.Now;

                //Console.WriteLine("Packet {0}",temp[5]);

                // paket geçmişini tut
                lock (yazmaKilidi)
                {
                    MAVlist[sysid, compid].addPacket(message);

                    // 3dr radyo durum paketi mevcut mav'a enjekte edildi
                    // çoğu telsizin sabit bir sysid VE componentid'i vardır ...
                    if ((msgid == (byte)MAVLINK_MSG_ID.RADIO_STATUS || msgid == (byte)MAVLINK_MSG_ID.RADIO)
                        && message.compid == 68 &&
                        message.sysid == 63) // ascii 63="3" ascii 68="D" => "3D" branding
                        MAVlist[SeciliSistemId, SeciliBilesenId].addPacket(message);

                    // MultiPoint ürün yazılımına sahip RFD900X telsizler kendilerini bağlı oldukları hava aracı ile aynı sysid ile ve 68'lik sabit bir bileşen kimliği ile sunarlar
                    if ((msgid == (byte)MAVLINK_MSG_ID.RADIO_STATUS || msgid == (byte)MAVLINK_MSG_ID.RADIO)
                        && message.compid == 68 && message.sysid != 63)
                    {
                        // mevcut mav'ı güncelleyin - bu, çoklu rssi paketlerine sahip daha yeni bir ürün yazılımı kullanıyorsanız rssi'nin atlamasına neden olacaktır
                        MAVlist[SeciliSistemId, SeciliBilesenId].addPacket(message);
                        // hedef mav'ı güncelleyin
                        MAVlist[message.sysid, SeciliBilesenId].addPacket(message);
                    }

                    // adsb paketleri yönlendirilir ve herhangi bir sysid/compid'den gelebilir
                    if (msgid == (byte)MAVLINK_MSG_ID.ADSB_VEHICLE)
                    {
                        var adsb = message.ToStructure<mavlink_adsb_vehicle_t>();

                        var id = adsb.ICAO_address.ToString("X5");
                    }

                    if (msgid == (byte)MAVLINK_MSG_ID.COLLISION)
                    {
                        var coll = message.ToStructure<mavlink_collision_t>();

                        var id = coll.id.ToString("X5");

                        var coll_type = (MAV_COLLISION_SRC)coll.src;

                        var action = (MAV_COLLISION_ACTION)coll.action;

                        if (action > MAV_COLLISION_ACTION.REPORT)
                        {
                            // bir tehdide tepki gösteriyoruz
                        }

                        var threat_level = (MAV_COLLISION_THREAT_LEVEL)coll.threat_level;

                    }
                }

                // görülen sysid'leri nabiz paketine göre ayarlayın - bu 3dr radyo paketlerini gizleyecektir
                if (msgid == (uint)MAVLINK_MSG_ID.UAVCAN_NODE_STATUS)
                {
                    var cannode = message.ToStructure<mavlink_uavcan_node_status_t>();

                    // görülen bir sysid ekleyin
                    if (!MAVlist.Contains(sysid, compid, false))
                    {
                        // bağlantıdan veya günlük oynatmadan ayarlandığından emin ol
                        MAVlist.Create(sysid, compid);
                        MAVlist[sysid, compid].aptype = MAV_TYPE.ONBOARD_CONTROLLER;
                        MAVlist[sysid, compid].apname = MAV_AUTOPILOT.INVALID;
                        MAVlist[sysid, compid].CANNode = true;
                        setAPType(sysid, compid);

                        // yeni cihaz, bu yüzden düğüm bilgisi iste
                        KomutGonder(sysid, compid, MAV_CMD.UAVCAN_GET_NODE_INFO, 0, 0, 0, 0, 0, 0, 0, false);
                    }
                }

                if (msgid == (uint)MAVLINK_MSG_ID.UAVCAN_NODE_INFO)
                {
                    var cannode = message.ToStructure<mavlink_uavcan_node_info_t>();

                    var name = Encoding.ASCII.GetString(cannode.name);

                    MAVlist[sysid, compid].VersionString = name;

                    MAVlist[sysid, compid].SoftwareVersions =
                        cannode.sw_version_major + "." + cannode.sw_version_minor;
                }

                // görülen sysid'leri nabiz paketine göre ayarlayın - bu, 3dr radyo paketlerini gizleyecektir (bir RADIO_STATUS gönderen, ancak bir HEARTBEAT göndermeyen)
                if (msgid == (byte)MAVLINK_MSG_ID.HEARTBEAT)
                {
                    var hb = message.ToStructure<mavlink_heartbeat_t>();

                    // yki degilse
                    if (hb.type != (byte)MAV_TYPE.GCS)
                    {
                        // görülen bir sistem id ekle
                        if (!MAVlist.Contains(sysid, compid, false))
                        {
                            // bağlantı veya kayıt oynatmada ayarlandığından emin olun
                            MAVlist.Create(sysid, compid);
                            MAVlist[sysid, compid].aptype = (MAV_TYPE)hb.type;
                            MAVlist[sysid, compid].apname = (MAV_AUTOPILOT)hb.autopilot;
                            setAPType(sysid, compid);
                        }

                        // tek uzak cihaza bağlanın. / varsayılan olarak görülen ilk cihaza
                        if (MAVlist.Count == 1)
                        {
                            // compidset yeni mavstate'i tetikleyeceği için private olarak ayarla
                            _seciliSistemId = sysid;
                            SeciliBilesenId = compid;
                        }
                    }
                }

                // Yanlızca aktif Mav için işle
                if (SeciliSistemId == sysid && SeciliBilesenId == compid)
                    PacketReceived(message);

                OnPacketReceived?.Invoke(this, message);

                if (debugmavlink)
                    DebugPacket(message);

                if (msgid == (byte)MAVLINK_MSG_ID.STATUSTEXT) // status text
                {
                    var msg = message.ToStructure<mavlink_statustext_t>();

                    var sev = msg.severity;

                    var logdata = Encoding.ASCII.GetString(msg.text);
                    var ind = logdata.IndexOf('\0');
                    if (ind != -1)
                        logdata = logdata.Substring(0, ind);
                    log.Info(DateTime.Now + " " + sev + " " + logdata);

                    MAVlist[sysid, compid].cs.messages.Add(logdata);

                    // görülen sysid'leri nabiz paketine göre ayarlayın - bu, 3dr radyo paketlerini gizleyecektir (bir RADIO_STATUS gönderen, ancak bir HEARTBEAT göndermeyen)
                    if (sysid == SeciliSistemId && compid != SeciliBilesenId)
                        MAVlist[SeciliSistemId, SeciliBilesenId].cs.messages.Add(compid + " : " + logdata);

                    var printit = false;

                    // önem derecesi değişikliği ve otopilot sürümü aynı anda tanıtıldı, bu nedenle 0 olmayan herhangi bir sürüm kullanılabilir
                    // copter 3.4+
                    // plane 3.4+
                    if (MAVlist[sysid, compid].cs.Surum.Major > 0 || MAVlist[sysid, compid].cs.Surum.Minor >= 4)
                    {
                        if (sev <= (byte)MAV_SEVERITY.WARNING) printit = true;
                    }
                    else
                    {
                        if (sev >= 3) printit = true;
                    }

                    if (logdata.StartsWith("Tuning:") || logdata.StartsWith("PreArm:") ||
                        logdata.StartsWith("Arm:"))
                        printit = true;

                    if (printit)
                    {
                        MAVlist[sysid, compid].cs.messageHigh = logdata;
                        MAVlist[sysid, compid].cs.messageHighTime = DateTime.Now;
                    }
                }


                if (lastparamset != DateTime.MinValue && lastparamset.AddSeconds(10) < DateTime.Now)
                {
                    lastparamset = DateTime.MinValue;

                    if (BaseStream.IsOpen) KomutGonder(MAV_CMD.PREFLIGHT_STORAGE, 1, 0, 0, 0, 0, 0, 0, false);
                }


                try
                {
                    if (kayitDosyasi != null && kayitDosyasi.CanWrite)
                        lock (kayitDosyasi)
                        {
                            var datearray =
                                BitConverter.GetBytes(
                                    (ulong)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds *
                                            1000));
                            Array.Reverse(datearray);
                            kayitDosyasi.Write(datearray, 0, datearray.Length);
                            kayitDosyasi.Write(buffer, 0, buffer.Length);

                            if (msgid == 0)
                            {
                                // kalp atışında yıkama - 1 saniye
                                kayitDosyasi.Flush();
                                hamKayitDosyasi.Flush();
                            }
                        }
                }
                catch (Exception ex)
                {
                    log.Error(ex);
                }

                try
                {
                    // yansıtma akışından full rw
                    if (MirrorStream != null && MirrorStream.IsOpen)
                    {
                        MirrorStream.Yaz(buffer, 0, buffer.Length);

                        while (MirrorStream.BytesToRead > 0)
                        {
                            var buf = new byte[1024];

                            var len = MirrorStream.Read(buf, 0, buf.Length);

                            if (MirrorStreamWrite)
                                BaseStream.Yaz(buf, 0, len);
                        }
                    }
                }
                catch
                {
                }
            }
        }
        catch (Exception ex)
        {
            log.Error(ex);
        }

        // son geçerli paket alma zamanını güncelle
        MAVlist[sysid, compid].lastvalidpacket = DateTime.Now;

        return message;
    }

    public void paketGonder(object indata, int sysid, int compid)
    {
        var validPacket = false;
        foreach (var ty in MAVLINK_MESSAGE_INFOS)
            if (ty.type == indata.GetType())
            {
                validPacket = true;
                paketOlusturGonder((int)ty.msgid, indata, sysid, compid);
                return;
            }

        if (!validPacket) log.Info("Mavlink : NOT VALID PACKET sendPacket() " + indata.GetType());
    }

    public KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>> PaketTuruneAboneOl(MAVLINK_MSG_ID type,
        Func<MAVLinkMessage, bool> function, bool exclusive = false)
    {
        var item = new KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>>(type, function);

        lock (Subscriptions)
        {
            if (exclusive)
                foreach (var subitem in Subscriptions)
                    if (subitem.Key == item.Key)
                    {
                        Subscriptions.Remove(subitem);
                        break;
                    }

            log.Info("SubscribeToPacketType " + item.Key + " " + item.Value);

            Subscriptions.Add(item);
        }

        return item;
    }

    public void PaketTurunuAboneliktenCik(KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>> item)
    {
        lock (Subscriptions)
        {
            log.Info("UnSubscribeToPacketType " + item.Key + " " + item.Value);
            Subscriptions.Remove(item);
        }
    }

    public void PaketTurunuAboneliktenCik(MAVLINK_MSG_ID msgtype, Func<MAVLinkMessage, bool> item)
    {
        lock (Subscriptions)
        {
            log.Info("UnSubscribeToPacketType " + msgtype + " " + item);
            var ans = Subscriptions.Where(a => { return a.Key == msgtype && a.Value == item; });
            Subscriptions.Remove(ans.First());
        }
    }

    /// <summary>
    ///     Bir Mavlink Paketi oluşturun ve serial'e yazın
    /// </summary>
    /// <param name="tur">MAVLINK_MSG_ID'nin tür numarası </param>
    /// <param name="icerik"> MAvlink paketinin içeriği</param>
    /// <param name="sisId"> Sistem ID</param>
    /// <param name="bilId"> Bileşen ID</param>
    /// <param name="mavlink2Zorla"> Mavlink2 zorla</param>
    /// <param name="imzalamZorla"> İmzalamayı zorla</param>
    public void paketOlusturGonder(int tur, object icerik, int sisId, int bilId, bool mavlink2Zorla = false,
        bool imzalamZorla = false)
    {
        if (BaseStream == null || !BaseStream.IsOpen) return;

        if (SaltOkunur)
        {
            // allow these messages
            if (tur == (byte)MAVLINK_MSG_ID.MISSION_REQUEST_LIST ||
                tur == (byte)MAVLINK_MSG_ID.MISSION_REQUEST_PARTIAL_LIST ||
                tur == (byte)MAVLINK_MSG_ID.MISSION_REQUEST ||
                tur == (byte)MAVLINK_MSG_ID.PARAM_REQUEST_LIST ||
                tur == (byte)MAVLINK_MSG_ID.PARAM_REQUEST_READ ||
                tur == (byte)MAVLINK_MSG_ID.RALLY_FETCH_POINT ||
                tur == (byte)MAVLINK_MSG_ID.FENCE_FETCH_POINT
               )
            {
            }
            else
            {
                return;
            }
        }

        lock (yazmaKilidi)
        {
            var data = MavlinkUtil.StructureToByteArray(icerik);
            var packet = new byte[0];
            var i = 0;

            // are we mavlink2 enabled for this sysid/compid
            if (!MAVlist[sisId, bilId].mavlinkv2 && tur < 256 && !mavlink2Zorla)
            {
                var info = MAVLINK_MESSAGE_INFOS.SingleOrDefault(p => p.msgid == tur);
                if (data.Length != info.minlength) Array.Resize(ref data, (int)info.minlength);

                //Console.WriteLine(DateTime.Now + " PC Doing req "+ messageType + " " + this.BytesToRead);
                packet = new byte[data.Length + 6 + 2];

                packet[0] = MAVLINK_STX_MAVLINK1;
                packet[1] = (byte)data.Length;
                packet[2] = (byte)PaketSayisi;

                PaketSayisi++;

                packet[3] = YkiSistemId;
                packet[4] = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
                packet[5] = (byte)tur;

                i = 6;
                foreach (var b in data)
                {
                    packet[i] = b;
                    i++;
                }

                var checksum = MavlinkCRC.crc_calculate(packet, packet[1] + 6);

                checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_INFOS.GetMessageInfo((uint)tur).crc,
                    checksum);


                var ck_a = (byte)(checksum & 0xFF); ///< High byte
                var ck_b = (byte)(checksum >> 8); ///< Low byte

                packet[i] = ck_a;
                i += 1;
                packet[i] = ck_b;
                i += 1;
            }
            else
            {
                // trim packet for mavlink2
                MavlinkUtil.trim_payload(ref data);

                packet = new byte[data.Length + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES +
                                  MAVLINK_SIGNATURE_BLOCK_LEN];

                packet[0] = MAVLINK_STX;
                packet[1] = (byte)data.Length;
                packet[2] = 0; // incompat
                if (MAVlist[sisId, bilId].signing || imzalamZorla) // current mav
                    packet[2] |= MAVLINK_IFLAG_SIGNED;
                packet[3] = 0; // compat
                packet[4] = (byte)PaketSayisi;

                PaketSayisi++;

                packet[5] = YkiSistemId;
                packet[6] = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
                packet[7] = (byte)(tur & 0xff);
                packet[8] = (byte)((tur >> 8) & 0xff);
                packet[9] = (byte)((tur >> 16) & 0xff);

                i = 10;
                foreach (var b in data)
                {
                    packet[i] = b;
                    i++;
                }

                var checksum = MavlinkCRC.crc_calculate(packet, packet[1] + MAVLINK_NUM_HEADER_BYTES);

                checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_INFOS.GetMessageInfo((uint)tur).crc,
                    checksum);

                var ck_a = (byte)(checksum & 0xFF); ///< High byte
                var ck_b = (byte)(checksum >> 8); ///< Low byte

                packet[i] = ck_a;
                i += 1;
                packet[i] = ck_b;
                i += 1;

                if (MAVlist[sisId, bilId].signing || imzalamZorla)
                {
                    //https://docs.google.com/document/d/1ETle6qQRcaNWAmpG2wz0oOpFKSF_bcTmYMQvtTGI8ns/edit

                    /*
                    8 bits of link ID
                    48 bits of timestamp
                    48 bits of signature
                    */

                    // signature = sha256_48(secret_key + header + payload + CRC + link-ID + timestamp)

                    var timestamp = (ulong)((DateTime.UtcNow - new DateTime(2015, 1, 1)).TotalMilliseconds * 100);

                    if (timestamp == MAVlist[sisId, bilId].timestamp)
                        timestamp++;

                    MAVlist[sisId, bilId].timestamp = timestamp;

                    var timebytes = BitConverter.GetBytes(timestamp);

                    var sig = new byte[7]; // 13 includes the outgoing hash
                    sig[0] = MAVlist[sisId, bilId].sendlinkid;
                    Array.Copy(timebytes, 0, sig, 1, 6); // timestamp

                    //Console.WriteLine("gen linkid {0}, time {1} {2} {3} {4} {5} {6} {7}", sig[0], sig[1], sig[2], sig[3], sig[4], sig[5], sig[6], timestamp);

                    var signingKey = MAVlist[sisId, bilId].signingKey;

                    if (signingKey == null || signingKey.Length != 32) signingKey = new byte[32];

                    using (var signit = new SHA256Managed())
                    {
                        signit.TransformBlock(signingKey, 0, signingKey.Length, null, 0);
                        signit.TransformBlock(packet, 0, i, null, 0);
                        signit.TransformFinalBlock(sig, 0, sig.Length);
                        var ctx = signit.Hash;
                        // trim to 48
                        Array.Resize(ref ctx, 6);

                        foreach (var b in sig)
                        {
                            packet[i] = b;
                            i++;
                        }

                        foreach (var b in ctx)
                        {
                            packet[i] = b;
                            i++;
                        }
                    }
                }
            }

            if (BaseStream.IsOpen)
            {
                BaseStream.Yaz(packet, 0, i);
                _gonderilenNesneBaytlari.OnNext(i);
            }

            try
            {
                if (kayitDosyasi != null && kayitDosyasi.CanWrite)
                    lock (kayitDosyasi)
                    {
                        var datearray =
                            BitConverter.GetBytes(
                                (ulong)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000));
                        Array.Reverse(datearray);
                        kayitDosyasi.Write(datearray, 0, datearray.Length);
                        kayitDosyasi.Write(packet, 0, i);
                    }
            }
            catch
            {
            }
        }
    }

    public bool Yaz(string line)
    {
        lock (yazmaKilidi)
        {
            BaseStream.Yaz(line);
        }

        _gonderilenNesneBaytlari.OnNext(line.Length);
        return true;
    }

    public bool imzalamaKur(string userseed, byte[] key = null)
    {
        byte[] shauser;
        var clearkey = false;

        if (key == null)
        {
            clearkey = string.IsNullOrEmpty(userseed);

            // sha the user input string
            var signit = new SHA256Managed();
            shauser = signit.ComputeHash(Encoding.UTF8.GetBytes(userseed));
            Array.Resize(ref shauser, 32);
        }
        else
        {
            shauser = key;
            Array.Resize(ref shauser, 32);
        }

        var sign = new mavlink_setup_signing_t();
        if (!clearkey)
        {
            MAV.signingKey = shauser;
            sign.initial_timestamp =
                (ulong)((DateTime.UtcNow - new DateTime(2015, 1, 1)).TotalMilliseconds * 100);
            sign.secret_key = shauser;
        }
        else
        {
            MAV.signingKey = new byte[32];
            sign.initial_timestamp = 0;
            sign.secret_key = new byte[32];
        }

        sign.target_component = (byte)SeciliBilesenId;
        sign.target_system = (byte)SeciliSistemId;

        paketOlusturGonder((int)MAVLINK_MSG_ID.SETUP_SIGNING, sign, MAV.SistemId, MAV.BilesenId);

        paketOlusturGonder((int)MAVLINK_MSG_ID.SETUP_SIGNING, sign, MAV.SistemId, MAV.BilesenId);

        if (clearkey) return imzalamaKapat(SeciliSistemId, SeciliBilesenId);

        return imzalamaAc(SeciliSistemId, SeciliBilesenId);
    }

    public bool imzalamaAc(int sysid, int compid)
    {
        MAVlist[sysid, compid].signing = true;
        MAVlist[sysid, compid].mavlinkv2 = true;


        return MAVlist[sysid, compid].signing;
    }

    public bool imzalamaKapat(int sysid, int compid)
    {
        MAVlist[sysid, compid].signing = false;
        MAVlist[sysid, compid].mavlinkv2 = false;

        return MAVlist[sysid, compid].signing;
    }

    /// <summary>
    ///     set param on apm, used for param rename
    /// </summary>
    /// <param name="paramname"></param>
    /// <param name="value"></param>
    /// <returns></returns>
    public bool parametreGuncelle(string[] paramnames, double value)
    {
        foreach (var paramname in paramnames)
            if (parametreGuncelle(paramname, value))
                return true;

        return false;
    }

    public bool parametreGuncelle(string paramname, double value, bool force = false)
    {
        return parametreGuncelle((byte)SeciliSistemId, (byte)SeciliBilesenId, paramname, value, force);
    }

    /// <summary>
    ///     Set parameter on apm
    /// </summary>
    /// <param name="paramname">name as a string</param>
    /// <param name="value"></param>
    public bool parametreGuncelle(byte sysid, byte compid, string paramname, double value, bool force = false)
    {
        if (!MAVlist[sysid, compid].param.ContainsKey(paramname))
        {
            log.Warn("Trying to set Param that doesnt exist " + paramname + "=" + value);
            return false;
        }

        if (MAVlist[sysid, compid].param[paramname].Value == value && !force)
        {
            log.Warn("setParam " + paramname + " not modified as same");
            return true;
        }

        giveComport = true;

        // param type is set here, however it is always sent over the air as a float 100int = 100f.
        var req = new mavlink_param_set_t
        {
            target_system = sysid,
            target_component = compid,
            param_type = (byte)MAVlist[sysid, compid].param_types[paramname]
        };

        var temp = Encoding.ASCII.GetBytes(paramname);

        Array.Resize(ref temp, 16);
        req.param_id = temp;
        if (MAVlist[sysid, compid].apname == MAV_AUTOPILOT.ARDUPILOTMEGA)
            req.param_value = new MAVLinkParam(paramname, value, MAV_PARAM_TYPE.REAL32).float_value;
        else
            req.param_value = new MAVLinkParam(paramname, value,
                MAVlist[sysid, compid].param_types[paramname]).float_value;

        var currentparamcount = MAVlist[sysid, compid].param.Count;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_SET, req, sysid, compid);

        log.InfoFormat("setParam '{0}' = '{1}' sysid {2} compid {3}", paramname, value, sysid,
            compid);

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setParam Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_SET, req, sysid, compid);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setParam " + paramname);
            }

            var buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.PARAM_VALUE && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var par = buffer.ToStructure<mavlink_param_value_t>();

                    var st = Encoding.ASCII.GetString(par.param_id);

                    var pos = st.IndexOf('\0');

                    if (pos != -1) st = st.Substring(0, pos);

                    if (st != paramname)
                    {
                        log.InfoFormat("MAVLINK bad param response - {0} vs {1}", paramname, st);
                        continue;
                    }

                    if (MAVlist[sysid, compid].apname == MAV_AUTOPILOT.ARDUPILOTMEGA)
                    {
                        var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                        MAVlist[sysid, compid].param[st] = new MAVLinkParam(st,
                            BitConverter.GetBytes(par.param_value), MAV_PARAM_TYPE.REAL32,
                            (MAV_PARAM_TYPE)par.param_type);
                    }
                    else
                    {
                        var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                        MAVlist[sysid, compid].param[st] = new MAVLinkParam(st,
                            BitConverter.GetBytes(par.param_value), (MAV_PARAM_TYPE)par.param_type,
                            (MAV_PARAM_TYPE)par.param_type);
                    }

                    log.Info("setParam gotback " + st + " : " + MAVlist[sysid, compid].param[st]);

                    lastparamset = DateTime.Now;

                    giveComport = false;

                    //System.Threading.Thread.Sleep(100);//(int)(8.5 * 5)); // 8.5ms per byte
                    // check if enabeling this param has added subparams, queue on gui thread
                    if (currentparamcount < par.param_count)
                    {
                        /*
                        MainV2.instance.BeginInvoke((Action) delegate
                        {
                            Loading.ShowLoading(String.Format(Strings.ParamRefreshRequired, currentparamcount,
                                par.param_count));
                        });
                        */
                    }

                    return true;
                }
        }
    }


    public void parametreListesiniGetir()
    {
        log.InfoFormat("getParamList {0} {1}", SeciliSistemId, SeciliBilesenId);

        parametreListesiniGetir(MAV.SistemId, MAV.BilesenId);

        ParamListChanged?.Invoke(this, null);
    }


    /// <summary>
    ///     Parametre listesini ArduPilot Mega'dan al
    /// </summary>
    /// <returns></returns>
    public Dictionary<string, double> parametreListesiniGetir(byte sysid, byte compid)
    {
        giveComport = true;
        var indexsreceived = new List<int>();

        // yeni liste oluşturun, böylece iptal edilirse eski listeyi kullanırız
        var newparamlist = new MAVLinkParamList();

        var param_total = 1;

        var req = new mavlink_param_request_list_t();
        req.target_system = sysid;
        req.target_component = compid;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_REQUEST_LIST, req);

        var start = DateTime.Now;
        var restart = DateTime.Now;

        var lastmessage = DateTime.MinValue;


        var packets = 0;
        var retry = 0;
        var onebyone = false;
        var lastonebyone = DateTime.MinValue;

        do
        {
            // Geçerli paketler arasında 4 saniye
            if (!(start.AddMilliseconds(4000) > DateTime.Now))
            {
                // toplamın %75'inden azına sahipsek tam liste çekmeyi kullanın
                if (retry < 6 && indexsreceived.Count < param_total / 4 * 3)
                {
                    retry++;
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_REQUEST_LIST, req);
                    start = DateTime.Now;
                    continue;
                }

                onebyone = true;

                if (lastonebyone.AddMilliseconds(600) < DateTime.Now)
                {
                    log.Info("Parametreler alınıyor - " + param_total + " üzerinden " + indexsreceived.Count +
                             " tane alındı");

                    var queued = 0;


                    // tek tek parametreleri almak denenir
                    for (short i = 0; i <= param_total - 1; i++)
                        if (!indexsreceived.Contains(i))
                            // bu parametreleri al döngüsünden çıkmayı önle
                            try
                            {
                                queued++;

                                var req2 = new mavlink_param_request_read_t();
                                req2.target_system = sysid;
                                req2.target_component = compid;
                                req2.param_index = i;
                                req2.param_id = new byte[] { 0x0 };

                                Array.Resize(ref req2.param_id, 16);

                                paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_REQUEST_READ, req2);

                                if (queued >= 10)
                                {
                                    lastonebyone = DateTime.Now;
                                    break;
                                }
                            }
                            catch (Exception excp)
                            {
                                log.Info("GetParam Failed index: " + i + " " + excp);
                                throw excp;
                            }
                }
            }


            var buffer = paketOku();

            if (buffer.Length > 5)
            {
                packets++;

                if (buffer.msgid == (byte)MAVLINK_MSG_ID.PARAM_VALUE && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    restart = DateTime.Now;

                    // eğer tek tek yapmıyorsak başlangıç zamanını güncelle
                    if (!onebyone)
                        start = DateTime.Now;

                    var par = buffer.ToStructure<mavlink_param_value_t>();

                    // yeni hedef belirle
                    param_total = par.param_count;
                    newparamlist.TotalReported = param_total;

                    var paramID = Encoding.ASCII.GetString(par.param_id);

                    var pos = paramID.IndexOf('\0');
                    if (pos != -1) paramID = paramID.Substring(0, pos);

                    // önceden aldık mı diye kontrol ediyoruz
                    if (indexsreceived.Contains(par.param_index))
                    {
                        log.Info("Already got " + par.param_index + " '" + paramID + "' " +
                                 indexsreceived.Count * 100 / param_total);
                        continue;
                    }


                    log.Info(DateTime.Now.Millisecond + " got param " + par.param_index + " of " +
                             par.param_count + " name: " + paramID);


                    if (MAVlist[sysid, compid].apname == MAV_AUTOPILOT.ARDUPILOTMEGA)
                    {
                        var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                        newparamlist[paramID] = new MAVLinkParam(paramID, BitConverter.GetBytes(par.param_value),
                            MAV_PARAM_TYPE.REAL32, (MAV_PARAM_TYPE)par.param_type);
                    }
                    else
                    {
                        var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                        newparamlist[paramID] = new MAVLinkParam(paramID, BitConverter.GetBytes(par.param_value),
                            (MAV_PARAM_TYPE)par.param_type, (MAV_PARAM_TYPE)par.param_type);
                    }


                    // exclude index of 65535
                    if (par.param_index != 65535)
                        indexsreceived.Add(par.param_index);

                    MAVlist[sysid, compid].param_types[paramID] = (MAV_PARAM_TYPE)par.param_type;


                    // we hit the last param - lets escape eq total = 176 index = 0-175
                    if (par.param_index == param_total - 1)
                        start = DateTime.MinValue;
                }

                if (buffer.msgid == (byte)MAVLINK_MSG_ID.STATUSTEXT)
                {
                    var msg = buffer.ToStructure<mavlink_statustext_t>();

                    var logdata = Encoding.ASCII.GetString(msg.text);

                    var ind = logdata.IndexOf('\0');
                    if (ind != -1)
                        logdata = logdata.Substring(0, ind);

                    if (logdata.ToLower().Contains("copter") || logdata.ToLower().Contains("rover") ||
                        logdata.ToLower().Contains("plane"))
                        MAVlist[sysid, compid].VersionString = logdata;
                    else if (logdata.ToLower().Contains("nuttx"))
                        MAVlist[sysid, compid].SoftwareVersions = logdata;
                    else if (logdata.ToLower().Contains("px4v2"))
                        MAVlist[sysid, compid].SerialString = logdata;
                    else if (logdata.ToLower().Contains("frame")) MAVlist[sysid, compid].FrameString = logdata;
                }
            }


            if (!BaseStream.IsOpen)
            {
                var exp = new Exception("Not Connected");
                throw exp;
            }
        } while (indexsreceived.Count < param_total);

        if (indexsreceived.Count != param_total)
        {
            var exp = new Exception("Missing Params " + indexsreceived.Count + " vs " + param_total);
            throw exp;
        }

        giveComport = false;

        MAVlist[sysid, compid].param.Clear();
        MAVlist[sysid, compid].param.TotalReported = param_total;
        MAVlist[sysid, compid].param.AddRange(newparamlist);
        return MAVlist[sysid, compid].param;
    }

    public void getParamPoll()
    {
        // hepsini aldığını doğrula
        if (MAV.param.TotalReceived >= MAV.param.TotalReported) return;

        // eğer tüm parametreleri alamadığımız bir araca birincil olarak bağlıysak, onlar için yoklama yapınız
        var i = (short)(_parampoll % MAV.param.TotalReported);

        ParametreGetir("", i, false);

        _parampoll++;
    }

    public float ParametreGetir(string name)
    {
        return ParametreGetir(name, -1);
    }

    public float ParametreGetir(short index)
    {
        return ParametreGetir("", index);
    }

    public float ParametreGetir(string name = "", short index = -1, bool requireresponce = true)
    {
        return ParametreGetir(MAV.SistemId, MAV.BilesenId, name, index, requireresponce);
    }

    /// <summary>
    ///     Parametreleri dizine veya isme göre al
    /// </summary>
    /// <param name="index"></param>
    /// <param name="name"></param>
    /// <returns></returns>
    public float ParametreGetir(byte sysid, byte compid, string name = "", short index = -1, bool requireresponce = true)
    {
        if (name == "" && index == -1)
            return 0;

        log.Info("GetParam name: '" + name + "' or index: " + index + " " + sysid + ":" + compid);

        MAVLinkMessage buffer;

        var req = new mavlink_param_request_read_t();
        req.target_system = sysid;
        req.target_component = compid;
        req.param_index = index;
        req.param_id = new byte[] { 0x0 };
        if (index == -1) req.param_id = Encoding.ASCII.GetBytes(name);

        Array.Resize(ref req.param_id, 16);

        paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_REQUEST_READ, req, sysid, compid);

        if (!requireresponce) return 0f;

        giveComport = true;

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("GetParam Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.PARAM_REQUEST_READ, req, sysid, compid);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - GetParam");
            }

            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.PARAM_VALUE && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    giveComport = false;

                    var par = buffer.ToStructure<mavlink_param_value_t>();

                    var st = Encoding.ASCII.GetString(par.param_id);

                    var pos = st.IndexOf('\0');

                    if (pos != -1) st = st.Substring(0, pos);

                    // doğru id değilse
                    if (!(par.param_index == index || st == name))
                    {
                        log.ErrorFormat("Wrong Answer {0} - {1} - {2}    --- '{3}' vs '{4}'", par.param_index,
                            Encoding.ASCII.GetString(par.param_id), par.param_value,
                            Encoding.ASCII.GetString(req.param_id).TrimEnd(), st);
                        continue;
                    }

                    // tabloyu güncelle
                    if (MAVlist[sysid, compid].apname == MAV_AUTOPILOT.ARDUPILOTMEGA)
                    {
                        var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                        MAVlist[sysid, compid].param[st] = new MAVLinkParam(st,
                            BitConverter.GetBytes(par.param_value), MAV_PARAM_TYPE.REAL32,
                            (MAV_PARAM_TYPE)par.param_type);
                    }
                    else
                    {
                        var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                        MAVlist[sysid, compid].param[st] = new MAVLinkParam(st,
                            BitConverter.GetBytes(par.param_value), (MAV_PARAM_TYPE)par.param_type,
                            (MAV_PARAM_TYPE)par.param_type);
                    }

                    MAVlist[sysid, compid].param_types[st] = (MAV_PARAM_TYPE)par.param_type;

                    log.Info(DateTime.Now.Millisecond + " got param " + par.param_index + " of " +
                             par.param_count + " name: " + st);

                    return par.param_value;
                }
        }
    }

    /// <summary>
    ///     İstenen tüm veri paketlerini durdur.
    /// </summary>
    public void TumunuDurdur(bool forget)
    {
        var req = new mavlink_request_data_stream_t();
        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;

        req.req_message_rate = 10;
        req.start_stop = 0; // durdur
        req.req_stream_id = 0; // tümünü

        // kötü durumda hata yok
        try
        {
            paketOlusturGonder((byte)MAVLINK_MSG_ID.REQUEST_DATA_STREAM, req);
            Thread.Sleep(20);
            paketOlusturGonder((byte)MAVLINK_MSG_ID.REQUEST_DATA_STREAM, req);
            Thread.Sleep(20);
            paketOlusturGonder((byte)MAVLINK_MSG_ID.REQUEST_DATA_STREAM, req);
            log.Info("Stopall Done");
        }
        catch
        {
        }
    }

    public void WpAckGonder()
    {
        var req = new mavlink_mission_ack_t();
        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;
        req.type = 0;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ACK, req);
    }

    public bool SuankiWPDegistir(ushort index)
    {
        giveComport = true;
        MAVLinkMessage buffer;

        var req = new mavlink_mission_set_current_t();

        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;
        req.seq = index;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_SET_CURRENT, req);

        var start = DateTime.Now;
        var retrys = 5;

        while (true)
        {
            if (!(start.AddMilliseconds(2000) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWPCurrent Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_SET_CURRENT, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWPCurrent");
            }

            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_CURRENT && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    giveComport = false;
                    return true;
                }
        }
    }


    /// <summary>
    ///     Aracı yeniden başlat
    /// </summary>
    /// <param name="bootloadermode">bootloader modunda başlat </param>
    /// <param name="currentvehicle">seçili sistemid/bileşenid kullan ya da onu ara</param>
    /// <returns></returns>
    public bool OtopilotYenidenBaslat(bool bootloadermode = false, bool currentvehicle = true)
    {
        var param1 = 1;
        if (bootloadermode) param1 = 3;

        // mevcut seçili mav'ı yeniden başlat
        if (currentvehicle)
        {
            KomutGonder(MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN, param1, 0, 0, 0, 0, 0, 0);
            KomutGonder(MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN, 1, 0, 0, 0, 0, 0, 0);
        }
        else
        {
            // bilinmeyen iha nabız için ara
            var buffer = nabizAl();

            if (buffer.Length > 5)
            {
                var hb = buffer.ToStructure<mavlink_heartbeat_t>();

                mavlinkSurum = hb.mavlink_version;
                MAV.aptype = (MAV_TYPE)hb.type;
                MAV.apname = (MAV_AUTOPILOT)hb.autopilot;
                MAV.SistemId = buffer.sysid;
                MAV.BilesenId = buffer.compid;
            }

            // nabiz verisi geldiyse yeniden baslat
            if (MAV.SistemId != 0 && MAV.BilesenId != 0)
            {
                KomutGonder(MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN, param1, 0, 0, 0, 0, 0, 0);
                KomutGonder(MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN, 1, 0, 0, 0, 0, 0, 0);
            }
        }

        giveComport = false;
        return true;
    }

    /// <summary>
    ///     Aracı ARM eder
    /// </summary>
    /// <param name="armit">arm etmek için true, disarm etmek için false</param>
    /// <param name="force"> zorla arm etmek için true</param>
    /// <returns></returns>
    public bool ArmEt(bool armit, bool force = false)
    {
        return ArmEt(MAV.SistemId, MAV.BilesenId, armit, force);
    }

    public bool ArmEt(byte sysid, byte compid, bool armit, bool force = false)
    {
        try
        {
            const float magic_force_arm_value = 2989.0f;
            const float magic_force_disarm_value = 21196.0f;

            if (force)
            {
                if (armit)
                    return KomutGonder(sysid, compid, MAV_CMD.COMPONENT_ARM_DISARM, 1, magic_force_arm_value, 0, 0, 0,
                        0,
                        0);
                return KomutGonder(sysid, compid, MAV_CMD.COMPONENT_ARM_DISARM, 0, magic_force_disarm_value, 0, 0, 0,
                    0, 0);
            }

            if (armit)
                return KomutGonder(sysid, compid, MAV_CMD.COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0);
            return KomutGonder(sysid, compid, MAV_CMD.COMPONENT_ARM_DISARM, 0, magic_force_disarm_value, 0, 0, 0,
                0, 0);
        }
        catch (TimeoutException e)
        {
            log.Error("'arm' işlemi zaman aşımına uğradı", e);
            throw;
        }
        catch (Exception e)
        {
            log.Error("'arm' işlemi sırasında hata oluştu", e);
            throw;
        }

    }

    public bool InisiIptalEt()
    {
        return KomutGonder(MAV_CMD.DO_GO_AROUND, 0, 0, 0, 0, 0, 0, 0);
    }

    public bool MotorTestYap(int motor, MOTOR_TEST_THROTTLE_TYPE thr_type, int throttle, int timeout,
        int motorcount = 0)
    {
        return KomutGonder(MAV_CMD.DO_MOTOR_TEST, motor, (byte)thr_type,
            throttle, timeout, motorcount, 0, 0);
    }

    public bool KomutGonder(MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7,
        bool requireack = true)
    {
        return KomutGonder(MAV.SistemId, MAV.BilesenId, actionid, p1, p2, p3, p4, p5, p6, p7, requireack);
    }

    public bool KomutGonder(byte sysid, byte compid, MAV_CMD actionid, float p1, float p2, float p3, float p4,
        float p5, float p6, float p7, bool requireack = true, Action uicallback = null)
    {
        giveComport = true;
        MAVLinkMessage buffer;

        var req = new mavlink_command_long_t();

        req.target_system = sysid;
        req.target_component = compid;

        req.command = (ushort)actionid;

        req.param1 = p1;
        req.param2 = p2;
        req.param3 = p3;
        req.param4 = p4;
        req.param5 = p5;
        req.param6 = p6;
        req.param7 = p7;

        log.InfoFormat("doCommand cmd {0} {1} {2} {3} {4} {5} {6} {7}", actionid.ToString(), p1, p2, p3, p4, p5, p6,
            p7);

        paketOlusturGonder((byte)MAVLINK_MSG_ID.COMMAND_LONG, req, sysid, compid);

        if (!requireack)
        {
            giveComport = false;
            return true;
        }

        var GUI = DateTime.Now;

        var start = DateTime.Now;
        var retrys = 3;

        var timeout = 2000;

        // imu calib take a little while
        if (actionid == MAV_CMD.PREFLIGHT_CALIBRATION && p5 == 1)
        {
            // this is for advanced accel offsets, and blocks execution
            giveComport = false;
            return true;
        }

        if (actionid == MAV_CMD.PREFLIGHT_CALIBRATION && p6 == 1)
        {
            // compassmot
            // send again just incase
            paketOlusturGonder((byte)MAVLINK_MSG_ID.COMMAND_LONG, req, sysid, compid);
            giveComport = false;
            return true;
        }

        if (actionid == MAV_CMD.PREFLIGHT_CALIBRATION)
        {
            retrys = 1;
            timeout = 25000;
        }
        else if (actionid == MAV_CMD.FLASH_BOOTLOADER)
        {
            retrys = 1;
            timeout = 25000;
        }
        else if (actionid == MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN)
        {
            paketOlusturGonder((byte)MAVLINK_MSG_ID.COMMAND_LONG, req, sysid, compid);
            giveComport = false;
            return true;
        }
        else if (actionid == MAV_CMD.COMPONENT_ARM_DISARM)
        {
            // imu kalibrasyonundaysa ihtiyaç duyabileceğinden 10 saniye ver
            timeout = 10000;
        }
        else if (actionid == MAV_CMD.GET_HOME_POSITION)
        {
            giveComport = false;
            return true;
        }

        while (true)
        {
            if (DateTime.Now > GUI.AddMilliseconds(100))
            {
                GUI = DateTime.Now;

                uicallback?.Invoke();
            }

            if (!(start.AddMilliseconds(timeout) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("doCommand Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.COMMAND_LONG, req, sysid, compid);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - doCommand");
            }

            // Aracın ACK mesajını al
            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.COMMAND_ACK && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var ack = buffer.ToStructure<mavlink_command_ack_t>();

                    if (ack.command != req.command)
                    {
                        log.InfoFormat("doCommand cmd resp {0} - {1} - Commands dont match", (MAV_CMD)ack.command,
                            (MAV_RESULT)ack.result);
                        continue;
                    }

                    log.InfoFormat("doCommand cmd resp {0} - {1}", (MAV_CMD)ack.command, (MAV_RESULT)ack.result);

                    if (ack.result == (byte)MAV_RESULT.ACCEPTED)
                    {
                        giveComport = false;
                        return true;
                    }
                    else if (ack.result == (byte)MAV_RESULT.TEMPORARILY_REJECTED)
                    {
                        giveComport = false;
                        return false;
                    }
                    else if (ack.result == (byte)MAV_RESULT.DENIED)
                    {
                        giveComport = false;
                        return false;
                    }
                    else if (ack.result == (byte)MAV_RESULT.UNSUPPORTED)
                    {
                        giveComport = false;
                        return false;
                    }
                    else if (ack.result == (byte)MAV_RESULT.FAILED)
                    {
                        giveComport = false;
                        return false;
                    }

                    giveComport = false;
                    return false;
                }
        }
    }

    public void AlindiGonder()
    {
        var ack = new mavlink_command_ack_t();
        ack.command = (ushort)MAV_CMD.PREFLIGHT_CALIBRATION;
        ack.result = 0;

        // send twice
        paketOlusturGonder(MAVLINK_MSG_ID.COMMAND_ACK, ack);
        Thread.Sleep(20);
        paketOlusturGonder(MAVLINK_MSG_ID.COMMAND_ACK, ack);
    }

    public void SerialControlGonder(SERIAL_CONTROL_DEV port, ushort timeoutms, byte[] data, uint baudrate = 0,
        bool close = false)
    {
        var ctl = new mavlink_serial_control_t();

        ctl.baudrate = baudrate; // no change
        ctl.device = (byte)port;
        ctl.timeout = timeoutms;
        ctl.data = new byte[70];
        ctl.count = 0;
        if (close)
            ctl.flags = 0;
        else
            ctl.flags = (byte)SERIAL_CONTROL_FLAG.EXCLUSIVE; // | SERIAL_CONTROL_FLAG.MULTI);

        if (data != null && data.Length != 0)
        {
            var packets = data.Length / 70 + 1;
            var len = data.Length;
            while (len > 0)
            {
                if (packets == 1)
                    ctl.flags |= (byte)SERIAL_CONTROL_FLAG.RESPOND;

                var n = (byte)Math.Min(70, len);

                ctl.count = n;
                Array.Copy(data, data.Length - len, ctl.data, 0, n);

                // portu spamlamayı engelle
                Thread.Sleep(10);

                paketOlusturGonder((byte)MAVLINK_MSG_ID.SERIAL_CONTROL, ctl);

                len -= n;
                packets--;
            }
        }
        else
        {
            if (!close)
                ctl.flags |= (byte)SERIAL_CONTROL_FLAG.RESPOND | (byte)SERIAL_CONTROL_FLAG.MULTI;

            paketOlusturGonder((byte)MAVLINK_MSG_ID.SERIAL_CONTROL, ctl);
        }
    }

    public void KumandaBaskilamaGonder(byte sysid, byte compid, ushort rc1, ushort rc2, ushort rc3, ushort rc4, ushort rc5,
        ushort rc6, ushort rc7, ushort rc8)
    {
        var rc = new mavlink_rc_channels_override_t();

        rc.target_component = compid;
        rc.target_system = sysid;

        rc.chan1_raw = rc1;
        rc.chan2_raw = rc2;
        rc.chan3_raw = rc3;
        rc.chan4_raw = rc4;
        rc.chan5_raw = rc5;
        rc.chan6_raw = rc6;
        rc.chan7_raw = rc7;
        rc.chan8_raw = rc8;

        paketGonder(rc, rc.target_system, rc.target_component);
    }

    public void SendManualControl(byte sysid, byte compid, short x, short y, short z, short r, ushort buttons)
    {
        var mc = new mavlink_manual_control_t();

        mc.target = sysid;

        mc.x = x;
        mc.y = y;
        mc.z = z;
        mc.r = r;

        mc.buttons = buttons;

        paketGonder(mc, sysid, compid);
    }

    public void VeriAkisiIste(MAV_DATA_STREAM id, byte hzrate, int sysid = -1, int compid = -1)
    {
        VeriAkisiIste(id, (int)hzrate, sysid, compid);
    }

    public void VeriAkisiIste(MAV_DATA_STREAM id, int hzrate, int sysid = -1, int compid = -1)
    {
        if (sysid == -1)
            sysid = SeciliSistemId;

        if (compid == -1)
            compid = SeciliBilesenId;

        if (hzrate == -1)
            return;

        double pps = 0;

        switch (id)
        {
            case MAV_DATA_STREAM.ALL:

                break;
            case MAV_DATA_STREAM.EXTENDED_STATUS:
                if (MAVlist[sysid, compid].packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.SYS_STATUS))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.SYS_STATUS] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.SYS_STATUS];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.EXTRA1:
                if (MAVlist[sysid, compid].packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.ATTITUDE))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.ATTITUDE] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.ATTITUDE];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.EXTRA2:
                if (MAVlist[sysid, compid].packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.VFR_HUD))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.VFR_HUD] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.VFR_HUD];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.EXTRA3:
                if (MAVlist[sysid, compid].packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.AHRS))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.AHRS] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.AHRS];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.POSITION:
                if (MAVlist[sysid, compid]
                    .packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.GLOBAL_POSITION_INT))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.GLOBAL_POSITION_INT] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.GLOBAL_POSITION_INT];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.RAW_CONTROLLER:
                if (MAVlist[sysid, compid]
                    .packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.RC_CHANNELS_SCALED))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.RC_CHANNELS_SCALED] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.RC_CHANNELS_SCALED];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.RAW_SENSORS:
                if (MAVlist[sysid, compid].packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.RAW_IMU))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.RAW_IMU] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.RAW_IMU];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
            case MAV_DATA_STREAM.RC_CHANNELS:
                if (MAVlist[sysid, compid].packetspersecondbuild.ContainsKey((byte)MAVLINK_MSG_ID.RC_CHANNELS_RAW))
                {
                    if (MAVlist[sysid, compid].packetspersecondbuild[(byte)MAVLINK_MSG_ID.RC_CHANNELS_RAW] <
                        DateTime.Now.AddSeconds(-2))
                        break;
                    pps = MAVlist[sysid, compid].packetspersecond[(byte)MAVLINK_MSG_ID.RC_CHANNELS_RAW];
                }

                if (hzratecheck(pps, hzrate)) return;

                break;
        }

        if (pps == 0 && hzrate == 0) return;

        log.InfoFormat("Request stream {0} at {1} hz for {2}:{3}",
            Enum.Parse(typeof(MAV_DATA_STREAM), id.ToString()), hzrate, sysid, compid);
        veriAkisiGetir((byte)sysid, (byte)compid, id, hzrate);
    }


    #region Rota Metotları

    public ushort getWPCount(MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        return getWPCountAsync(MAV.SistemId, MAV.BilesenId, type).AwaitSync();
    }

    public ushort getWPCount(byte sysid, byte compid, MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        return getWPCountAsync(sysid, compid, type).AwaitSync();
    }

    /// <summary>
    /// Returns WP count
    /// </summary>
    /// <returns></returns>
    public async Task<ushort> getWPCountAsync(byte sysid, byte compid,
        MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        giveComport = true;
        MAVLinkMessage buffer;
        mavlink_mission_request_list_t req = new mavlink_mission_request_list_t
        {
            target_system = sysid,
            target_component = compid,
            mission_type = (byte)type
        };

        // request list
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_LIST, req);

        DateTime start = DateTime.Now;
        int retrys = 6;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("getWPCount Retry " + retrys + " - giv com " + giveComport);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_LIST, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                throw new TimeoutException("Timeout on read - getWPCount");
            }

            buffer = await readPacketAsync().ConfigureAwait(false);
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_COUNT && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var count = buffer.ToStructure<mavlink_mission_count_t>();
                    // check this gcs sent it
                    if (count.target_system != YkiSistemId)
                        continue;

                    log.Info("wpcount: " + count.count);
                    giveComport = false;
                    return count.count; // should be ushort, but apm has limited wp count < byte
                }
            }
        }
    }

    public Locationwp getWP(ushort index, MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        return getWP(MAV.SistemId, MAV.BilesenId, index, type);
    }

    public Locationwp getWP(byte sysid, byte compid, ushort index,
        MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        return getWPAsync(sysid, compid, index, type).AwaitSync();
    }

    /// <summary>
    /// Gets specfied WP
    /// </summary>
    /// <param name="index"></param>
    /// <returns>WP</returns>
    public async Task<Locationwp> getWPAsync(byte sysid, byte compid, ushort index,
        MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        while (giveComport == true)
            Thread.Sleep(10);

        bool use_int = (MAVlist[sysid, compid].cs.capabilities & (uint)MAV_PROTOCOL_CAPABILITY.MISSION_INT) > 0;

        object req;

        if (use_int)
        {
            mavlink_mission_request_int_t reqi = new mavlink_mission_request_int_t
            {
                target_system = sysid,
                target_component = compid,
                mission_type = (byte)type,
                seq = index
            };

            // request
            paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT, reqi);

            req = reqi;
        }
        else
        {
            mavlink_mission_request_t reqf = new mavlink_mission_request_t
            {
                target_system = sysid,
                target_component = compid,
                mission_type = (byte)type,
                seq = index
            };

            // request
            paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, reqf);

            req = reqf;
        }

        giveComport = true;
        Locationwp loc = new Locationwp();

        DateTime start = DateTime.Now;
        int retrys = 5;

        while (true)
        {
            if (!(start.AddMilliseconds(2500) > DateTime.Now)) // apm times out after 5000ms
            {
                if (retrys > 0)
                {
                    log.Info("getWP Retry " + retrys);
                    if (use_int)
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT, req);
                    else
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - getWP");
            }

            //Console.WriteLine("getwp read " + DateTime.Now.Millisecond);
            MAVLinkMessage buffer = await readPacketAsync().ConfigureAwait(false);
            //Console.WriteLine("getwp readend " + DateTime.Now.Millisecond);
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ITEM && buffer.sysid == sysid &&
                    buffer.compid == compid)
                {
                    //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);

                    var wp = buffer.ToStructure<mavlink_mission_item_t>();
                    if (wp.target_system != YkiSistemId)
                        continue;

                    // received a packet, but not what we requested
                    if (index != wp.seq)
                    {
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, req);
                        continue;
                    }

                    loc.frame = wp.frame;
                    loc.id = (ushort)(wp.command);
                    loc.p1 = (wp.param1);
                    loc.p2 = (wp.param2);
                    loc.p3 = (wp.param3);
                    loc.p4 = (wp.param4);

                    loc.alt = ((wp.z));
                    loc.lat = ((wp.x));
                    loc.lng = ((wp.y));

                    log.InfoFormat("getWP {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat, loc.lng,
                        loc.frame);

                    break;
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ITEM_INT && buffer.sysid == sysid &&
                         buffer.compid == compid)
                {
                    //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);

                    var wp = buffer.ToStructure<mavlink_mission_item_int_t>();
                    // check this gcs sent it
                    if (wp.target_system != YkiSistemId)
                        continue;

                    // received a packet, but not what we requested
                    if (index != wp.seq)
                    {
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT, req);
                        continue;
                    }

                    loc.frame = wp.frame;
                    loc.id = (ushort)(wp.command);
                    loc.p1 = (wp.param1);
                    loc.p2 = (wp.param2);
                    loc.p3 = (wp.param3);
                    loc.p4 = (wp.param4);

                    loc.alt = ((wp.z));
                    loc.lat = ((wp.x / 1.0e7));
                    loc.lng = ((wp.y / 1.0e7));


                    log.InfoFormat("getWPint {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat,
                        loc.lng,
                        loc.frame);

                    break;
                }
                else
                {
                    //log.Info(DateTime.Now + " PC getwp " + buffer.msgid);
                }
            }
        }

        giveComport = false;
        return loc;
    }


    public void setWPACK(MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        setWPACK(MAV.SistemId, MAV.BilesenId, type);
    }

    public void setWPACK(byte sysid, byte compid, MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        mavlink_mission_ack_t req = new mavlink_mission_ack_t
        {
            target_system = sysid,
            target_component = compid,
            type = 0,
            mission_type = (byte)type
        };

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ACK, req);
    }

    public void setWPTotal(ushort wp_total, MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        setWPTotalAsync(MAV.SistemId, MAV.BilesenId, wp_total, type).ConfigureAwait(false).GetAwaiter().GetResult();
        return;
    }

    /// <summary>
    /// Sets wp total count
    /// </summary>
    /// <param name="wp_total"></param>
    public async Task setWPTotalAsync(byte sysid, byte compid, ushort wp_total,
        MAVLink.MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        giveComport = true;
        mavlink_mission_count_t req = new mavlink_mission_count_t
        {
            target_system = sysid,
            target_component = compid,
            count = wp_total,
            mission_type = (byte)type
        };

        log.Info("setWPTotal req MISSION_COUNT " + req.ToString());
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_COUNT, req);

        DateTime start = DateTime.Now;
        int retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWPTotal Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_COUNT, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWPTotal");
            }

            MAVLinkMessage buffer = await readPacketAsync().ConfigureAwait(false);
            if (buffer.Length > 9)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var request = buffer.ToStructure<mavlink_mission_request_t>();
                    // check this gcs sent it
                    if (request.target_system != YkiSistemId)
                        continue;

                    if (request.seq == 0 || request.seq == 1)
                    {
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.MISSION)
                        {
                            if (MAV.param["WP_TOTAL"] != null)
                                MAV.param["WP_TOTAL"].Value = wp_total - 1;
                            if (MAV.param["CMD_TOTAL"] != null)
                                MAV.param["CMD_TOTAL"].Value = wp_total - 1;
                            if (MAV.param["MIS_TOTAL"] != null)
                                MAV.param["MIS_TOTAL"].Value = wp_total - 1;
                            MAVlist[req.target_system, req.target_component].wps.Clear();
                        }

                        if (req.mission_type == (byte)MAV_MISSION_TYPE.FENCE)
                            MAVlist[req.target_system, req.target_component].fencepoints.Clear();
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.RALLY)
                            MAVlist[req.target_system, req.target_component].rallypoints.Clear();

                        giveComport = false;
                        return;
                    }
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT &&
                         buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var request = buffer.ToStructure<mavlink_mission_request_int_t>();
                    // check this gcs sent it
                    if (request.target_system != YkiSistemId)
                        continue;

                    if (request.seq == 0 || request.seq == 1)
                    {
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.MISSION)
                        {
                            if (MAV.param["WP_TOTAL"] != null)
                                MAV.param["WP_TOTAL"].Value = wp_total - 1;
                            if (MAV.param["CMD_TOTAL"] != null)
                                MAV.param["CMD_TOTAL"].Value = wp_total - 1;
                            if (MAV.param["MIS_TOTAL"] != null)
                                MAV.param["MIS_TOTAL"].Value = wp_total - 1;
                            MAVlist[req.target_system, req.target_component].wps.Clear();
                        }

                        if (req.mission_type == (byte)MAV_MISSION_TYPE.FENCE)
                            MAVlist[req.target_system, req.target_component].fencepoints.Clear();
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.RALLY)
                            MAVlist[req.target_system, req.target_component].rallypoints.Clear();

                        giveComport = false;
                        return;
                    }
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ACK && buffer.sysid == req.target_system &&
                         buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_ack_t>();
                    log.Info("setWPTotal ACK 47 : " + buffer.msgid + " ans " +
                             Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                    // check this gcs sent it
                    if (ans.target_system != YkiSistemId)
                        continue;

                    giveComport = false;
                    return; // (MAV_MISSION_RESULT)ans.type;
                }
                else
                {
                    //Console.WriteLine(DateTime.Now + " PC getwp " + buffer.msgid);
                }
            }
        }
    }


    /// <summary>
    /// Save wp to eeprom
    /// </summary>
    /// <param name="loc">location struct</param>
    /// <param name="index">wp no</param>
    /// <param name="frame">global or relative</param>
    /// <param name="current">0 = no , 2 = guided mode</param>
    public async Task<MAV_MISSION_RESULT> setWPAsync(byte sysid, byte compid, Locationwp loc, ushort index,
        MAV_FRAME frame, byte current = 0,
        byte autocontinue = 1, bool use_int = false, MAV_MISSION_TYPE mission_type = MAV_MISSION_TYPE.MISSION)
    {
        if (use_int)
        {
            mavlink_mission_item_int_t req = new mavlink_mission_item_int_t
            {
                target_system = sysid,
                target_component = compid,
                command = loc.id,
                current = current,
                autocontinue = autocontinue,
                mission_type = (byte)mission_type,
                frame = (byte)frame
            };


            req.y = (int)(loc.lng * 1.0e7);
            req.x = (int)(loc.lat * 1.0e7);

            req.z = (float)(loc.alt);

            req.param1 = loc.p1;
            req.param2 = loc.p2;
            req.param3 = loc.p3;
            req.param4 = loc.p4;

            req.seq = index;

            return await setWPAsync(req).ConfigureAwait(false);
        }
        else
        {
            mavlink_mission_item_t req = new mavlink_mission_item_t
            {
                target_system = sysid,
                target_component = compid,
                command = loc.id,
                current = current,
                autocontinue = autocontinue,
                mission_type = (byte)mission_type,
                frame = (byte)frame,
                y = (float)(loc.lng),
                x = (float)(loc.lat),
                z = (float)(loc.alt),
                param1 = loc.p1,
                param2 = loc.p2,
                param3 = loc.p3,
                param4 = loc.p4,
                seq = index
            };

            return await setWPAsync(req).ConfigureAwait(false);
        }
    }


    public async Task<MAV_MISSION_RESULT> setWPAsync(mavlink_mission_item_t req)
    {
        giveComport = true;

        ushort index = req.seq;

        log.InfoFormat("setWP {7}:{8} {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command,
            req.param1,
            req.x, req.y, req.z, index, req.target_system, req.target_component);

        // request
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM, req);

        DateTime start = DateTime.Now;
        int retrys = 10;

        while (true)
        {
            if (!(start.AddMilliseconds(450) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWP Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM, req);

                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWP");
            }

            MAVLinkMessage buffer = await readPacketAsync().ConfigureAwait(false);
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ACK && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_ack_t>();
                    log.Info("set wp " + index + " ACK 47 : " + buffer.msgid + " ans " +
                             Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                    // check this gcs sent it
                    if (ans.target_system != YkiSistemId)
                        continue;

                    if (req.current == 2)
                    {
                        MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                    }
                    else if (req.current == 3)
                    {
                    }
                    else
                    {
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.MISSION)
                            MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.FENCE)
                            MAVlist[req.target_system, req.target_component].fencepoints[req.seq] =
                                (Locationwp)req;
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.RALLY)
                            MAVlist[req.target_system, req.target_component].rallypoints[req.seq] =
                                (Locationwp)req;
                    }

                    //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                    {
                        giveComport = false;
                        return (MAV_MISSION_RESULT)ans.type;
                    }
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST &&
                         buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_t>();

                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (ans.seq == (index + 1))
                    {
                        log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer.msgid);
                        giveComport = false;

                        if (req.current == 2)
                        {
                            MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                        }
                        else if (req.current == 3)
                        {
                        }
                        else
                        {
                            if (req.mission_type == (byte)MAV_MISSION_TYPE.MISSION)
                                MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                            if (req.mission_type == (byte)MAV_MISSION_TYPE.FENCE)
                                MAVlist[req.target_system, req.target_component].fencepoints[req.seq] =
                                    (Locationwp)req;
                            if (req.mission_type == (byte)MAV_MISSION_TYPE.RALLY)
                                MAVlist[req.target_system, req.target_component].rallypoints[req.seq] =
                                    (Locationwp)req;
                        }

                        //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                        {
                            giveComport = false;
                            return MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
                        }
                    }
                    else
                    {
                        log.InfoFormat(
                            "set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " +
                            buffer.msgid +
                            " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                        // resend point now
                        start = DateTime.MinValue;
                    }
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT &&
                         buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_int_t>();

                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (ans.seq == (index + 1))
                    {
                        log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer.msgid);
                        giveComport = false;

                        if (req.current == 2)
                        {
                            MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                        }
                        else if (req.current == 3)
                        {
                        }
                        else
                        {
                            MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                        }

                        //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                        {
                            giveComport = false;
                            return MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
                        }
                    }
                    else
                    {
                        log.InfoFormat(
                            "set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " +
                            buffer.msgid +
                            " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                        // resend point now
                        start = DateTime.MinValue;
                    }
                }
                else
                {
                    //Console.WriteLine(DateTime.Now + " PC setwp " + buffer.msgid);
                }
            }
        }

        // return MAV_MISSION_RESULT.MAV_MISSION_INVALID;
    }

    public async Task<MAV_MISSION_RESULT> setWPAsync(mavlink_mission_item_int_t req)
    {
        giveComport = true;

        ushort index = req.seq;

        log.Info("setWPint req MISSION_ITEM_INT " + req.ToString());
        //log.InfoFormat("setWPint {7}:{8} {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command, req.param1,
        //  req.x / 1.0e7, req.y /1.0e7 , req.z, index, req.target_system, req.target_component);

        // request
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM_INT, req);

        DateTime start = DateTime.Now;
        int retrys = 10;

        while (true)
        {
            if (!(start.AddMilliseconds(450) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWP Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM_INT, req);

                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWP");
            }

            MAVLinkMessage buffer = await readPacketAsync().ConfigureAwait(false);
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ACK && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_ack_t>();
                    log.Info("setWPint resp MISSION_ACK " + buffer.ToString());
                    //log.Info("set wp " + index + " ACK 47 : " + buffer.msgid + " ans " +
                    //       Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                    // check this gcs sent it
                    if (ans.target_system != YkiSistemId)
                        continue;

                    if (req.current == 2)
                    {
                        MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                    }
                    else if (req.current == 3)
                    {
                    }
                    else
                    {
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.MISSION)
                            MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.FENCE)
                            MAVlist[req.target_system, req.target_component].fencepoints[req.seq] =
                                (Locationwp)req;
                        if (req.mission_type == (byte)MAV_MISSION_TYPE.RALLY)
                            MAVlist[req.target_system, req.target_component].rallypoints[req.seq] =
                                (Locationwp)req;
                    }

                    //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                    {
                        giveComport = false;
                        return (MAV_MISSION_RESULT)ans.type;
                    }
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST &&
                         buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_t>();
                    if (ans.target_system != YkiSistemId)
                        continue;

                    if (ans.seq == (index + 1))
                    {
                        log.Info("setWPint resp MISSION_REQUEST" + buffer.ToString());
                        //log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer.msgid);

                        if (req.current == 2)
                        {
                            MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                        }
                        else if (req.current == 3)
                        {
                        }
                        else
                        {
                            if (req.mission_type == (byte)MAV_MISSION_TYPE.MISSION)
                                MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                            if (req.mission_type == (byte)MAV_MISSION_TYPE.FENCE)
                                MAVlist[req.target_system, req.target_component].fencepoints[req.seq] =
                                    (Locationwp)req;
                            if (req.mission_type == (byte)MAV_MISSION_TYPE.RALLY)
                                MAVlist[req.target_system, req.target_component].rallypoints[req.seq] =
                                    (Locationwp)req;
                        }

                        //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                        {
                            giveComport = false;
                            return MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
                        }
                    }
                    else
                    {
                        log.InfoFormat(
                            "set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " +
                            buffer.msgid +
                            " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                        // resend point now
                        start = DateTime.MinValue;
                    }
                }
                else
                {
                    //Console.WriteLine(DateTime.Now + " PC setwp " + buffer.msgid);
                }
            }
        }

        // return MAV_MISSION_RESULT.MAV_MISSION_INVALID;
    }


    public async Task<int> getRequestedWPNoAsync(byte sysid, byte compid)
    {
        giveComport = true;
        DateTime start = DateTime.Now;

        while (true)
        {
            if (!(start.AddMilliseconds(1500) > DateTime.Now))
            {
                giveComport = false;
                throw new TimeoutException("Timeout on read - getRequestedWPNo");
            }

            MAVLinkMessage buffer = await readPacketAsync().ConfigureAwait(false);
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST && buffer.sysid == sysid &&
                    buffer.compid == compid)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_t>();

                    log.InfoFormat("getRequestedWPNo seq {0} ts {1} tc {2}", ans.seq, ans.target_system,
                        ans.target_component);

                    giveComport = false;

                    return ans.seq;
                }
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT && buffer.sysid == sysid &&
                    buffer.compid == compid)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_int_t>();

                    log.InfoFormat("getRequestedWPNo INT seq {0} ts {1} tc {2}", ans.seq, ans.target_system,
                        ans.target_component);

                    giveComport = false;

                    return ans.seq;
                }
            }
        }
    }


    /// <summary>
    /// Set start and finish for partial wp upload.
    /// </summary>
    /// <param name="startwp"></param>
    /// <param name="endwp"></param>
    public async Task setWPPartialUpdateAsync(byte sysid, byte compid, ushort startwp, ushort endwp,
        MAV_MISSION_TYPE type = MAV_MISSION_TYPE.MISSION)
    {
        mavlink_mission_write_partial_list_t req = new mavlink_mission_write_partial_list_t
        {
            target_system = sysid,
            target_component = compid,
            start_index = (short)startwp,
            end_index = (short)endwp,
            mission_type = (byte)type
        };

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_WRITE_PARTIAL_LIST, req);
    }

    #endregion
    
    public Locationwp HomeKonumuGetir()
    {
        return getHomePositionAsync((byte)SeciliSistemId, (byte)SeciliBilesenId).AwaitSync();
    }

    public async Task<Locationwp> getHomePositionAsync(byte sysid, byte compid)
    {
        Locationwp? ans = null;

        var sub1 = PaketTuruneAboneOl(MAVLINK_MSG_ID.HOME_POSITION, buffer =>
        {
            if (buffer.msgid == (byte)MAVLINK_MSG_ID.HOME_POSITION)
            {
                var home = buffer.ToStructure<mavlink_home_position_t>();

                ans = new Locationwp().Set(home.latitude / 1.0e7, home.longitude / 1.0e7, home.altitude / 1000.0,
                    (byte)MAV_CMD.WAYPOINT);
            }

            return true;
        });

        try
        {
            KomutGonder(sysid, compid, MAV_CMD.GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, false);

            giveComport = true;
            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (ans != null)
                    return ans.Value;

                if (!(start.AddMilliseconds(700) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("getHomePosition Retry " + retrys + " - giv com " + giveComport);
                        KomutGonder(sysid, compid, MAV_CMD.GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, false);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }

                    throw new TimeoutException("Timeout on read - getHomePosition");
                }

                await readPacketAsync().ConfigureAwait(false);
            }
        }
        finally
        {
            giveComport = false;
            PaketTurunuAboneliktenCik(sub1);
        }
    }

    public Locationwp WpGetir(ushort index)
    {
        return WpGetir(MAV.SistemId, MAV.BilesenId, index);
    }

    /// <summary>
    ///     Gets specfied WP
    /// </summary>
    /// <param name="index"></param>
    /// <returns>WP</returns>
    public Locationwp WpGetir(byte sysid, byte compid, ushort index)
    {
        while (giveComport)
            Thread.Sleep(100);

        var use_int = (MAVlist[sysid, compid].cs.capabilities & (uint)MAV_PROTOCOL_CAPABILITY.MISSION_INT) > 0;

        object req;

        if (use_int)
        {
            var reqi = new mavlink_mission_request_int_t();

            reqi.target_system = sysid;
            reqi.target_component = compid;

            reqi.seq = index;

            // request
            paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT, reqi);

            req = reqi;
        }
        else
        {
            var reqf = new mavlink_mission_request_t();

            reqf.target_system = sysid;
            reqf.target_component = compid;

            reqf.seq = index;

            // request
            paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, reqf);

            req = reqf;
        }

        giveComport = true;
        var loc = new Locationwp();

        var start = DateTime.Now;
        var retrys = 5;

        while (true)
        {
            if (!(start.AddMilliseconds(2500) > DateTime.Now)) // apm times out after 5000ms
            {
                if (retrys > 0)
                {
                    log.Info("getWP Retry " + retrys);
                    if (use_int)
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT, req);
                    else
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - getWP");
            }

            //Console.WriteLine("getwp read " + DateTime.Now.Millisecond);
            var buffer = paketOku();
            //Console.WriteLine("getwp readend " + DateTime.Now.Millisecond);
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ITEM && buffer.sysid == sysid &&
                    buffer.compid == compid)
                {
                    //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);

                    var wp = buffer.ToStructure<mavlink_mission_item_t>();
                    if (wp.target_system != YkiSistemId ||
                        wp.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    // received a packet, but not what we requested
                    if (index != wp.seq)
                    {
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, req);
                        continue;
                    }

                    loc.options = wp.frame;
                    loc.id = wp.command;
                    loc.p1 = wp.param1;
                    loc.p2 = wp.param2;
                    loc.p3 = wp.param3;
                    loc.p4 = wp.param4;

                    loc.alt = wp.z;
                    loc.lat = wp.x;
                    loc.lng = wp.y;

                    log.InfoFormat("getWP {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat, loc.lng,
                        loc.options);

                    break;
                }

                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ITEM_INT && buffer.sysid == sysid &&
                    buffer.compid == compid)
                {
                    //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);

                    var wp = buffer.ToStructure<mavlink_mission_item_int_t>();
                    // check this gcs sent it
                    if (wp.target_system != YkiSistemId ||
                        wp.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    // received a packet, but not what we requested
                    if (index != wp.seq)
                    {
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_REQUEST, req);
                        continue;
                    }

                    loc.options = wp.frame;
                    loc.id = wp.command;
                    loc.p1 = wp.param1;
                    loc.p2 = wp.param2;
                    loc.p3 = wp.param3;
                    loc.p4 = wp.param4;

                    loc.alt = wp.z;
                    loc.lat = wp.x / 1.0e7;
                    loc.lng = wp.y / 1.0e7;

                    if (loc.id == (ushort)MAV_CMD.DO_DIGICAM_CONTROL ||
                        loc.id == (ushort)MAV_CMD.DO_DIGICAM_CONFIGURE)
                        loc.lat = wp.x;

                    log.InfoFormat("getWPint {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat,
                        loc.lng,
                        loc.options);

                    break;
                }
                //log.Info(DateTime.Now + " PC getwp " + buffer.msgid);
            }
        }

        giveComport = false;
        return loc;
    }

    public object DebugPacket(MAVLinkMessage datin)
    {
        var text = "";
        return DebugPacket(datin, ref text, true);
    }

    public object DebugPacket(MAVLinkMessage datin, bool PrintToConsole)
    {
        var text = "";
        return DebugPacket(datin, ref text, PrintToConsole);
    }

    public object DebugPacket(MAVLinkMessage datin, ref string text)
    {
        return DebugPacket(datin, ref text, true);
    }

    /// <summary>
    ///     Print entire decoded packet to console
    /// </summary>
    /// <param name="datin">packet byte array</param>
    /// <returns>struct of data</returns>
    public object DebugPacket(MAVLinkMessage datin, ref string text, bool PrintToConsole, string delimeter = " ")
    {
        var textoutput = "";
        try
        {
            if (datin.Length > 5)
            {
                textoutput =
                    string.Format(
                        "{0,2:X}{8}{1,2:X}{8}{2,2:X}{8}{3,2:X}{8}{4,2:X}{8}{5,2:X}{8}{6,2:X}{8}{7,6:X}{8}",
                        datin.header,
                        datin.payloadlength, datin.incompat_flags, datin.compat_flags, datin.seq, datin.sysid,
                        datin.compid, datin.msgid, delimeter);

                var data = datin.data;

                var test = data.GetType();

                {
                    textoutput = textoutput + test.Name + delimeter;

                    foreach (var field in test.GetFields())
                    {
                        // field.Name has the field's name.

                        var fieldValue = field.GetValue(data); // Get value

                        if (field.FieldType.IsArray)
                        {
                            textoutput = textoutput + field.Name + delimeter;
                            if (fieldValue.GetType() == typeof(byte[]))
                                try
                                {
                                    var crap = (byte[])fieldValue;

                                    foreach (var fiel in crap)
                                        if (fiel == 0)
                                            break;
                                        else
                                            textoutput = textoutput + (char)fiel;
                                }
                                catch
                                {
                                }

                            if (fieldValue.GetType() == typeof(short[]))
                                try
                                {
                                    var crap = (short[])fieldValue;

                                    foreach (var fiel in crap)
                                        if (fiel == 0)
                                            break;
                                        else
                                            textoutput = textoutput + Convert.ToString(fiel, 16) + "|";
                                }
                                catch
                                {
                                }

                            textoutput = textoutput + delimeter;
                        }
                        else
                        {
                            textoutput = textoutput + field.Name + delimeter + fieldValue + delimeter;
                        }
                    }

                    var sig = "";
                    if (datin.sig != null)
                        sig = Convert.ToBase64String(datin.sig);

                    textoutput = textoutput + delimeter + "sig " + sig + delimeter + "Len" + delimeter +
                                 datin.Length + "\r\n";
                    if (PrintToConsole)
                        Console.Write(textoutput);

                    if (text != null)
                        text = textoutput;
                }

                return data;
            }
        }
        catch
        {
            textoutput = textoutput + "\r\n";
        }

        return null;
    }

    /// <summary>
    ///     Set start and finish for partial wp upload.
    /// </summary>
    /// <param name="startwp"></param>
    /// <param name="endwp"></param>
    public void setWPPartialUpdate(ushort startwp, ushort endwp)
    {
        var req = new mavlink_mission_write_partial_list_t();

        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;

        req.start_index = (short)startwp;
        req.end_index = (short)endwp;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_WRITE_PARTIAL_LIST, req);
    }

    /// <summary>
    ///     Sets wp total count
    /// </summary>
    /// <param name="wp_total"></param>
    public void setWPTotal(ushort wp_total)
    {
        giveComport = true;
        var req = new mavlink_mission_count_t();

        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId; // MSG_NAMES.MISSION_COUNT

        req.count = wp_total;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_COUNT, req);

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWPTotal Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_COUNT, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWPTotal");
            }

            var buffer = paketOku();
            if (buffer.Length > 9)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var request = buffer.ToStructure<mavlink_mission_request_t>();
                    // check this gcs sent it
                    if (request.target_system != YkiSistemId ||
                        request.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (request.seq == 0)
                    {
                        if (MAV.param["WP_TOTAL"] != null)
                            MAV.param["WP_TOTAL"].Value = wp_total - 1;
                        if (MAV.param["CMD_TOTAL"] != null)
                            MAV.param["CMD_TOTAL"].Value = wp_total - 1;
                        if (MAV.param["MIS_TOTAL"] != null)
                            MAV.param["MIS_TOTAL"].Value = wp_total - 1;

                        MAV.wps.Clear();

                        giveComport = false;
                        return;
                    }
                }
            //Console.WriteLine(DateTime.Now + " PC getwp " + buffer.msgid);
        }
    }

    public void InjectGpsData(byte[] data, ushort length)
    {
        InjectGpsData(MAV.SistemId, MAV.BilesenId, data, length);
    }

    /// <summary>
    ///     used to inject data into the gps ie rtcm/sbp/ubx
    /// </summary>
    /// <param name="data"></param>
    public void InjectGpsData(byte sysid, byte compid, byte[] data, ushort length, bool rtcm_message = true)
    {
        // new message
        if (rtcm_message)
        {
            var gps = new mavlink_gps_rtcm_data_t();
            var msglen = 180;

            if (length > msglen * 4)
                log.Error("Message too large " + length);

            // number of packets we need, including a termination packet if needed
            var nopackets = length % msglen == 0 ? length / msglen + 1 : length / msglen + 1;

            if (nopackets >= 4)
                nopackets = 4;

            // flags = isfrag(1)/frag(2)/seq(5)

            for (var a = 0; a < nopackets; a++)
            {
                // check if its a fragment
                if (nopackets > 1)
                    gps.flags = 1;
                else
                    gps.flags = 0;

                // add fragment number
                gps.flags += (byte)((a & 0x3) << 1);

                // add seq number
                gps.flags += (byte)((inject_seq_no & 0x1f) << 3);

                // create the empty buffer
                gps.data = new byte[msglen];

                // calc how much data we are copying
                var copy = Math.Min(length - a * msglen, msglen);

                // copy the data
                Array.Copy(data, a * msglen, gps.data, 0, copy);

                // set the length
                gps.len = (byte)copy;

                paketOlusturGonder((byte)MAVLINK_MSG_ID.GPS_RTCM_DATA, gps, sysid, compid);
            }

            inject_seq_no++;
        }
        else
        {
            var gps = new mavlink_gps_inject_data_t();
            var msglen = 110;

            var len = length % msglen == 0 ? length / msglen : length / msglen + 1;

            for (var a = 0; a < len; a++)
            {
                gps.data = new byte[msglen];

                var copy = Math.Min(length - a * msglen, msglen);

                Array.Copy(data, a * msglen, gps.data, 0, copy);
                gps.len = (byte)copy;
                gps.target_component = compid;
                gps.target_system = sysid;

                paketOlusturGonder((byte)MAVLINK_MSG_ID.GPS_INJECT_DATA, gps, sysid, compid);
            }
        }
    }

    public MAV_MISSION_RESULT setWP(Locationwp loc, ushort index, MAV_FRAME frame, byte current = 0,
        byte autocontinue = 1, bool use_int = false)
    {
        return setWP(MAV.SistemId, MAV.BilesenId, loc, index, frame, current, autocontinue, use_int);
    }

    /// <summary>
    ///     Save wp to eeprom
    /// </summary>
    /// <param name="loc">location struct</param>
    /// <param name="index">wp no</param>
    /// <param name="frame">global or relative</param>
    /// <param name="current">0 = no , 2 = guided mode</param>
    public MAV_MISSION_RESULT setWP(byte sysid, byte compid, Locationwp loc, ushort index, MAV_FRAME frame,
        byte current = 0,
        byte autocontinue = 1, bool use_int = false)
    {
        if (use_int)
        {
            var req = new mavlink_mission_item_int_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.command = loc.id;

            req.current = current;
            req.autocontinue = autocontinue;

            req.frame = (byte)frame;
            if (loc.id == (ushort)MAV_CMD.DO_DIGICAM_CONTROL || loc.id == (ushort)MAV_CMD.DO_DIGICAM_CONFIGURE)
            {
                req.y = (int)loc.lng;
                req.x = (int)loc.lat;
            }
            else
            {
                req.y = (int)(loc.lng * 1.0e7);
                req.x = (int)(loc.lat * 1.0e7);
            }

            req.z = loc.alt;

            req.param1 = loc.p1;
            req.param2 = loc.p2;
            req.param3 = loc.p3;
            req.param4 = loc.p4;

            req.seq = index;

            return setWP(req);
        }
        else
        {
            var req = new mavlink_mission_item_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.command = loc.id;

            req.current = current;
            req.autocontinue = autocontinue;

            req.frame = (byte)frame;
            req.y = (float)loc.lng;
            req.x = (float)loc.lat;
            req.z = loc.alt;

            req.param1 = loc.p1;
            req.param2 = loc.p2;
            req.param3 = loc.p3;
            req.param4 = loc.p4;

            req.seq = index;

            return setWP(req);
        }
    }

    public MAV_MISSION_RESULT setWP(mavlink_mission_item_t req)
    {
        giveComport = true;

        var index = req.seq;

        log.InfoFormat("setWP {7}:{8} {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command,
            req.param1,
            req.x, req.y, req.z, index, req.target_system, req.target_component);

        // request
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM, req);


        var start = DateTime.Now;
        var retrys = 10;

        while (true)
        {
            if (!(start.AddMilliseconds(450) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWP Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM, req);

                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWP");
            }

            var buffer = paketOku();
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ACK && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_ack_t>();
                    log.Info("set wp " + index + " ACK 47 : " + buffer.msgid + " ans " +
                             Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                    // check this gcs sent it
                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (req.current == 2)
                    {
                        MAVlist[req.target_system, req.target_component].GuidedMode = req;
                    }
                    else if (req.current == 3)
                    {
                    }
                    else
                    {
                        MAVlist[req.target_system, req.target_component].wps[req.seq] = req;
                    }

                    //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                    {
                        giveComport = false;
                        return (MAV_MISSION_RESULT)ans.type;
                    }
                }

                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST &&
                    buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_t>();

                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (ans.seq == index + 1)
                    {
                        log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer.msgid);
                        giveComport = false;

                        if (req.current == 2)
                        {
                            MAVlist[req.target_system, req.target_component].GuidedMode = req;
                        }
                        else if (req.current == 3)
                        {
                        }
                        else
                        {
                            MAVlist[req.target_system, req.target_component].wps[req.seq] = req;
                        }

                        //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                        {
                            giveComport = false;
                            return MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
                        }
                    }

                    log.InfoFormat(
                        "set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " +
                        buffer.msgid +
                        " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                    // resend point now
                    start = DateTime.MinValue;
                }
                else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST_INT &&
                         buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_int_t>();

                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (ans.seq == index + 1)
                    {
                        log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer.msgid);
                        giveComport = false;

                        if (req.current == 2)
                        {
                            MAVlist[req.target_system, req.target_component].GuidedMode = req;
                        }
                        else if (req.current == 3)
                        {
                        }
                        else
                        {
                            MAVlist[req.target_system, req.target_component].wps[req.seq] = req;
                        }

                        //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                        {
                            giveComport = false;
                            return MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
                        }
                    }

                    log.InfoFormat(
                        "set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " +
                        buffer.msgid +
                        " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                    // resend point now
                    start = DateTime.MinValue;
                }
                //Console.WriteLine(DateTime.Now + " PC setwp " + buffer.msgid);
            }
        }

        // return MAV_MISSION_RESULT.MAV_MISSION_INVALID;
    }

    public MAV_MISSION_RESULT setWP(mavlink_mission_item_int_t req)
    {
        giveComport = true;

        var index = req.seq;

        log.InfoFormat("setWPint {7}:{8} {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command,
            req.param1,
            req.x / 1.0e7, req.y / 1.0e7, req.z, index, req.target_system, req.target_component);

        // request
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM_INT, req);

        var start = DateTime.Now;
        var retrys = 10;

        while (true)
        {
            if (!(start.AddMilliseconds(450) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("setWP Retry " + retrys);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_ITEM_INT, req);

                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - setWP");
            }

            var buffer = paketOku();
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ACK && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_ack_t>();
                    log.Info("set wp " + index + " ACK 47 : " + buffer.msgid + " ans " +
                             Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                    // check this gcs sent it
                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    giveComport = false;

                    if (req.current == 2)
                    {
                        MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                    }
                    else if (req.current == 3)
                    {
                    }
                    else
                    {
                        MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                    }

                    //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                    {
                        giveComport = false;
                        return (MAV_MISSION_RESULT)ans.type;
                    }
                }

                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST &&
                    buffer.sysid == req.target_system && buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_t>();
                    if (ans.target_system != YkiSistemId ||
                        ans.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (ans.seq == index + 1)
                    {
                        log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer.msgid);
                        giveComport = false;

                        if (req.current == 2)
                        {
                            MAVlist[req.target_system, req.target_component].GuidedMode = (Locationwp)req;
                        }
                        else if (req.current == 3)
                        {
                        }
                        else
                        {
                            MAVlist[req.target_system, req.target_component].wps[req.seq] = (Locationwp)req;
                        }

                        //if (ans.target_system == req.target_system && ans.target_component == req.target_component)
                        {
                            giveComport = false;
                            return MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
                        }
                    }

                    log.InfoFormat(
                        "set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " +
                        buffer.msgid +
                        " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                    // resend point now
                    start = DateTime.MinValue;
                }
                //Console.WriteLine(DateTime.Now + " PC setwp " + buffer.msgid);
            }
        }

        // return MAV_MISSION_RESULT.MAV_MISSION_INVALID;
    }

    public int istenenWpNoGetir()
    {
        giveComport = true;
        var start = DateTime.Now;

        while (true)
        {
            if (!(start.AddMilliseconds(1500) > DateTime.Now))
            {
                giveComport = false;
                throw new TimeoutException("Timeout on read - getRequestedWPNo");
            }

            var buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_REQUEST)
                {
                    var ans = buffer.ToStructure<mavlink_mission_request_t>();

                    log.InfoFormat("getRequestedWPNo seq {0} ts {1} tc {2}", ans.seq, ans.target_system,
                        ans.target_component);

                    giveComport = false;

                    return ans.seq;
                }
        }
    }

    public void siradakiWpHedefIrtifaBelirle(ushort wpno, float alt)
    {
        // get the existing wp
        var current = WpGetir(wpno);

        var req = new mavlink_mission_write_partial_list_t();
        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;

        req.start_index = (short)wpno;
        req.end_index = (short)wpno;

        // change the alt
        current.alt = alt;

        // send a request to update single point
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_WRITE_PARTIAL_LIST, req);
        Thread.Sleep(10);
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MISSION_WRITE_PARTIAL_LIST, req);

        var frame = (current.options & 0x1) == 0 ? MAV_FRAME.GLOBAL : MAV_FRAME.GLOBAL_RELATIVE_ALT;

        //send the point with new alt
        setWP(current, wpno, MAV_FRAME.GLOBAL_RELATIVE_ALT);

        // set the point as current to reload the modified command
        SuankiWPDegistir(wpno);
    }

    public void KonumaGit(PointLatLngAlt konum)
    {
        KonumaGit(MAV.SistemId, MAV.BilesenId, konum.ConvertToLocationwp());
    }

    public void KonumaGit(Locationwp konum)
    {
        KonumaGit(MAV.SistemId, MAV.BilesenId, konum);
    }

    public void KonumaGit(byte sysid, byte compid, Locationwp gotohere)
    {
        if (gotohere.alt == 0 || gotohere.lat == 0 || gotohere.lng == 0)
            return;

        giveComport = true;

        try
        {
            gotohere.id = (ushort)MAV_CMD.WAYPOINT;

            log.InfoFormat("KonumaGit {0}:{1} lat {2} lng {3} alt {4}", sysid, compid, gotohere.lat,
                gotohere.lng, gotohere.alt);

            if (MAVlist[sysid, compid].cs.firmware == Firmwares.ArduPlane)
            {
                var ans = setWP(sysid, compid, gotohere, 0, MAV_FRAME.GLOBAL_RELATIVE_ALT, 2);

                if (ans != MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                    throw new Exception("Konuma gitme komutu araç tarafından kabul edilmedi");
            }
            else
            {
                setPositionTargetGlobalInt(sysid, compid,
                    true, false, false, false, MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                    gotohere.lat, gotohere.lng, gotohere.alt, 0, 0, 0, 0, 0);
            }
        }
        catch (Exception ex)
        {
            log.Error(ex);
        }

        giveComport = false;
    }

    public void KalkisYap(float irtifa)
    {
        if (BaseStream.IsOpen)
            try
            {
                KomutGonder((byte)SeciliSistemId, (byte)SeciliBilesenId,
                    MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, irtifa);
            }
            catch (Exception ex)
            {
                log.Error(ex.Message);
            }
    }

    public void setNewWPAlt(Locationwp gotohere)
    {
        setNewWPAlt((byte)SeciliSistemId, (byte)SeciliBilesenId, gotohere);
    }

    public void setNewWPAlt(byte sysid, byte compid, Locationwp gotohere)
    {
        giveComport = true;

        try
        {
            gotohere.id = (ushort)MAV_CMD.WAYPOINT;

            log.InfoFormat("setNewWPAlt {0}:{1} lat {2} lng {3} alt {4}", sysid, compid, gotohere.lat, gotohere.lng,
                gotohere.alt);

            var ans = setWP(gotohere, 0, MAV_FRAME.GLOBAL_RELATIVE_ALT, 3);

            if (ans != MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                throw new Exception("Alt Change Failed");

            /*
            // currently plane supports just an alt change, copter requires all lat/lng/alt
            setPositionTargetGlobalInt((byte)sysid, (byte)compid,
                true, false, false, false, MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT,
                gotohere.lat, gotohere.lng, gotohere.alt, 0, 0, 0, 0, 0);
                */
        }
        catch (Exception ex)
        {
            giveComport = false;
            log.Error(ex);
            throw;
        }

        giveComport = false;
    }

    public void setPositionTargetGlobalInt(byte sysid, byte compid, bool pos, bool vel, bool acc, bool yaw,
        MAV_FRAME frame, double lat, double lng, double alt, double vx, double vy, double vz, double yawangle,
        double yawrate)
    {
        // for mavlink SET_POSITION_TARGET messages
        const ushort MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE = (1 << 0) | (1 << 1) | (1 << 2);
        const ushort MAVLINK_SET_POS_TYPE_MASK_ALT_IGNORE = (0 << 0) | (0 << 1) | (1 << 2);
        const ushort MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE = (1 << 3) | (1 << 4) | (1 << 5);
        const ushort MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE = (1 << 6) | (1 << 7) | (1 << 8);
        const ushort MAVLINK_SET_POS_TYPE_MASK_FORCE = 1 << 9;
        const ushort MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE = (1 << 10) | (1 << 11);

        var target = new mavlink_set_position_target_global_int_t
        {
            target_system = sysid,
            target_component = compid,
            alt = (float)alt,
            lat_int = (int)(lat * 1e7),
            lon_int = (int)(lng * 1e7),
            coordinate_frame = (byte)frame,
            vx = (float)vx,
            vy = (float)vy,
            vz = (float)vz,
            yaw = (float)yawangle,
            yaw_rate = (float)yawrate
        };

        target.type_mask = ushort.MaxValue;

        if (pos && lat != 0 && lng != 0)
            target.type_mask -= MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        if (pos && lat == 0 && lng == 0)
            target.type_mask -= MAVLINK_SET_POS_TYPE_MASK_ALT_IGNORE;
        if (vel)
            target.type_mask -= MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        if (acc)
            target.type_mask -= MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        if (yaw)
            target.type_mask -= MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;

        if (pos)
        {
            if (lat != 0)
                MAVlist[sysid, compid].GuidedMode.x = (float)lat;
            if (lng != 0)
                MAVlist[sysid, compid].GuidedMode.y = (float)lng;
            MAVlist[sysid, compid].GuidedMode.z = (float)alt;
        }

        var pos_ignore = (target.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE) > 0;
        var vel_ignore = (target.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE) > 0;
        var acc_ignore = (target.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE) > 0;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.SET_POSITION_TARGET_GLOBAL_INT, target, sysid, compid);
    }

    public void setAttitudeTarget()
    {
        var target = new mavlink_set_attitude_target_t
        {
            target_system = (byte)SeciliSistemId,
            target_component = (byte)SeciliBilesenId
        };
    }

    public void setDigicamConfigure()
    {
        // not implmented
    }

    public void setDigicamControl(bool shot)
    {
        var req = new mavlink_digicam_control_t();

        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;
        req.shot = shot ? (byte)1 : (byte)0;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.DIGICAM_CONTROL, req);

        KomutGonder(MAV_CMD.DO_DIGICAM_CONTROL, 0, 0, 0, 0, 1, 0, 0);

        //MAVLINK_MSG_ID.CAMERA_FEEDBACK;

        //mavlink_camera_feedback_t
    }

    public void setMountConfigure(MAV_MOUNT_MODE mountmode, bool stabroll, bool stabpitch, bool stabyaw)
    {
        var req = new mavlink_mount_configure_t();

        req.target_system = MAV.SistemId;
        req.target_component = MAV.BilesenId;
        req.mount_mode = (byte)mountmode;
        req.stab_pitch = stabpitch ? (byte)1 : (byte)0;
        req.stab_roll = stabroll ? (byte)1 : (byte)0;
        req.stab_yaw = stabyaw ? (byte)1 : (byte)0;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MOUNT_CONFIGURE, req);
        Thread.Sleep(20);
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MOUNT_CONFIGURE, req);
    }

    public void setMountControl(double pa, double pb, double pc, bool islatlng)
    {
        setMountControl(MAV.SistemId, MAV.BilesenId, pa, pb, pc, islatlng);
    }

    public void setMountControl(byte sisId, byte bilId, double pa, double pb, double pc, bool islatlng)
    {
        var req = new mavlink_mount_control_t();

        req.target_system = sisId;
        req.target_component = bilId;
        if (!islatlng)
        {
            req.input_a = (int)pa;
            req.input_b = (int)pb;
            req.input_c = (int)pc;
        }
        else
        {
            req.input_a = (int)(pa * 10000000.0);
            req.input_b = (int)(pb * 10000000.0);
            req.input_c = (int)(pc * 100.0);
        }

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MOUNT_CONTROL, req);
        Thread.Sleep(20);
        paketOlusturGonder((byte)MAVLINK_MSG_ID.MOUNT_CONTROL, req);
    }

    public void ModGuncelle(string mod)
    {
        ModGuncelle(MAV.SistemId, MAV.BilesenId, mod);
    }

    public void ModGuncelle(byte sisId, byte bilId, string mod)
    {
        var mode = new mavlink_set_mode_t();

        if (translateMode(sisId, bilId, mod, ref mode))
            ModGuncelle(sisId, bilId, mode);
        else
        {
            throw new Exception("ModGuncelle: Unknown mode " + mod);
        }
    }

    public void ModGuncelle(mavlink_set_mode_t mode, MAV_MODE_FLAG base_mode = 0)
    {
        ModGuncelle(MAV.SistemId, MAV.BilesenId, mode, base_mode);
    }

    public void ModGuncelle(byte sisId, byte bilId, mavlink_set_mode_t mode, MAV_MODE_FLAG base_mode = 0)
    {
        mode.base_mode |= (byte)base_mode;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.SET_MODE, mode, sisId, bilId);
        Thread.Sleep(10);
        paketOlusturGonder((byte)MAVLINK_MSG_ID.SET_MODE, mode, sisId, bilId);
    }

    public async Task<MAVLinkMessage> readPacketAsync()
    {
        return await Task.Run(() =>
        {
            var start = DateTime.Now;
            var ans = paketOku();
            //Console.WriteLine("readPacketAsync " + (DateTime.Now - start));
            return ans;
        });
    }


    public bool getVersion(bool responcerequired = true)
    {
        return getVersion(MAV.SistemId, MAV.BilesenId, responcerequired);
    }

    public bool getVersion(byte sysid, byte compid, bool responcerequired = true)
    {
        var req = new mavlink_autopilot_version_request_t();

        req.target_component = compid;
        req.target_system = sysid;

        // request point
        paketOlusturGonder((byte)MAVLINK_MSG_ID.AUTOPILOT_VERSION_REQUEST, req);

        if (!responcerequired)
            return true;

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(500) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("getVersion Retry " + retrys + " - giv com " + giveComport);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.AUTOPILOT_VERSION_REQUEST, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                return false;
            }

            var buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.AUTOPILOT_VERSION && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    giveComport = false;

                    return true;
                }
        }
    }

    public PointLatLngAlt getFencePoint(int no, ref int total)
    {
        MAVLinkMessage buffer;

        giveComport = true;

        var plla = new PointLatLngAlt();
        var req = new mavlink_fence_fetch_point_t();

        req.idx = (byte)no;
        req.target_component = MAV.BilesenId;
        req.target_system = MAV.SistemId;

        // request point
        paketOlusturGonder((byte)MAVLINK_MSG_ID.FENCE_FETCH_POINT, req);

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("getFencePoint Retry " + retrys + " - giv com " + giveComport);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.FENCE_FETCH_POINT, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - getFencePoint");
            }

            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.FENCE_POINT && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var fp = buffer.ToStructure<mavlink_fence_point_t>();

                    // check this gcs sent it
                    if (fp.target_system != YkiSistemId ||
                        fp.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    giveComport = false;

                    plla.Lat = fp.lat;
                    plla.Lng = fp.lng;
                    plla.Tag = fp.idx.ToString();

                    total = fp.count;

                    return plla;
                }
        }
    }

    public FileStream GetLog(ushort no)
    {
        var ms = new FileStream(Path.GetTempFileName(), FileMode.Create, FileAccess.ReadWrite);
        var set = new Hashtable();

        giveComport = true;
        MAVLinkMessage buffer;

        Progress?.Invoke(0, "");

        uint totallength = 0;
        uint ofs = 0;
        uint bps = 0;
        var bpstimer = DateTime.Now;

        var req = new mavlink_log_request_data_t();

        req.target_component = MAV.BilesenId;
        req.target_system = MAV.SistemId;
        req.id = no;
        req.ofs = ofs;
        // entire log
        req.count = 0xFFFFFFFF;

        // request point
        paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_REQUEST_DATA, req);

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(3000) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("GetLog Retry " + retrys + " - giv com " + giveComport);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_REQUEST_DATA, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - GetLog");
            }

            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.LOG_DATA && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var data = buffer.ToStructure<mavlink_log_data_t>();

                    if (data.id != no)
                        continue;

                    // reset retrys
                    retrys = 3;
                    start = DateTime.Now;

                    bps += data.count;

                    // record what we have received
                    set[(data.ofs / 90).ToString()] = 1;

                    ms.Seek(data.ofs, SeekOrigin.Begin);
                    ms.Write(data.data, 0, data.count);

                    // update new start point
                    req.ofs = data.ofs + data.count;

                    if (bpstimer.Second != DateTime.Now.Second)
                    {
                        Progress?.Invoke((int)req.ofs, "");

                        //Console.WriteLine("log dl bps: " + bps.ToString());
                        bpstimer = DateTime.Now;
                        bps = 0;
                    }

                    // if data is less than max packet size or 0 > exit
                    if (data.count < 90 || data.count == 0)
                    {
                        totallength = data.ofs + data.count;
                        log.Info("start fillin len " + totallength + " count " + set.Count + " datalen " +
                                 data.count);
                        break;
                    }
                }
        }

        log.Info("set count " + set.Count);
        log.Info("count total " + (totallength / 90 + 1));
        log.Info("totallength " + totallength);
        log.Info("current length " + ms.Length);

        while (true && BaseStream != null && BaseStream.IsOpen)
        {
            if (totallength == ms.Length && set.Count >= totallength / 90 + 1)
            {
                giveComport = false;
                return ms;
            }

            if (!(start.AddMilliseconds(500) > DateTime.Now))
                for (var a = 0; a < totallength / 90 + 1; a++)
                    if (!set.ContainsKey(a.ToString()))
                    {
                        // request large chunk if they are back to back
                        uint bytereq = 90;
                        var b = a + 1;
                        while (!set.ContainsKey(b.ToString()))
                        {
                            bytereq += 90;
                            b++;
                        }

                        req.ofs = (uint)(a * 90);
                        req.count = bytereq;
                        log.Info("req missing " + req.ofs + " bytes " + req.count + " got " + set.Count + "/" +
                                 (totallength / 90 + 1));
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_REQUEST_DATA, req);
                        start = DateTime.Now;
                        break;
                    }

            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.LOG_DATA && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var data = buffer.ToStructure<mavlink_log_data_t>();

                    if (data.id != no)
                        continue;

                    // reset retrys
                    retrys = 3;
                    start = DateTime.Now;

                    bps += data.count;

                    // record what we have received
                    set[(data.ofs / 90).ToString()] = 1;

                    ms.Seek(data.ofs, SeekOrigin.Begin);
                    ms.Write(data.data, 0, data.count);

                    // update new start point
                    req.ofs = data.ofs + data.count;

                    if (bpstimer.Second != DateTime.Now.Second)
                    {
                        Progress?.Invoke((int)req.ofs, "");

                        //Console.WriteLine("log dl bps: " + bps.ToString());
                        bpstimer = DateTime.Now;
                        bps = 0;
                    }

                    // check if we have next set and invalidate to request next packets
                    if (set.ContainsKey((data.ofs / 90 + 1).ToString())) start = DateTime.MinValue;

                    // if data is less than max packet size or 0 > exit
                    if (data.count < 90 || data.count == 0) continue;
                }
        }

        throw new Exception("Failed to get log");
    }

    public List<mavlink_log_entry_t> GetLogList()
    {
        var ans = new List<mavlink_log_entry_t>();

        var entry1 = GetLogEntry();

        log.Info("id " + entry1.id + " lastllogno " + entry1.last_log_num + " #logs " + entry1.num_logs + " size " +
                 entry1.size);
        //ans.Add(entry1);

        for (var a = (ushort)(entry1.last_log_num - entry1.num_logs + 1); a <= entry1.last_log_num; a++)
        {
            var entry = GetLogEntry(a, a);
            ans.Add(entry);
        }

        return ans;
    }

    public void GetMountStatus()
    {
        var req = new mavlink_mount_status_t();
        req.target_component = MAV.BilesenId;
        req.target_system = MAV.SistemId;

        paketOlusturGonder((byte)MAVLINK_MSG_ID.MOUNT_STATUS, req);
    }

    public mavlink_log_entry_t GetLogEntry(ushort startno = 0, ushort endno = ushort.MaxValue)
    {
        giveComport = true;
        MAVLinkMessage buffer;

        var req = new mavlink_log_request_list_t();

        req.target_component = MAV.BilesenId;
        req.target_system = MAV.SistemId;
        req.start = startno;
        req.end = endno;

        log.Info("GetLogEntry " + startno + "-" + endno);

        // request point
        paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_REQUEST_LIST, req);

        var start = DateTime.Now;
        var retrys = 5;

        while (true)
        {
            if (!(start.AddMilliseconds(2000) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("GetLogEntry Retry " + retrys + " - giv com " + giveComport);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_REQUEST_LIST, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - GetLogEntry");
            }

            buffer = paketOku();
            if (buffer.Length > 5)
            {
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.LOG_ENTRY && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var ans = buffer.ToStructure<mavlink_log_entry_t>();

                    if (ans.id >= startno && ans.id <= endno)
                    {
                        giveComport = false;
                        return ans;
                    }
                }

                if (buffer.msgid == (byte)MAVLINK_MSG_ID.LOG_DATA && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                    throw new Exception("Existing log download already in progress.");
            }
        }
    }

    public void EraseLog()
    {
        var req = new mavlink_log_erase_t();

        req.target_component = MAV.BilesenId;
        req.target_system = MAV.SistemId;

        // send twice - we have no feedback on this
        paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_ERASE, req);
        paketOlusturGonder((byte)MAVLINK_MSG_ID.LOG_ERASE, req);
    }

    public List<PointLatLngAlt> getRallyPoints()
    {
        var points = new List<PointLatLngAlt>();

        if (!MAV.param.ContainsKey("RALLY_TOTAL"))
            return points;

        var count = int.Parse(MAV.param["RALLY_TOTAL"].ToString());

        for (var a = 0; a < count - 1; a++)
            try
            {
                var plla = getRallyPoint(a, ref count);
                points.Add(plla);
            }
            catch
            {
                return points;
            }

        return points;
    }

    public PointLatLngAlt getRallyPoint(int no, ref int total)
    {
        MAVLinkMessage buffer;

        giveComport = true;

        var plla = new PointLatLngAlt();
        var req = new mavlink_rally_fetch_point_t();

        req.idx = (byte)no;
        req.target_component = MAV.BilesenId;
        req.target_system = MAV.SistemId;

        // request point
        paketOlusturGonder((byte)MAVLINK_MSG_ID.RALLY_FETCH_POINT, req);

        var start = DateTime.Now;
        var retrys = 3;

        while (true)
        {
            if (!(start.AddMilliseconds(700) > DateTime.Now))
            {
                if (retrys > 0)
                {
                    log.Info("getRallyPoint Retry " + retrys + " - giv com " + giveComport);
                    paketOlusturGonder((byte)MAVLINK_MSG_ID.FENCE_FETCH_POINT, req);
                    start = DateTime.Now;
                    retrys--;
                    continue;
                }

                giveComport = false;
                throw new TimeoutException("Timeout on read - getRallyPoint");
            }

            buffer = paketOku();
            if (buffer.Length > 5)
                if (buffer.msgid == (byte)MAVLINK_MSG_ID.RALLY_POINT && buffer.sysid == req.target_system &&
                    buffer.compid == req.target_component)
                {
                    var fp = buffer.ToStructure<mavlink_rally_point_t>();

                    // check this gcs sent it
                    if (fp.target_system != YkiSistemId ||
                        fp.target_component != (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER)
                        continue;

                    if (req.idx != fp.idx)
                    {
                        paketOlusturGonder((byte)MAVLINK_MSG_ID.FENCE_FETCH_POINT, req);
                        continue;
                    }

                    plla.Lat = fp.lat / t7;
                    plla.Lng = fp.lng / t7;
                    plla.Tag = fp.idx.ToString();
                    plla.Alt = fp.alt;

                    total = fp.count;

                    giveComport = false;

                    return plla;
                }
        }
    }

    public bool setFencePoint(byte index, PointLatLngAlt plla, byte fencepointcount)
    {
        var fp = new mavlink_fence_point_t();

        fp.idx = index;
        fp.count = fencepointcount;
        fp.lat = (float)plla.Lat;
        fp.lng = (float)plla.Lng;
        fp.target_component = MAV.BilesenId;
        fp.target_system = MAV.SistemId;

        var retry = 3;

        PointLatLngAlt newfp;

        while (retry > 0)
        {
            paketOlusturGonder((byte)MAVLINK_MSG_ID.FENCE_POINT, fp);
            var counttemp = 0;
            newfp = getFencePoint(fp.idx, ref counttemp);

            if (newfp.MesafeGetir(plla) < 5)
                return true;
            retry--;
        }

        throw new Exception("Could not verify GeoFence Point");
    }

    public bool setRallyPoint(byte index, PointLatLngAlt plla, short break_alt, ushort land_dir_cd, byte flags,
        byte rallypointcount)
    {
        var rp = new mavlink_rally_point_t();

        rp.idx = index;
        rp.count = rallypointcount;
        rp.lat = (int)(plla.Lat * t7);
        rp.lng = (int)(plla.Lng * t7);
        rp.alt = (short)plla.Alt;
        rp.break_alt = break_alt;
        rp.land_dir = land_dir_cd;
        rp.flags = flags;
        rp.target_component = MAV.BilesenId;
        rp.target_system = MAV.SistemId;

        var retry = 3;

        while (retry > 0)
        {
            paketOlusturGonder((byte)MAVLINK_MSG_ID.RALLY_POINT, rp);
            var counttemp = 0;
            var newfp = getRallyPoint(rp.idx, ref counttemp);

            if (newfp.Lat == plla.Lat && newfp.Lng == rp.lng)
            {
                EkranaYazdir("Rally Set");
                return true;
            }

            retry--;
        }

        return false;
    }

    public bool SetSensorOffsets(sensoroffsetsenum sensor, float x, float y, float z)
    {
        return KomutGonder(MAV_CMD.PREFLIGHT_SET_SENSOR_OFFSETS, (int)sensor, x, y, z, 0, 0, 0);
    }


    public bool translateMode(string modein, ref mavlink_set_mode_t mode)
    {
        return translateMode(MAV.SistemId, MAV.BilesenId, modein, ref mode);
    }

    public bool translateMode(byte sysid, byte compid, string secilenMod, ref mavlink_set_mode_t mode)
    {
        mode.target_system = sysid;

        if (secilenMod == null || secilenMod == "")
            return false;

        try
        {
            var modelist = Common.getModesList(MAVlist[sysid, compid].cs.firmware);

            foreach (var pair in modelist)
                if (pair.Value.ToLower() == secilenMod.ToLower())
                {
                    mode.base_mode = (byte)MAV_MODE_FLAG.CUSTOM_MODE_ENABLED;
                    mode.custom_mode = (uint)pair.Key;
                }

            if (mode.base_mode == 0)
            {
                log.Error($"Mod {secilenMod} olarak değiştirilemedi");
                return false;
            }
        }
        catch (Exception ex)
        {
            log.Error("Mod aranırken hata oluştu!", ex);
            return false;
        }

        return true;
    }

    public void setAPType(byte sysid, byte compid)
    {
        MAVlist[sysid, compid].SistemId = sysid;
        MAVlist[sysid, compid].BilesenId = compid;

        switch (MAVlist[sysid, compid].apname)
        {
            case MAV_AUTOPILOT.ARDUPILOTMEGA:
                switch (MAVlist[sysid, compid].aptype)
                {
                    case MAV_TYPE.FIXED_WING:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduPlane;
                        break;
                    case MAV_TYPE.QUADROTOR:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduCopter2;
                        break;
                    case MAV_TYPE.TRICOPTER:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduCopter2;
                        break;
                    case MAV_TYPE.HEXAROTOR:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduCopter2;
                        break;
                    case MAV_TYPE.OCTOROTOR:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduCopter2;
                        break;
                    case MAV_TYPE.HELICOPTER:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduCopter2;
                        break;
                    case MAV_TYPE.GROUND_ROVER:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduRover;
                        break;
                    case MAV_TYPE.SURFACE_BOAT:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduRover;
                        break;
                    case MAV_TYPE.SUBMARINE:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduSub;
                        break;
                    case MAV_TYPE.ANTENNA_TRACKER:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduTracker;
                        break;
                    default:
                        log.Error(MAVlist[sysid, compid].aptype + " not registered as valid type");
                        break;
                }

                break;
            case MAV_AUTOPILOT.UDB:
                switch (MAVlist[sysid, compid].aptype)
                {
                    case MAV_TYPE.FIXED_WING:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.ArduPlane;
                        break;
                }

                break;
            case MAV_AUTOPILOT.GENERIC:
                switch (MAVlist[sysid, compid].aptype)
                {
                    case MAV_TYPE.FIXED_WING:
                        MAVlist[sysid, compid].cs.firmware = Firmwares.Ateryx;
                        break;
                }

                break;
            case MAV_AUTOPILOT.PX4:
                MAVlist[sysid, compid].cs.firmware = Firmwares.PX4;
                break;
            default:
                switch (MAVlist[sysid, compid].aptype)
                {
                    case MAV_TYPE.GIMBAL: // storm32 - name 83
                        MAVlist[sysid, compid].cs.firmware = Firmwares.Gymbal;
                        break;
                }

                break;
        }
    }
    #endregion

    #region ### Private Metotlar ###
    private void AcAp(bool getparams)
    {
        giveComport = true;

        if (BaseStream is SerialPort)
            // ayarların yerleşmesine izin verin - önceki dtr 
            Thread.Sleep(1000);


        var hbseen = false;

        try
        {
            BaseStream.ReadBufferSize = 16 * 1024;

            lock (yazmaKilidi) // so we dont have random traffic
            {
                log.Info("Open port with " + BaseStream.PortName + " " + BaseStream.BaudRate);

                BaseStream.Open();

                BaseStream.DiscardInBuffer();

                // gecikme yoksa diğer kartlarda sorun var gibi görünüyor?
                // olası önyükleyici zaman aşımı sorunu
                if (BaseStream is SerialPort) Thread.Sleep(1000);
            }

            var hbhistory = new List<MAVLinkMessage>();

            var start = DateTime.Now;
            var deadline = start.AddSeconds(BaglantiZamanAsimi);

            var countDown = new Timer { Interval = 1000, AutoReset = false };
            countDown.Elapsed += (sender, e) =>
            {
                var secondsRemaining = (deadline - e.SignalTime).Seconds;

                if (secondsRemaining > 0)
                    countDown.Start();
            };
            countDown.Start();

            // px4 native
            BaseStream.WriteLine("sh /etc/init.d/rc.usb");

            var count = 0;

            while (true)
            {
                //Bağlantı kurma burada başlıyor

                if (DateTime.Now > deadline)
                {
                    // bağlantı zaman aşımına uğradığında

                    // Bağlantıyı kapat
                    countDown.Stop();
                    Kapat();

                    if (hbseen)
                        //Yanlızca 1 Nabız verisi alındıysa
                        throw new Exception("Bu bağlantı noktasından yalnızca 1 Mavlink Nabız Paketi okundu - " +
                                            "Donanımınızın doğru kurulduğunu doğrulayın\r\n" +
                                            "Bağlanmadan önce 2 geçerli kalp atışı paketi beklenir.");
                    throw new Exception("Otopilottan nabız paketi alınmadı!");
                }

                Thread.Sleep(1);

                var buffer = nabizAl();
                if (buffer.Length > 0)
                {
                    var hb = buffer.ToStructure<mavlink_heartbeat_t>();

                    if (hb.type != (byte)MAV_TYPE.GCS) hbhistory.Add(buffer);
                }

                if (hbhistory.Count > 0)
                    hbseen = true;

                count++;

                // veri alamazsak, rts/cts'yi etkinleştiriyoruz
                if (buffer.Length == 0 && BaseStream is SerialPort)
                    try
                    {
                        BaseStream.RtsEnable = !BaseStream.RtsEnable;
                    }
                    catch
                    {
                    }

                // Nabiz var mı diye kontrol et
                if (hbhistory.Count > 0)
                {
                    var exit = false;

                    var mavLinkMessage = hbhistory.FirstOrDefault();
                    // en çok görülen nabızları al
                    var mostseenlist = hbhistory.GroupBy(s => MAVList.GetID(s.sysid, s.compid))
                        .OrderByDescending(s => s.Count()).Where(s => s.Count() >= 2);
                    foreach (var mostseen in mostseenlist)
                    {
                        // görülme sayısını hesapla
                        var seentimes = mostseen.Count();

                        // get the most seen mavlinkmessage
                        var msg = mostseen.First();

                        // preference compid of 1, failover to anything that we have seen 4 times
                        if ((seentimes >= 2 && msg.compid == 1) || seentimes >= 4)
                        {
                            SetupMavConnect(msg, (mavlink_heartbeat_t)msg.data);
                            SeciliSistemId = msg.sysid;
                            SeciliBilesenId = msg.compid;
                            exit = true;
                            break;
                        }
                    }

                    if (exit)
                        break;
                }
            }

            countDown.Stop();

            var temp = Encoding.ASCII.GetBytes("Furkan's Ground Control Station is connecting\0");
            Array.Resize(ref temp, 50);
            // 
            paketOlusturGonder((byte)MAVLINK_MSG_ID.STATUSTEXT,
                new mavlink_statustext_t { severity = (byte)MAV_SEVERITY.INFO, text = temp });
            // mavlink2
            paketOlusturGonder((byte)MAVLINK_MSG_ID.STATUSTEXT,
                new mavlink_statustext_t { severity = (byte)MAV_SEVERITY.INFO, text = temp }, SeciliSistemId,
                SeciliBilesenId, true, true);

            // this ensures a mavlink2 change has been noticed
            nabizAl();

            getVersion();

            if (getparams)
            {
                EkranaYazdir("Parametreler Alınıyor.. (sysid " + MAV.SistemId + " compid " + MAV.BilesenId + ") ");

                parametreListesiniGetir(MAV.SistemId, MAV.BilesenId);
            }
        }
        catch (Exception e)
        {
            try
            {
                BaseStream.Close();
            }
            catch
            {
            }

            giveComport = false;

            log.Error(e);
            throw;
        }

        //frmProgressReporter.Close();
        giveComport = false;

        EkranaYazdir("Parametreler alındı");

        log.Info("Done open " + MAV.SistemId + " " + MAV.BilesenId);
        MAV.packetslost = 0;
        MAV.synclost = 0;
    }

    private static void EkranaYazdir(string text)
    {
        if (log.Logger.Repository.Configured)
            log.Info(text);
        else
            Console.WriteLine(text);
    }

    private void SetupMavConnect(MAVLinkMessage message, mavlink_heartbeat_t hb)
    {
        mavlinkSurum = hb.mavlink_version;
        MAVlist[message.sysid, message.compid].aptype = (MAV_TYPE)hb.type;
        MAVlist[message.sysid, message.compid].apname = (MAV_AUTOPILOT)hb.autopilot;

        setAPType(message.sysid, message.compid);

        MAVlist[message.sysid, message.compid].SistemId = message.sysid;
        MAVlist[message.sysid, message.compid].BilesenId = message.compid;
        MAVlist[message.sysid, message.compid].recvpacketcount = message.seq;
        log.InfoFormat("ID sys {0} comp {1} ver{2} type {3} name {4}", message.sysid, message.compid,
            mavlinkSurum,
            MAV.aptype.ToString(), MAV.apname.ToString());
    }

    private void paketOlusturGonder(MAVLINK_MSG_ID messageType, object icerik)
    {
        paketOlusturGonder((int)messageType, icerik);
    }

    private void paketOlusturGonder(int tur, object icerik)
    {
        //uses currently targeted mavs sysid and compid
        paketOlusturGonder(tur, icerik, MAV.SistemId, MAV.BilesenId);
    }


    // returns true for ok
    private bool hzratecheck(double pps, int hzrate)
    {
        if (double.IsInfinity(pps))
            return false;
        // 0 request
        if (hzrate == 0 && pps == 0) return true;

        // range check pps, include packetloss
        if (pps > hzrate - 1 && pps < hzrate + 0.1)
            return true;

        return false;
    }

    private void veriAkisiGetir(MAV_DATA_STREAM id, int hzrate)
    {
        veriAkisiGetir(MAV.SistemId, MAV.BilesenId, id, hzrate);
    }

    private void veriAkisiGetir(byte sysid, byte compid, MAV_DATA_STREAM id, int hzrate)
    {
        if (hzrate == -1)
            return;

        var req = new mavlink_request_data_stream_t();
        req.target_system = sysid;
        req.target_component = compid;

        req.req_message_rate = (byte)hzrate;
        req.start_stop = 1; // start
        req.req_stream_id = (byte)id; // id

        // send each one twice.
        paketOlusturGonder((byte)MAVLINK_MSG_ID.REQUEST_DATA_STREAM, req, sysid, compid);
        paketOlusturGonder((byte)MAVLINK_MSG_ID.REQUEST_DATA_STREAM, req, sysid, compid);
    }
    private void PacketReceived(MAVLinkMessage buffer)
    {
        var type = (MAVLINK_MSG_ID)buffer.msgid;

        lock (Subscriptions)
        {
            foreach (var item in Subscriptions.ToArray())
                if (item.Key == type)
                    try
                    {
                        item.Value(buffer);
                    }
                    catch (Exception ex)
                    {
                        log.Error(ex);
                    }
        }
    }

    /// <summary>
    ///     Used to extract mission from log file - both sent or received
    /// </summary>
    /// <param name="buffer">packet</param>
    private void getInfoFromStream(ref MAVLinkMessage buffer, byte sysid, byte compid)
    {
        if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_COUNT)
        {
            // clear old
            var wp = buffer.ToStructure<mavlink_mission_count_t>();

            if (wp.target_system == YkiSistemId)
            {
                wp.target_system = sysid;
                wp.target_component = compid;
            }

            MAVlist[wp.target_system, wp.target_component].wps.Clear();
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ITEM)
        {
            var wp = buffer.ToStructure<mavlink_mission_item_t>();

            if (wp.target_system == YkiSistemId)
            {
                wp.target_system = sysid;
                wp.target_component = compid;
            }

            if (wp.current == 2)
                // guide mode wp
                MAVlist[wp.target_system, wp.target_component].GuidedMode = wp;
            else
                MAVlist[wp.target_system, wp.target_component].wps[wp.seq] = wp;

            //Console.WriteLine("WP # {7} cmd {8} p1 {0} p2 {1} p3 {2} p4 {3} x {4} y {5} z {6}", wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, wp.seq, wp.command);
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.MISSION_ITEM_INT)
        {
            var wp = buffer.ToStructure<mavlink_mission_item_int_t>();

            if (wp.target_system == YkiSistemId)
            {
                wp.target_system = sysid;
                wp.target_component = compid;
            }

            if (wp.current == 2)
                // guide mode wp
                MAVlist[wp.target_system, wp.target_component].GuidedMode =
                    (Locationwp)wp;
            else
                MAVlist[wp.target_system, wp.target_component].wps[wp.seq] =
                    (Locationwp)wp;

            //Console.WriteLine("WP INT # {7} cmd {8} p1 {0} p2 {1} p3 {2} p4 {3} x {4} y {5} z {6}", wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, wp.seq, wp.command);
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.SET_POSITION_TARGET_GLOBAL_INT)
        {
            var setpos =
                buffer.ToStructure<mavlink_set_position_target_global_int_t>();

            MAVlist[setpos.target_system, setpos.target_component].GuidedMode =
                (Locationwp)setpos;

            //Console.WriteLine("SET_POSITION_TARGET_GLOBAL_INT x {0} y {1} z {2} frame {3}", setpos.lat_int/1e7, setpos.lon_int/1e7, setpos.alt, setpos.coordinate_frame);
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.RALLY_POINT)
        {
            var rallypt = buffer.ToStructure<mavlink_rally_point_t>();

            if (rallypt.target_system == YkiSistemId)
            {
                rallypt.target_system = sysid;
                rallypt.target_component = compid;
            }

            MAVlist[rallypt.target_system, rallypt.target_component].rallypoints[rallypt.idx] =
                    new mavlink_mission_item_int_t(0, 0, 0, 0, rallypt.lat, rallypt.lng, rallypt.alt, rallypt.idx,
                        (ushort)MAV_CMD.RALLY_POINT, rallypt.target_system, rallypt.target_component,
                        (byte)MAV_FRAME.GLOBAL_RELATIVE_ALT, 0, 0, (byte)MAV_MISSION_TYPE.RALLY);

            //Console.WriteLine("RP # {0} {1} {2} {3} {4}", rallypt.idx, rallypt.lat, rallypt.lng, rallypt.alt, rallypt.break_alt);
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.CAMERA_FEEDBACK)
        {
            var camerapt = buffer.ToStructure<mavlink_camera_feedback_t>();

            if (MAVlist[sysid, compid].camerapoints.Count == 0 ||
                MAVlist[sysid, compid].camerapoints.Last().time_usec != camerapt.time_usec)
            {
                MAVlist[sysid, compid].camerapoints.RemoveAll(a =>
                    a.cam_idx * 256 + a.img_idx == camerapt.cam_idx * 256 + camerapt.img_idx);
                MAVlist[sysid, compid].camerapoints.Add(camerapt);
            }
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.FENCE_POINT)
        {
            var fencept = buffer.ToStructure<mavlink_fence_point_t>();

            if (fencept.target_system == YkiSistemId)
            {
                fencept.target_system = sysid;
                fencept.target_component = compid;
            }

            MAVlist[fencept.target_system, fencept.target_component].fencepoints[fencept.idx] =
                    new mavlink_mission_item_int_t(fencept.count - 1, 0, 0, 0, (int)(fencept.lat * 1e7),
                        (int)(fencept.lng * 1e7), 0, fencept.idx,
                        fencept.idx == 0
                            ? (ushort)MAV_CMD.FENCE_RETURN_POINT
                            : (ushort)MAV_CMD.FENCE_POLYGON_VERTEX_INCLUSION,
                        fencept.target_system, fencept.target_component, (byte)MAV_FRAME.GLOBAL_RELATIVE_ALT, 0, 0,
                        (byte)MAV_MISSION_TYPE.FENCE);
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.PARAM_VALUE)
        {
            var value = buffer.ToStructure<mavlink_param_value_t>();

            var st = Encoding.ASCII.GetString(value.param_id);

            var pos = st.IndexOf('\0');

            if (pos != -1) st = st.Substring(0, pos);

            MAVlist[sysid, compid].param_types[st] = (MAV_PARAM_TYPE)value.param_type;

            if (MAVlist[sysid, compid].apname == MAV_AUTOPILOT.ARDUPILOTMEGA &&
                buffer.compid != (byte)MAV_COMPONENT.MAV_COMP_ID_UDP_BRIDGE)
            {
                var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                MAVlist[sysid, compid].param[st] = new MAVLinkParam(st, BitConverter.GetBytes(value.param_value),
                    MAV_PARAM_TYPE.REAL32, (MAV_PARAM_TYPE)value.param_type);
            }
            else
            {
                var offset = Marshal.OffsetOf(typeof(mavlink_param_value_t), "param_value");
                MAVlist[sysid, compid].param[st] = new MAVLinkParam(st, BitConverter.GetBytes(value.param_value),
                    (MAV_PARAM_TYPE)value.param_type, (MAV_PARAM_TYPE)value.param_type);
            }

            MAVlist[sysid, compid].param.TotalReported = value.param_count;
        }
        else if (buffer.msgid == (byte)MAVLINK_MSG_ID.TIMESYNC)
        {
            var now_ns =
                (long)((DateTime.Now - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalMilliseconds *
                       1000000);

            var tsync = buffer.ToStructure<mavlink_timesync_t>();
            // tc1 - client
            // ts1 - server

            // system does not know the time
            if (tsync.tc1 == 0)
            {
                tsync.tc1 = now_ns;
                if (BaseStream != null && BaseStream.IsOpen)
                    paketGonder(tsync, buffer.sysid, buffer.compid);
            } // system knows the time 
            else if (tsync.tc1 > 0)
            {
                var offset_ns = (tsync.ts1 + now_ns - tsync.tc1 * 2) / 2;
                var dt = MAVlist[buffer.sysid, buffer.compid].time_offset_ns - offset_ns;

                if (Math.Abs(dt) > 10000000) // 10 millisecond skew
                {
                    MAVlist[buffer.sysid, buffer.compid].time_offset_ns = offset_ns; // hard-set it.
                }
                else
                {
                    var offset_avg_alpha = 0.6;
                    var avg = offset_avg_alpha * offset_ns +
                              (1.0 - offset_avg_alpha) * MAVlist[buffer.sysid, buffer.compid].time_offset_ns;
                    MAVlist[buffer.sysid, buffer.compid].time_offset_ns = (long)avg;
                }
            }
        }
    }

    #endregion

    public override string ToString()
    {
        if (BaseStream != null && BaseStream.IsOpen)
            return "MAV " + MAV.SistemId + " on " + BaseStream.PortName;

        return "MAV " + MAV.SistemId + " on Ice";
    }
}

public delegate void ProgressEventHandler(int percent, string status);