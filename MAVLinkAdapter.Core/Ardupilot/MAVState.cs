using System.Collections.Concurrent;
using System.Reflection;
using System.Runtime.Serialization;
using System.Text.Json.Serialization;

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.Serialization;

namespace MAVLinkAdapter.Core.Ardupilot
{
    /// <summary>
    /// MAVLink bağlantı durumunu tutan sınıf
    /// </summary>
    public class MAVState : MAVLink, IDisposable
    {

        [JsonIgnore]
        [IgnoreDataMember]
        public MavLinkArayuzu parent;

        public MAVState(MavLinkArayuzu mavLinkArayuzu, byte sistemId, byte bilesenId)
        {
            parent = mavLinkArayuzu;
            this.SistemId = sistemId;
            this.BilesenId = bilesenId;
            packetspersecond = new Dictionary<uint, double>();
            packetspersecondbuild = new Dictionary<uint, DateTime>();
            lastvalidpacket = DateTime.MinValue;
            sendlinkid = (byte)new Random().Next(256);
            signing = false;
            param = new MAVLinkParamList();
            packets = new Dictionary<uint, Queue<MAVLinkMessage>>();
            aptype = 0;
            apname = 0;
            recvpacketcount = 0;
            VersionString = "";
            SoftwareVersions = "";
            SerialString = "";
            FrameString = "";

            camerapoints.Clear();


            packetslost = 0f;
            packetsnotlost = 0f;
            packetlosttimer = DateTime.MinValue;
            cs.parent = this;
        }

        public float packetslost = 0;
        public float packetsnotlost = 0;
        public DateTime packetlosttimer = DateTime.MinValue;
        public float synclost = 0;


        // all
        public string VersionString { get; set; }
        // px4+ only
        public string SoftwareVersions { get; set; }
        // px4+ only
        public string SerialString { get; set; }
        // AC frame type
        public string FrameString { get; set; }

        public string Guid { get; set; }

        /// <summary>
        /// o anda bağlı olan MAV'ın statik küresel durumu
        /// </summary>
        public CurrentState cs = new CurrentState();

        private byte _sistemId;
        /// <summary>
        /// mavlink uzak sysid
        /// </summary>
        public byte SistemId
        {
            get { return _sistemId; }
            set { _sistemId = value; }
        }

        /// <summary>
        /// mavlink remove compid
        /// </summary>
        public byte BilesenId { get; set; }

        public byte linkid { get; set; }

        public byte sendlinkid { get; internal set; }

        public ulong timestamp { get; set; }

        internal byte[] signingKey;

        /// <summary>
        /// Giden paketleri imzalıyor ve gelen paket imzalarını kontrol ediyor muyuz?
        /// </summary>
        public bool signing { get; set; }

        /// <summary>
        /// gelen imzayı yoksay
        /// </summary>
        public bool signingignore { get; set; }

        /// <summary>
        /// mavlink 2 etkinleştir
        /// </summary>
        public bool mavlinkv2 = false;

        /// <summary>
        /// tüm parametre listesini tutar
        /// </summary>
        [JsonIgnore]
        [IgnoreDataMember]
        public MAVLinkParamList param { get; set; }
        [JsonIgnore]
        [IgnoreDataMember]
        public Dictionary<string, MAV_PARAM_TYPE> param_types = new Dictionary<string, MAV_PARAM_TYPE>();

        /// <summary>
        /// belirli bir türde alınan önceki bir paketin depolanması
        /// </summary>
        Dictionary<uint, Queue<MAVLinkMessage>> packets { get; set; }

        object packetslock = new object();

        public MAVLinkMessage getPacket(uint mavlinkid)
        {
            lock (packetslock)
            {
                if (packets.ContainsKey(mavlinkid))
                {
                    if (packets[mavlinkid].Count > 0)
                        return packets[mavlinkid].Dequeue();
                }
            }

            return null;
        }

        public void addPacket(MAVLinkMessage msg)
        {
            lock (packetslock)
            {
                // mevcut değilse kuyruk oluştur
                if (!packets.ContainsKey(msg.msgid))
                {
                    packets[msg.msgid] = new Queue<MAVLinkMessage>();
                }
                // bu mesaj sorgulanmıyorsa kuyruğu temizle
                while (packets[msg.msgid].Count > 5)
                {
                    packets[msg.msgid].Dequeue();
                }

                packets[msg.msgid].Enqueue(msg);
            }
        }

        public void clearPacket(uint mavlinkid)
        {
            lock (packetslock)
            {
                if (packets.ContainsKey(mavlinkid))
                {
                    packets[mavlinkid].Clear(); ;
                }
            }
        }

        public void Dispose()
        {
        }

        /// <summary>
        /// en son mavlink paketinin görülme zamanı
        /// </summary>
        public DateTime lastvalidpacket { get; set; }

        /// <summary>
        /// herhangi bir tek mesaj türünde saniye başına paketleri hesaplamak için kullanılır - akış hızı karşılaştırması için kullanılır
        /// </summary>
        [JsonIgnore]
        [IgnoreDataMember]
        public Dictionary<uint, double> packetspersecond { get; set; }

        /// <summary>
        /// bir paketin en son görüldüğü zaman
        /// </summary>
        [JsonIgnore]
        [IgnoreDataMember]
        public Dictionary<uint, DateTime> packetspersecondbuild { get; set; }

        /// <summary>
        /// mavlink ardupilot türü
        /// </summary>
        public MAV_TYPE aptype { get; set; }

        public MAV_AUTOPILOT apname { get; set; }

        public bool CANNode { get; set; } = false;

        public ap_product Product_ID
        {
            get
            {
                if (param.ContainsKey("INS_PRODUCT_ID")) return (ap_product)(float)param["INS_PRODUCT_ID"];
                return ap_product.AP_PRODUCT_ID_NONE;
            }
        }

        /// <summary>
        /// ardupilot atm yüklenenlerin anlık görüntüsü olarak kullanılır. - akıştan türetilmiştir
        /// </summary>
        public ConcurrentDictionary<int, mavlink_mission_item_t> wps = new ConcurrentDictionary<int, mavlink_mission_item_t>();
        
        public ConcurrentDictionary<int, mavlink_mission_item_int_t> rallypoints = new ConcurrentDictionary<int, mavlink_mission_item_int_t>();

        public ConcurrentDictionary<int, mavlink_mission_item_int_t> fencepoints = new ConcurrentDictionary<int, mavlink_mission_item_int_t>();

        public List<mavlink_camera_feedback_t> camerapoints = new List<mavlink_camera_feedback_t>();


        /// <summary>
        /// Guided mod wp konumunu tutar.
        /// </summary>
        public mavlink_mission_item_t GuidedMode = new mavlink_mission_item_t();
        

        internal int recvpacketcount = 0;
        public long time_offset_ns { get; set; }

        public override string ToString()
        {
            return SistemId.ToString();
        }
    }
}