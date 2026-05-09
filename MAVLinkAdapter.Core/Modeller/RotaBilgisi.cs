using log4net;
using MAVLinkAdapter.Core.Ardupilot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Xml;

namespace MAVLinkAdapter.Core.Modeller
{
    /// <summary>
    /// Aracın yüklenmiş rota bilgilerine erişim sağlar.
    /// </summary>
    public class RotaBilgisi
    {
        private MavlinkAdapter _arac;

        public RotaBilgisi(MavlinkAdapter arac)
        {
            _arac = arac;
        }

        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Aracın EEPROM hafızasında bulunan rota bilgilerini liste olarak döndürür.
        /// </summary>
        /// <param name="type"> İstenilen rota tipi.(sadece görev rota bilgisi için MISSION, sadece fence rota bilgisi için FENCE, her ikisi için ALL)</param>
        /// <returns> Rota bilgilerini içeren Locationwp listesi</returns>
        public List<Locationwp> Indir(MAVLink.MAV_MISSION_TYPE type = MAVLink.MAV_MISSION_TYPE.ALL)
        {
            return Indir(_arac.ComPort, _arac.ComPort.MAV.SistemId, _arac.ComPort.MAV.BilesenId, type).GetAwaiter().GetResult();
        }
        private static async Task<List<Locationwp>> Indir(MavLinkArayuzu port, byte sysid, byte compid, MAVLink.MAV_MISSION_TYPE type, Action<int, string> progress = null)
        {
            List<Locationwp> commandlist = new List<Locationwp>();

            try
            {
                if (!port.BaseStream.IsOpen)
                {
                    throw new Exception("Please Connect First!");
                }

                bool use_supportfence = (port.MAVlist[sysid, compid].cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FENCE) > 0;

                bool use_int = (port.MAVlist[sysid, compid].cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT) > 0;

                if (!use_supportfence && type == MAVLink.MAV_MISSION_TYPE.FENCE)
                    throw new Exception("Fence mission items not supported, please use legacy geofence");

                if (!use_supportfence && !port.MAVlist[sysid, compid].mavlinkv2 && type != MAVLink.MAV_MISSION_TYPE.MISSION)
                {
                    throw new Exception("Mission type only supported under mavlink2");
                }

                progress?.Invoke(0, "Getting WP count");

                log.Info("Getting WP # " + type);

                int cmdcount = await port.getWPCountAsync(sysid, compid, type).ConfigureAwait(false);

                for (ushort a = 0; a < cmdcount; a++)
                {
                    log.Info("Getting WP" + a);
                    progress?.Invoke((a * 100) / cmdcount, "Getting WP " + a);
                    commandlist.Add(await port.getWPAsync(sysid, compid, a, type).ConfigureAwait(false));
                }

                port.setWPACK(sysid, compid, type);

                progress?.Invoke(100, "Done");

                log.Info("Done " + type);
            }
            catch (Exception ex)
            {
                log.Error(ex);
                throw;
            }

            return commandlist;
        }


        /// <summary>
        /// Liste içerisindeki rota bilgilerini araca tek tek yükler.
        /// </summary>
        /// <param name="komutlar"> Rota bilgilerini içeren Locationwp listesi</param>
        public void Yukle(List<Locationwp> komutlar)
        {
            Locationwp home = _arac.ComPort.HomeKonumuGetir();

            if (_arac.ComPort.MAV.apname == MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA)
                komutlar.Insert(0, home);

            Task.Run(async () =>
            {
                await Yukle(_arac.ComPort, _arac.ComPort.MAV.SistemId, _arac.ComPort.MAV.BilesenId, MAVLink.MAV_MISSION_TYPE.MISSION,
                    komutlar).ConfigureAwait(false);

                try
                {
                    await _arac.ComPort.getHomePositionAsync((byte)_arac.ComPort.SeciliSistemId,
                        (byte)_arac.ComPort.SeciliBilesenId).ConfigureAwait(false);
                }
                catch (Exception ex2)
                {
                    log.Error(ex2);
                    try
                    {
                        await _arac.ComPort.getWPAsync((byte)_arac.ComPort.SeciliSistemId,
                            (byte)_arac.ComPort.SeciliBilesenId, 0);
                    }
                    catch (Exception ex3)
                    {
                        log.Error(ex3);
                    }
                }
            }).GetAwaiter().GetResult();
        }


        /// <summary>
        /// Liste içerisindeki rota bilgilerini araca hizli bir şekilde tek defada yükler.
        /// </summary>
        /// <param name="komutlar"> Rota bilgilerini içeren Locationwp listesi</param>
        public void HizliYukle(List<Locationwp> komutlar)
        {
            var totalwpcountforupload = (ushort)(komutlar.Count + 1);
            var reqno = 0;
            MAVLink.MAV_MISSION_RESULT result = MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;

            var sub1 = _arac.ComPort.PaketTuruneAboneOl(MAVLink.MAVLINK_MSG_ID.MISSION_ACK,
                message =>
                {
                    var data = ((MAVLink.mavlink_mission_ack_t)message.data);
                    var ans = (MAVLink.MAV_MISSION_RESULT)data.type;
                    if (_arac.ComPort.MAV.SistemId != message.sysid &&
                        _arac.ComPort.MAV.BilesenId != message.compid)
                        return true;
                    // check this gcs sent it
                    if (data.target_system != MavLinkArayuzu.YkiSistemId)
                        return true;
                    result = ans;
                    Console.WriteLine("MISSION_ACK " + ans + " " + data.ToString());
                    return true;
                });

            var sub2 = _arac.ComPort.PaketTuruneAboneOl(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST,
                message =>
                {
                    var data = ((MAVLink.mavlink_mission_request_t)message.data);
                    // check what we sent is what the message is.
                    if (_arac.ComPort.MAV.SistemId != message.sysid &&
                        _arac.ComPort.MAV.BilesenId != message.compid)
                        return true;
                    // check this gcs sent it
                    if (data.target_system != MavLinkArayuzu.YkiSistemId)
                        return true;
                    reqno = data.seq;
                    Console.WriteLine("MISSION_REQUEST " + reqno + " " + data.ToString());
                    return true;
                });

            var sub3 = _arac.ComPort.PaketTuruneAboneOl(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_INT,
                message =>
                {
                    var data = ((MAVLink.mavlink_mission_request_int_t)message.data);
                    // check what we sent is what the message is.
                    if (_arac.ComPort.MAV.SistemId != message.sysid &&
                        _arac.ComPort.MAV.BilesenId != message.compid)
                        return true;
                    // check this gcs sent it
                    if (data.target_system != MavLinkArayuzu.YkiSistemId)
                        return true;
                    reqno = data.seq;
                    Console.WriteLine("MISSION_REQUEST_INT " + reqno + " " + data.ToString());
                    return true;
                });

            _arac.ComPort.setWPTotal(totalwpcountforupload);

            // define the home point
            Locationwp home = _arac.ComPort.HomeKonumuGetir();

            // define the default frame.
            MAVLink.MAV_FRAME frame = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT;


            komutlar.Insert(0, home);

            // process commandlist to the mav
            for (var a = 0; a < komutlar.Count; a++)
            {
                if (a % 10 == 0 && a != 0)
                {
                    var start = DateTime.Now;
                    while (true)
                    {

                        if (reqno == a)
                        {
                            // all received
                            break;
                        }

                        if (start.AddSeconds(1.1) < DateTime.Now)
                        {
                            // do next 10 starting at reqno
                            a = reqno;
                            break;
                        }

                        if (result == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE)
                            Thread.Sleep(500);

                        if (result == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ERROR)
                        {
                            // resend for partial upload
                            _arac.ComPort.setWPPartialUpdate((ushort)(reqno), totalwpcountforupload);
                            a = reqno;
                            break;
                        }

                        if (result == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE)
                        {
                            log.Error("Upload failed, please reduce the number of wp's");
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub1);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub2);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub3);
                            return;
                        }

                        if (result == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID)
                        {
                            log.Error("Upload failed, mission was rejected byt the Mav,\n item had a bad option wp# " +
                                      a +
                                      " " +
                                      result);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub1);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub2);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub3);
                            return;
                        }

                        if (result != MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                        {
                            log.Error("Upload wps failed " + reqno +
                                      " " + Enum.Parse(typeof(MAVLink.MAV_MISSION_RESULT),
                                          result.ToString()));
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub1);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub2);
                            _arac.ComPort.PaketTurunuAboneliktenCik(sub3);
                            return;
                        }

                        System.Threading.Thread.Sleep(10);
                    }
                }

                var loc = komutlar[a];

                // make sure we are using the correct frame for these commands
                if (loc.id < (ushort)MAVLink.MAV_CMD.LAST || loc.id == (ushort)MAVLink.MAV_CMD.DO_SET_HOME)
                {
                    frame = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT;
                }

                MAVLink.mavlink_mission_item_int_t req = new MAVLink.mavlink_mission_item_int_t();

                req.target_system = _arac.ComPort.MAV.SistemId;
                req.target_component = _arac.ComPort.MAV.BilesenId;

                req.command = loc.id;

                req.current = 0;
                req.autocontinue = 1;

                req.frame = (byte)frame;
                if (loc.id == (ushort)MAVLink.MAV_CMD.DO_DIGICAM_CONTROL ||
                    loc.id == (ushort)MAVLink.MAV_CMD.DO_DIGICAM_CONFIGURE)
                {
                    req.y = (int)(loc.lng);
                    req.x = (int)(loc.lat);
                }
                else
                {
                    req.y = (int)(loc.lng * 1.0e7);
                    req.x = (int)(loc.lat * 1.0e7);
                }

                req.z = (float)(loc.alt);

                req.param1 = loc.p1;
                req.param2 = loc.p2;
                req.param3 = loc.p3;
                req.param4 = loc.p4;

                req.seq = (ushort)a;

                log.Info("WP no " + a + " " + req.ToString());


                _arac.ComPort.paketGonder(req, _arac.ComPort.MAV.SistemId, _arac.ComPort.MAV.BilesenId);
            }

            _arac.ComPort.PaketTurunuAboneliktenCik(sub1);
            _arac.ComPort.PaketTurunuAboneliktenCik(sub2);
            _arac.ComPort.PaketTurunuAboneliktenCik(sub3);

            _arac.ComPort.setWPACK();

            _ = _arac.ComPort.getHomePositionAsync((byte)_arac.ComPort.SeciliSistemId,
                (byte)_arac.ComPort.SeciliBilesenId);

        }

        private static async Task Yukle(MavLinkArayuzu port, byte sysid, byte compid, MAVLink.MAV_MISSION_TYPE type, List<Locationwp> commandlist, Action<int, string> progress = null)
        {
            try
            {
                if (!port.BaseStream.IsOpen)
                {
                    throw new Exception("Please connect first!");
                }

                bool use_supportfence = (port.MAVlist[sysid, compid].cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FENCE) > 0;

                bool use_int = (port.MAVlist[sysid, compid].cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT) > 0;

                if (!use_supportfence && type == MAVLink.MAV_MISSION_TYPE.FENCE)
                    throw new Exception("Fence mission items not supported, please use legacy geofence");

                if (!use_supportfence && !port.MAVlist[sysid, compid].mavlinkv2 && type != MAVLink.MAV_MISSION_TYPE.MISSION)
                {
                    throw new Exception("Mission type only supported under mavlink2");
                }

                int a;

                await port.setWPTotalAsync(sysid, compid, (ushort)commandlist.Count, type).ConfigureAwait(false);

                // process commandlist to the mav
                for (a = 0; a < commandlist.Count; a++)
                {
                    var temp = commandlist[a];

                    // handle current wp upload number
                    int uploadwpno = a;

                    progress?.Invoke(((a * 100) / commandlist.Count), "Uploading WP " + a);

                    // try send the wp
                    MAVLink.MAV_MISSION_RESULT ans = await port.setWPAsync(sysid, compid, temp, (ushort)(uploadwpno), (MAVLink.MAV_FRAME)temp.frame, 0, 1, use_int, type).ConfigureAwait(false);

                    // we timed out while uploading wps/ command wasnt replaced/ command wasnt added
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ERROR)
                    {
                        // resend for partial upload
                        await port.setWPPartialUpdateAsync(sysid, compid, (ushort)(uploadwpno), (ushort)commandlist.Count, type).ConfigureAwait(false);
                        // reupload this point.
                        ans = await port.setWPAsync(sysid, compid, temp, (ushort)(uploadwpno),
                            (MAVLink.MAV_FRAME)temp.frame, 0, 1, use_int, type).ConfigureAwait(false);
                    }

                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE)
                    {
                        throw new Exception("Upload failed, please reduce the number of wp's");
                    }
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID)
                    {
                        throw new Exception(
                            "Upload failed, mission was rejected by the Mav,\n item had a bad option wp# " + a + " " +
                            ans);
                    }
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE)
                    {
                        // invalid sequence can only occur if we failed to see a response from the apm when we sent the request.
                        // or there is io lag and we send 2 mission_items and get 2 responces, one valid, one a ack of the second send

                        // the ans is received via mission_ack, so we dont know for certain what our current request is for. as we may have lost the mission_request

                        // get requested wp no - 1;
                        try
                        {
                            a = await port.getRequestedWPNoAsync(sysid, compid).ConfigureAwait(false) - 1;
                        }
                        catch
                        {
                            // resend for partial upload
                            await port.setWPPartialUpdateAsync(sysid, compid, (ushort)(uploadwpno), (ushort)commandlist.Count, type).ConfigureAwait(false);
                            // reupload this point.
                        }

                        continue;
                    }
                    if (ans != MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                    {
                        throw new Exception("Upload " + type.ToString() + " failed " + Enum.Parse(typeof(MAVLink.MAV_CMD), temp.id.ToString()) +
                                            " " + Enum.Parse(typeof(MAVLink.MAV_MISSION_RESULT), ans.ToString()));
                    }
                }

                port.setWPACK(sysid, compid, type);

            }
            catch (Exception ex)
            {
                log.Error(ex);
                throw;
            }
        }

        private static async Task TekTekYukle(MavLinkArayuzu port, byte sysid, byte compid, MAVLink.MAV_MISSION_TYPE type, List<Locationwp> commandlist, ushort start, Action<int, string> progress = null)
        {
            try
            {
                if (!port.BaseStream.IsOpen)
                {
                    throw new Exception("Please connect first!");
                }

                bool use_supportfence = (port.MAVlist[sysid, compid].cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FENCE) > 0;

                bool use_int = (port.MAVlist[sysid, compid].cs.capabilities & (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT) > 0;

                if (!use_supportfence && type == MAVLink.MAV_MISSION_TYPE.FENCE)
                    throw new Exception("Fence mission items not supported, please use legacy geofence");

                if (!use_supportfence && !port.MAVlist[sysid, compid].mavlinkv2 && type != MAVLink.MAV_MISSION_TYPE.MISSION)
                {
                    throw new Exception("Mission type only supported under mavlink2");
                }

                int a;

                await port.setWPPartialUpdateAsync(sysid, compid, start, (ushort)(start + commandlist.Count), type)
                    .ConfigureAwait(false);

                // process commandlist to the mav
                for (a = 0; a < commandlist.Count; a++)
                {
                    var temp = commandlist[a];

                    // handle current wp upload number
                    int uploadwpno = start + a;

                    progress?.Invoke(((a * 100) / commandlist.Count), "Uploading WP " + a);

                    // try send the wp
                    MAVLink.MAV_MISSION_RESULT ans = await port.setWPAsync(sysid, compid, temp, (ushort)(uploadwpno), (MAVLink.MAV_FRAME)temp.frame, 0, 1, use_int, type).ConfigureAwait(false);

                    // we timed out while uploading wps/ command wasnt replaced/ command wasnt added
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ERROR)
                    {
                        // resend for partial upload
                        await port.setWPPartialUpdateAsync(sysid, compid, (ushort)(uploadwpno), (ushort)commandlist.Count, type).ConfigureAwait(false);
                        // reupload this point.
                        ans = await port.setWPAsync(sysid, compid, temp, (ushort)(uploadwpno),
                            (MAVLink.MAV_FRAME)temp.frame, 0, 1, use_int, type).ConfigureAwait(false);
                    }

                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE)
                    {
                        throw new Exception("Upload failed, please reduce the number of wp's");
                    }
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID)
                    {
                        throw new Exception(
                            "Upload failed, mission was rejected by the Mav,\n item had a bad option wp# " + a + " " +
                            ans);
                    }
                    if (ans == MAVLink.MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE)
                    {
                        // invalid sequence can only occur if we failed to see a response from the apm when we sent the request.
                        // or there is io lag and we send 2 mission_items and get 2 responces, one valid, one a ack of the second send

                        // the ans is received via mission_ack, so we dont know for certain what our current request is for. as we may have lost the mission_request

                        // get requested wp no - 1;
                        try
                        {
                            a = await port.getRequestedWPNoAsync(sysid, compid).ConfigureAwait(false) - 1;
                        }
                        catch
                        {
                            // resend for partial upload
                            await port.setWPPartialUpdateAsync(sysid, compid, (ushort)(uploadwpno), (ushort)commandlist.Count, type).ConfigureAwait(false);
                            // reupload this point.
                        }

                        continue;
                    }
                    if (ans != MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                    {
                        throw new Exception("Upload " + type.ToString() + " failed " + Enum.Parse(typeof(MAVLink.MAV_CMD), temp.id.ToString()) +
                                            " " + Enum.Parse(typeof(MAVLink.MAV_MISSION_RESULT), ans.ToString()));
                    }
                }

                port.setWPACK(sysid, compid, type);

            }
            catch (Exception ex)
            {
                log.Error(ex);
                throw;
            }
        }

    }

    public enum altmode
    {
        Relative = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
        Absolute = MAVLink.MAV_FRAME.GLOBAL,
        Terrain = MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT
    }
}
