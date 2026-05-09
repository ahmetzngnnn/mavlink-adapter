namespace MAVLinkAdapter.Core.Utilities;

public interface IMAVLinkArayuzu
{
    int SeciliSistemId { get; set; }
    int SeciliBilesenId { get; set; }

    void paketGonder(object indata, int sysid, int compid);

    KeyValuePair<MAVLink.MAVLINK_MSG_ID, Func<MAVLink.MAVLinkMessage, bool>> PaketTuruneAboneOl(
        MAVLink.MAVLINK_MSG_ID type,
        Func<MAVLink.MAVLinkMessage, bool> function, bool exclusive = false);

    void PaketTurunuAboneliktenCik(KeyValuePair<MAVLink.MAVLINK_MSG_ID, Func<MAVLink.MAVLinkMessage, bool>> item);
    void PaketTurunuAboneliktenCik(MAVLink.MAVLINK_MSG_ID msgtype, Func<MAVLink.MAVLinkMessage, bool> item);
}

public interface IMAVLinkArayuzuKayitOkuma
{
    BinaryReader logplaybackfile { get; set; }
    bool logreadmode { get; set; }

    DateTime sonKayitOkuma { get; set; }

    MAVLink.MAVLinkMessage paketOku();
}