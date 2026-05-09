using MAVLinkAdapter.Core;
using MAVLinkAdapter.Core.Ardupilot;
using MAVLinkAdapter.Core.Utilities;

MerhabaDrone();
//KalkisYap();
//Yonlendirme();
//RotaYukle();
//RotaYazdir();
//KonumaGitmeVeKanalBaskilama();

void RotaYukle()
{
    // Araca bağlan
    MavlinkAdapter arac = new MavlinkAdapter();
    arac.Baglan("tcp:127.0.0.1:5763");

    // Rota yükleme işlemi için rota listesi oluştur
    List<Locationwp> rotaListesi = new List<Locationwp>();

    // Rota listesine aracın bulunduğu noktadan kalkış yapabilmesi için 1 adet takeoff türünde rota noktası ekle. (araç 5 metre yükseğe kalkış yapacak)
    rotaListesi.Add(new Locationwp().Set(0, 0, 5, (byte)MAVLink.MAV_CMD.TAKEOFF));

    // Rota listesine 2 adet waypoint türünde rota noktası ekle
    rotaListesi.Add(new Locationwp().Set(40.1842361, 28.9288487, 10, (byte)MAVLink.MAV_CMD.WAYPOINT));
    rotaListesi.Add(new Locationwp().Set(40.1839144, 28.9283565, 10, (byte)MAVLink.MAV_CMD.WAYPOINT));

    // Rota listesine aracın bulunduğu noktaya iniş yapabilmesi için 1 adet land türünde rota noktası ekle.
    rotaListesi.Add(new Locationwp().Set(0, 0, 0, (byte)MAVLink.MAV_CMD.LAND));

    // Rota listesini araca yükle
    arac.rotaBilgisi.HizliYukle(rotaListesi);
}

void RotaYazdir()
{
    MavlinkAdapter arac = new MavlinkAdapter();
    arac.Baglan("tcp:127.0.0.1:5763");

    // Aracın hafızasındaki MISSION türündeki rota bilgisini getir.
    List <Locationwp> rotaListesi = arac.rotaBilgisi.Indir(MAVLink.MAV_MISSION_TYPE.MISSION);

    // Rota listesindeki tüm noktaları yazdır
    Console.WriteLine("Rota Listesi:");
    foreach (var rotaNoktasi in rotaListesi)
    {
        Console.WriteLine(rotaNoktasi);
    }
}

void Yonlendirme()
{
    MavlinkAdapter arac = new MavlinkAdapter();
    arac.Baglan("tcp:127.0.0.1:5763");

    // Tek satırda aracı arm et, modu değiştir, kalkış yap ve 5 metre yükseğe çıkması için bekle
    arac.KalkisYap(5, true);

    // Aracın Yere Göre hızını 5 m/s yap
    arac.YerHizi = 5;

    // Aracın Havaya Göre hızını 5 m/s yap
    arac.HavaHizi = 5;

    // Konuma git ve konuma varmasını bekle
    arac.KonumaGit(new PointLatLngAlt(40.1842361, 28.9288487, 10), true);
}

void KalkisYap()
{
    MavlinkAdapter arac = new MavlinkAdapter(); 
    arac.Baglan("tcp:127.0.0.1:5763");   

    // Aracı arm et ve arm olmasını tamamlanmasını bekle. (Öncesinde arm edilebilir mi kontrolü yapılır)
    var armEdildi = arac.ArmEt(true, true);
    Console.WriteLine(armEdildi ? "Arm edildi" : "Arm edilemedi");

    // Otonom moduna geç ve mod değişikliğini bekle
    arac.ModGuncelle("GUIDED");
    Console.WriteLine("GUIDED moduna geçildi");

    // Kalkış yap ve 5 metre yükseğe çıkması için bekle
    arac.KalkisYap(5, true);
    Console.WriteLine("Kalkış tamamlandı");


    // ya da yukarıdaki tüm komutlar yerine sadece aşağıdaki tek satırı kullanabilirsiniz. 
    //arac.KalkisYap(5, true);
}

void MerhabaDrone()
{
    //Araca Bağla
    MavlinkAdapter arac = new MavlinkAdapter();

    //Araca com port üzerinden bağlan
    arac.Baglan("com4","115200");

    // Bazı araç bilgilerini yazdır
    Console.WriteLine("Aracın bazı özellikleri:");
    Console.WriteLine("GPS: {0}", arac.Gps);
    Console.WriteLine("Batarya: {0}", arac.Batarya);
    Console.WriteLine("Durum: {0}", arac.Durum);
    Console.WriteLine("Mod : {0}", arac.Mod);
    Console.WriteLine($"Yer Hızı: {arac.YerHizi} m/s");
    Console.WriteLine($"Hava Hızı: {arac.HavaHizi} m/s");
    Console.WriteLine("EKF Hazır mı?: {0}", arac.Ekf.EkfOkMu(arac.ArmEdildi) ? "Evet" : "Hayır");
    Console.WriteLine("Arm Durumu: {0}", arac.ArmEdildi ? "Arm" : "Disarm");
    Console.WriteLine("Arm edilebilir mi? : {0}", arac.ArmEdilebilirMi ? "Evet" : "Hayır");


    //Programı kapatmadan önce bağlantı nesnesini kapat
    arac.BaglantiyiKes();

    Console.WriteLine("Tamamamlandı!");
}


void KonumaGitmeVeKanalBaskilama()
{
    Console.WriteLine("Merhaba, İyi Uçuşlar!");

    MavlinkAdapter mavlink = new MavlinkAdapter();

    var bagli = mavlink.Baglan("tcp:127.0.0.1:5763", parametreleriAl: false);

    if (!bagli)
    {
        Console.WriteLine("Bağlantı kurulamadı");
        return;
    }
    else
    {
        Console.WriteLine("Bağlantı kuruldu\n");
    }
    
    Console.WriteLine("\nGörev Başlatılıyor\n");

    Console.WriteLine("Arm ediliyor");
    // Aracı arm et ve arm olmasını tamamlanmasını bekle
    var armEdildi = mavlink.ArmEt(true, true);
    Console.WriteLine(armEdildi ? "Arm edildi" : "Arm edilemedi");

    Console.WriteLine("\nMod değiştiriliyor");
    // Otonom moduna geç ve mod değişikliğini bekle
    mavlink.ModGuncelle("GUIDED");
    Console.WriteLine("GUIDED moduna geçildi\n");

    Console.WriteLine("Kalkış yapılıyor");
    // Kalkış yap ve 5 metre yükseğe çıkması için bekle
    mavlink.KalkisYap(5, true);
    Console.WriteLine("Kalkış tamamlandı\n");

    Console.WriteLine("Konuma gidiliyor");
    // Konuma git ve konuma varmasını bekle
    mavlink.KonumaGit(new PointLatLngAlt(40.1842361, 28.9288487, 10), true);
    Console.WriteLine("Konuma gidildi\n");

    mavlink.KanalBaskilama["4"] = 1400;
    Console.WriteLine("Kumandanın 4. kanalı (yaw) 1400 pwm değeri ile baskılama ayarlandı");


    mavlink.KanalBaskilama["3"] = 2000;
    Console.WriteLine("Kumandanın 3. kanalı (throttle) 1400 pwm değeri ile baskılama ayarlandı");


    mavlink.KanalBaskilama.TekrarlayiciAktif = true;
    Console.WriteLine("Kumandanın baskılama tekrarlayıcısı aktifleştirildi\n");


    Console.WriteLine("Diğer Konuma gidiliyor");
    // Diğer konuma git ve konuma varmasını bekle
    mavlink.KonumaGit(new PointLatLngAlt(40.1839144, 28.9283565, 10), true);
    Console.WriteLine("Diğer Konuma gidildi\n");

    mavlink.KanalBaskilama.TekrarlayiciAktif = false;
    Console.WriteLine("Kanal baskılama tekrarlayıcısı durduruldu\n");

    Console.WriteLine("İniş yapılıyor");
    // İniş yap ve iniş tamamlanmasını bekle
    mavlink.InisYap(true);
    Console.WriteLine("İniş tamamlandı\n");

    Console.WriteLine("Görev Tamamlandı!");


    //Bağlantıyı kapat
    mavlink.BaglantiyiKes();
}