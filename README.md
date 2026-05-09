# MAVLinkAdapter

MAVLink protokolüne sahip araçlarla (ArduPilot / PX4) iletişim kurmayı sağlayan **C# .NET kütüphanesi**.  
Türkçe isimlendirilmiş sade bir API ile bağlanma, arm/disarm, kalkış, iniş, konuma gitme, rota yükleme ve kumanda kanal baskılama işlemlerini birkaç satırda gerçekleştirmenizi sağlar.

---

## Çözüm Yapısı

```
MAVLinkAdapter.sln
├── MAVLinkAdapter.Core/          ← Ana kütüphane
│   ├── MavlinkAdapter.cs         → Ana giriş noktası / public API
│   ├── Ardupilot/                → ArduPilot bağlantı katmanı
│   ├── Communication/            → Serial, TCP, UDP transport
│   ├── Mavlink/                  → MAVLink protokol parser + mesaj tanımları
│   ├── Modeller/                 → Telemetri veri modelleri
│   └── Utilities/                → Yardımcı sınıflar (PointLatLngAlt, vs.)
└── MAVLinkAdapter.Console/       ← Örnek kullanım senaryoları
    └── Program.cs
```

---

## Hızlı Başlangıç

### Kurulum

```bash
# Çözümü derle
dotnet build MAVLinkAdapter.sln
```

### Basit Bağlantı (COM Port)

```csharp
using MAVLinkAdapter.Core;

MavlinkAdapter arac = new MavlinkAdapter();
arac.Baglan("com4", "115200");

Console.WriteLine("GPS: {0}", arac.Gps);
Console.WriteLine("Batarya: {0}", arac.Batarya);
Console.WriteLine("Mod: {0}", arac.Mod);
Console.WriteLine("Arm: {0}", arac.ArmEdildi);

arac.BaglantiyiKes();
```

### TCP Bağlantısı (SITL / Mission Planner)

```csharp
MavlinkAdapter arac = new MavlinkAdapter();
arac.Baglan("tcp:127.0.0.1:5763");
```

### UDP Bağlantısı

```csharp
arac.Baglan("udp:127.0.0.1:14550");    // UDP server
arac.Baglan("udpcl:127.0.0.1:14550");  // UDP client
```

---

## API Referansı

### Bağlantı

| Metot | Açıklama |
|-------|---------|
| `Baglan(string baglantiMetni, string baud, bool parametreleriAl)` | Araca bağlanır (COM, TCP, UDP) |
| `BaglantiyiKes()` | Bağlantıyı kapatır |

**Bağlantı formatları:**
```
"com4"                   → Serial COM4 (115200 baud varsayılan)
"tcp:127.0.0.1:5763"    → TCP
"udp:0.0.0.0:14550"     → UDP server
"udpcl:192.168.1.1:14550" → UDP client
```

---

### Uçuş Komutları

| Metot / Özellik | Açıklama |
|----------------|---------|
| `ArmEt(zorla, bekle, zamanAsimi)` | Aracı arm eder |
| `DisarmEt(zorla, bekle, zamanAsimi)` | Aracı disarm eder |
| `KalkisYap(irtifa, bekle, zamanAsimi, epsilon)` | Kalkış yapar (GUIDED + ARM otomatik) |
| `InisYap(bekle, zamanAsimi, epsilon)` | LAND moduna geçer |
| `KonumaGit(konum, bekle, zamanAsimi, epsilon)` | Belirtilen GPS konumuna gider |
| `ModGuncelle(yeniMod, bekle, zamanAsimi)` | Uçuş modunu değiştirir |
| `YawDegistir(derece, aracaGore, yawHizi, cw)` | Yaw açısını değiştirir |
| `YerHizi` | Yer hızını okur/yazar (m/s) |
| `HavaHizi` | Hava hızını okur/yazar (m/s) |

---

### Telemetri Özellikleri

| Özellik | Tür | Açıklama |
|---------|-----|---------|
| `Gps` | `GpsBilgisi` | Enlem, boylam, irtifa, GPS durumu |
| `Durum` | `DurumBilgisi` | Pitch, roll, yaw, hız |
| `Batarya` | `BataryaBilgisi` | Voltaj, akım, yüzde |
| `Ekf` | `EkfBilgisi` | EKF sağlık durumu |
| `Mod` | `string` | Aktif uçuş modu (GUIDED, AUTO, STABILIZE, vs.) |
| `ArmEdildi` | `bool` | Arm durumu |
| `ArmEdilebilirMi` | `bool` | GPS fix + EKF hazır kontrolü |

---

### Rota Yönetimi

```csharp
// Rota oluştur
List<Locationwp> rotaListesi = new List<Locationwp>();
rotaListesi.Add(new Locationwp().Set(0, 0, 5, (byte)MAVLink.MAV_CMD.TAKEOFF));
rotaListesi.Add(new Locationwp().Set(40.1842361, 28.9288487, 10, (byte)MAVLink.MAV_CMD.WAYPOINT));
rotaListesi.Add(new Locationwp().Set(40.1839144, 28.9283565, 10, (byte)MAVLink.MAV_CMD.WAYPOINT));
rotaListesi.Add(new Locationwp().Set(0, 0, 0, (byte)MAVLink.MAV_CMD.LAND));

// Araca yükle
arac.rotaBilgisi.HizliYukle(rotaListesi);

// Aracın hafızasındaki rotayı indir
List<Locationwp> mevcutRota = arac.rotaBilgisi.Indir(MAVLink.MAV_MISSION_TYPE.MISSION);
```

---

### Kumanda Kanal Baskılama

RC kumanda kanallarını yazılım üzerinden geçersiz kılmak için kullanılır:

```csharp
// Tek kanal baskıla
arac.KanalBaskilama["4"] = 1400;  // Yaw kanalı → 1400 PWM

// Birden fazla kanal
var yeniBaskılama = new KumandaKanalBaskilama();
yeniBaskılama["3"] = 2000;   // Gaz
yeniBaskılama["4"] = 1400;   // Yaw
arac.KanalBaskilama = yeniBaskılama;

// Tekrarlayıcıyı aktif et (sürekli gönderim)
arac.KanalBaskilama.TekrarlayiciAktif = true;
// ...görev...
arac.KanalBaskilama.TekrarlayiciAktif = false;
```

---

## Tam Örnek Senaryolar

### 1 — Araç Bilgilerini Oku

```csharp
MavlinkAdapter arac = new MavlinkAdapter();
arac.Baglan("com4", "115200");

Console.WriteLine($"GPS: {arac.Gps}");
Console.WriteLine($"Batarya: {arac.Batarya}");
Console.WriteLine($"Mod: {arac.Mod}");
Console.WriteLine($"Arm: {arac.ArmEdildi}");
Console.WriteLine($"EKF: {arac.Ekf.EkfOkMu(arac.ArmEdildi)}");

arac.BaglantiyiKes();
```

### 2 — Kalkış ve Konuma Gitme

```csharp
MavlinkAdapter arac = new MavlinkAdapter();
arac.Baglan("tcp:127.0.0.1:5763");

// Arm + GUIDED + Kalkış (5m) — hepsini tek satırda
arac.KalkisYap(5, bekle: true);

arac.YerHizi = 5;  // 5 m/s yer hızı

// Konuma git ve varmasını bekle
arac.KonumaGit(new PointLatLngAlt(40.1842361, 28.9288487, 10), bekle: true);

// İnişe geç
arac.InisYap(bekle: true);
arac.BaglantiyiKes();
```

### 3 — Rota Yükleme

```csharp
MavlinkAdapter arac = new MavlinkAdapter();
arac.Baglan("tcp:127.0.0.1:5763");

var rota = new List<Locationwp>
{
    new Locationwp().Set(0, 0, 5, (byte)MAVLink.MAV_CMD.TAKEOFF),
    new Locationwp().Set(40.1842361, 28.9288487, 10, (byte)MAVLink.MAV_CMD.WAYPOINT),
    new Locationwp().Set(40.1839144, 28.9283565, 10, (byte)MAVLink.MAV_CMD.WAYPOINT),
    new Locationwp().Set(0, 0, 0, (byte)MAVLink.MAV_CMD.LAND),
};

arac.rotaBilgisi.HizliYukle(rota);
```

---

## Mimari

```
MavlinkAdapter (public API)
        │
        ▼
MavLinkArayuzu (Ardupilot katmanı)
        │
        ├── ICommsSerial
        │       ├── SerialPort   → COM portu
        │       ├── TcpSerial    → TCP bağlantısı
        │       ├── UdpSerial    → UDP server
        │       └── UdpSerialConnect → UDP client
        │
        ├── MavlinkParse         → MAVLink v1/v2 paketi ayrıştırma
        │
        └── CurrentState         → Canlı telemetri durumu
                ├── GpsBilgisi
                ├── DurumBilgisi
                ├── BataryaBilgisi
                ├── EkfBilgisi
                └── ...
```

---

## Desteklenen Veri Modelleri

| Sınıf | İçerik |
|-------|--------|
| `GpsBilgisi` | Enlem, boylam, irtifa, GPS fix durumu, uydu sayısı |
| `DurumBilgisi` | Pitch, roll, yaw, yer hızı, hava hızı |
| `BataryaBilgisi` | Voltaj (V), akım (A), şarj yüzdesi |
| `EkfBilgisi` | EKF bayrakları, sağlık durumu |
| `ImuBilgisi` | İvme, jiroskop değerleri |
| `KumandaBilgisi` | RC kanal PWM değerleri |
| `MotorBilgisi` | Motor çıkış değerleri |
| `SonarBilgisi` | Menzil sensörü mesafesi |
| `BarometreBilgisi` | Basınç, sıcaklık, barometrik irtifa |
| `OpticFlowSensorBilgisi` | Optik akış sensörü verileri |
| `PidTuningBilgisi` | PID ayar değerleri |
| `RotaBilgisi` | Görev (mission) waypoint listesi |

---

## Gereksinimler

- .NET 6.0+
- Visual Studio 2022 veya Rider
- ArduPilot / PX4 tabanlı araç (gerçek donanım veya SITL simülatörü)

## SITL ile Test

```bash
# ArduCopter SITL başlat
sim_vehicle.py -v ArduCopter --console --map

# Varsayılan TCP portu: 5763
arac.Baglan("tcp:127.0.0.1:5763");
```
