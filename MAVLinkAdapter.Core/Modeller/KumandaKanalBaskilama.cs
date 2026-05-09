using MAVLinkAdapter.Core.Ardupilot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Modeller;

public class KumandaKanalBaskilama : Dictionary<string, int?>
{
    private bool _aktif;
    public readonly int _count = 8; // MAVLink tarafından belirlenen sabit
    Timer _timer; // tekrarlayan kanal baskılama gönderimi için kullanılacak

    /// <summary>
    /// Kanal baskılama gönderimlerinin belirli aralıklarla tekrarlanıp tekrarlanmayacağını aktifleştirir.
    /// </summary>
    public bool TekrarlayiciAktif
    {
        get => _timer != null;
        set
        {
            if (value)
            {
                if (_timer == null)
                {
                    _timer = new Timer(Tick, null, 0, TekrarlamaAraligi);
                }
            }
            else
            {
                if (_timer != null)
                {
                    _timer.Dispose();
                    _timer = null;
                }
            }
        }
    }

    // Kanal baskılama gönderimlerinin tekrarlanma aralığı (milisaniye)
    public int TekrarlamaAraligi { get; set; } = 1000;


    public KumandaKanalBaskilama()
    {

    }

    private void Tick(object? state)
    {
        if (TekrarlayiciAktif)
        {
            KanalBaskilamaGonder?.Invoke(this, EventArgs.Empty);
        }
    }

    public new int? this[string key]
    {
        get { return base[key]; }
        set
        {
            if (!int.TryParse(key, out int parsedKey) || parsedKey <= 0 || parsedKey > _count)
            {
                throw new KeyNotFoundException($"Gecersiz kanal indeksi {key}");
            }

            if (value == null)
            {
                if (this.ContainsKey(key))
                {
                    base.Remove(key);
                }
            }
            else
            {
                base[key] = value;
            }

            KanalBaskilamaGonder?.Invoke(this, EventArgs.Empty);
        }
    }

    public bool Aktif
    {
        get => _aktif;
        set => _aktif = value;
    }

    public event EventHandler KanalBaskilamaGonder;

}


