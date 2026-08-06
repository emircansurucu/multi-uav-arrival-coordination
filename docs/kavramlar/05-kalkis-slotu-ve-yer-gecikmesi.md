# Kalkış Zamanı ve Yer Gecikmesi

Araçların rota süreleri farklıdır. Daha kısa rotaya sahip araçlar hedefe erken gitmemek için yerde bekler.

Temel hesap:
- kalkış zamanı = planlanan varış - rüzgâr düzeltilmiş uçuş süresi

Bekleme sıra numarasının sabit bir katı değildir. Her aracın rota uzunluğu, yönü ve rüzgâr altında beklenen uçuş süresi farklıdır.

Nominal örnekte rota süreleri yaklaşık HA-1 için 533 s, HA-2 için 538 s ve HA-3 için 405 s oldu. Buna göre HA-2 yaklaşık 15 s, HA-3 yaklaşık 168 s yerde bekledi.

Araç yerdeyken peer planı veya geçerli rüzgâr değişirse kalkış zamanı yeniden hesaplanır. Arm işlemi ancak bu zaman geldiğinde başlar.

Kod konumu:
- `mission_manager.py`
- `coordination/arrival_schedule.py`
Bu yöntem havada gereksiz loiter ve yol uzatmayı azaltır.
