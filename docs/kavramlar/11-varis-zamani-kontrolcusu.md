# Varış Zamanı Kontrolcüsü

Kontrolcü küçük zaman hatalarını hava hızını değiştirerek düzeltir.

plana kalan süre = planlanan varış - şimdi
zaman hatası = model ETA - plana kalan süre

Pozitif hata aracın geç, negatif hata erken olduğunu gösterir. Gerekli hız mevcut hızın ETA oranıyla ölçeklenmesiyle bulunur.

Uygulanan sınırlar:

- hava hızı 13 ile 28 m/s arasında
- ölü bant ±0,5 s
- hız değişim sınırı 1,5 m/s²
- komut gönderme eşiği 0,1 m/s

Komut eşiği son gönderilen değere göre ölçülür. Böylece tek çevrimde küçük kalan rate limit adımları birikip gerçek komuta dönüşebilir.

Kod konumu:
- `control/arrival_controller.py`
- `mission_manager.py`
