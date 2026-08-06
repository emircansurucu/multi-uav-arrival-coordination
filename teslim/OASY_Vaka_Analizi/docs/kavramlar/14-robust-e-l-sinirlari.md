# E ve L Ulaşılabilirlik Sınırları

`E` aracın en erken, `L` en geç ulaşabileceği varış anıdır. Planın uygulanabilir kalması için:

E <= planlanan varış <= L

Kalan rota yaklaşık 180 saniyelik zaman bloklarına ayrılır. Her blokta şu rüzgâr adayları taranır:

- hız 0 ile 10 m/s
- hız adımı 2 m/s
- yön adımı 30 derece

Azami hıza giden zincir en erken tarafı, asgari hıza giden zincir en geç tarafı oluşturur. Hız değişimi bir anda olmadığı için iki zincirde de 1,5 m/s² rampa hesaba katılır.

Zarf bütün adaylarda kullanılabilen ortak kontrol penceresini gösterir. Zarf fazla geniş seçilirse `E` değeri `L` değerini geçebilir ve ortak garanti kalmaz.

Kod konumu:
- `mission_manager.py`
- `estimation/wind_estimator.py`