# Varış Tespiti

Varış hedef merkezindeki 5 m kabul çemberine ilk giriş anıdır. Yalnız en yakın telemetri örneğini kullanmak yerine iki konum arasındaki uçuş parçası çemberle kesiştirilir.

Uçuş parçası:
- p(f) = p0 + f * (p1 - p0)

Çemberi kesen `f` oranı bulunduğunda varış zamanı iki monotonic zaman damgası arasında doğrusal olarak hesaplanır.

Dedektör ayrıca koşu boyunca hedefe en yakın mesafeyi tutar. Varış ikinci kez üretilmez. Ardından görev RTL durumuna geçer.

Kod konumu:
- `estimation/arrival_detector.py`
- `estimation/geodesy.py`
