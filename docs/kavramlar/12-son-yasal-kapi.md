# Son Yasal Kapı

Hedefe 2 km kala havada bekleme yapılmaz. Son yasal kapı, görev rotasının bu çembere dışarıdan son giriş noktasıdır.

Kapıdan hedefe en erken süre `tE`, en geç süre `tL` ise kapıdan geçiş aralığı:

planlanan varış - tL <= kapı zamanı <= planlanan varış - tE

Araç kapıya bu aralıktan erken ulaşırsa GUIDED moda geçerek kapı çevresinde bekler. Hesaplanan bırakma anında AUTO moda dönüp görev rotasına devam eder.

Bekleme yalnız bir kez kullanılabilir. Hedef mesafesi veya rota sapması güvenlik payına yaklaşırsa bekleme iptal edilir.

Kod konumu:

- `mission_manager.py`
- `coordination/arrival_schedule.py`
- `estimation/geodesy.py`
