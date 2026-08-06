# ETA ve Kalan Mesafe

ETA estimator aktif waypointi ve rota üzerindeki ilerlemeyi izler.

Waypoint şu iki koşuldan biriyle geçilmiş sayılır:
- araç kabul yarıçapına girmiştir
- araç bacak sonunu izdüşüm olarak geçmiştir

Kalan mesafe güncel konumdan aktif waypoint mesafesi ile sonraki bacakların toplamıdır. İlerleme hızı yer hızının aktif bacak yönündeki izdüşümüdür ve 3 s zaman sabitli filtreyle süzülür.

Anlık ETA tanılama için kullanılır. Zaman kontrolündeki ana ETA ise kalan rotayı hava hızı ve rüzgârla bacak bacak hesaplar. Böylece waypoint dönüşünde düşen anlık ilerleme hızı sahte gecikme üretmez.

Kod konumu:
- `estimation/eta_estimator.py`
- `mission_manager.py`
