# Rüzgâr Profili ve Gerçekçilik

Rüzgâr senaryosu ayrı bir MAVLink bağlantısıyla üç SITL aracına aynı anda uygulanır. Bu nedenle araçlar arasında ilerleyen fiziksel bir rüzgâr cephesi modellenmez.

Ana profiller:
- sakin hava
- sabit 8 m/s rüzgâr
- yönü ve şiddeti zamanla değişen variable profil
- kabul doğrulamasından ayrı extreme profil

`SIM_WIND_TC` rüzgâr geçişini yumuşatır. Variable profil 30 derecelik yön adımı ve 60 s zaman sabitiyle yaklaşık 0,5 derece/s tepe dönüş hızı üretir. Extreme profil sistem sınırını görmek için daha hızlı değişir.

Profil deterministiktir. Aynı senaryo tekrarlandığında aynı zamanlarda aynı parametreler gönderilir. Böylece koşular karşılaştırılabilir.

Kod konumu:
- `scripts/wind_profile.py`
- `oasy_bringup/params/wind/steady.parm`
- `oasy_bringup/params/wind/variable.parm`

Not: Oluşturduğum SITL rüzgârı gerçek atmosferin uzamsal türbülansını ve yerel değişimini tam olarak temsil etmez.
