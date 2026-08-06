# Rüzgâr Kestirimi

Rüzgâr yer hızı ile hava hızı arasındaki vektör farkından hesaplanır:
- rüzgâr = yer hızı - hava hızı

AP_DDS hava hızını gövde FLU ekseninde verir. Yer hızı ENU eksenindedir. Bu nedenle hava hızı quaternion yönelimiyle ENU eksenine çevrilmeden çıkarma yapılmaz.

Pitch dönüşü özellikle tırmanışta önemlidir. Yalnız yaw kullanılırsa gövde ileri hızının azalan yatay bileşeni rüzgâr gibi görünür.

Doğu ve kuzey bileşenleri 2,5 s zaman sabitli birinci derece filtreyle süzülür. Kestirim 7,5 s dolmadan geçerli sayılmaz. Yön yerine vektör bileşenlerini filtrelemek 359 ve 1 derece arasındaki sarma sorununu önler.

Kod konumu:
- `estimation/wind_estimator.py`
- `autopilot_adapter/dds_telemetry.py`
