# Rüzgâr Düzeltilmiş Rota Süresi

Rota her waypoint çifti arasında ayrı bacak olarak hesaplanır. Rüzgârın bacak yönündeki ve bacağı kesen bileşenleri bulunur.

Hava hızı `Va`, çapraz rüzgâr `w_yan`, ileri rüzgâr `w_ileri` ise bacak yönündeki yer hızı:

Vg = sqrt(Va^2 - w_yan^2) + w_ileri

Her bacağın süresi uzunluğun `Vg` değerine bölünmesiyle bulunur. Toplam rota süresi bütün bacakların toplamıdır.

Çapraz rüzgâr uçağın yengeç açısıyla uçmasına neden olduğu için ileri hızı da azaltır. Bu nedenle yalnız karşı veya kuyruk rüzgâr bileşenine bakmak yeterli değildir.

Kod konumu:
- `estimation/wind_estimator.py`
- `mission_manager.py`