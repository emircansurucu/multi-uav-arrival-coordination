# Varış Kontrolü ve Algoritmalar

Amaç üç aracın hedefe HA-1, HA-2 ve HA-3 sırasıyla 20 saniye arayla varmasıdır. Büyük zaman farkları yerde, küçük hatalar uçuş sırasında giderilir.

## Ortak zamanlama

Araç `i` için en erken ulaşılabilir varış `E_i`, araçlar arası fark `Delta` olsun. Ortak çıpa:
- A = max(E_i - (i - 1) * Delta)
- T_i = A + (i - 1) * Delta

Her agent aynı girdilerle bu hesabı yaptığı için ayrıca lider seçilmez. Plan yalnız ileri taşınabilir.

## Kalkış gecikmesi

Kalkış anı planlanan varıştan rüzgâr düzeltilmiş rota süresi çıkarılarak bulunur:

kalkış zamanı = planlanan varış - tahmini uçuş süresi

HA-3 daha kısa rotaya sahip olduğu için uzun süre yerde bekler. Böylece zaman farkını havada loiter veya gereksiz yol uzatmayla tüketmez.

## Rüzgâr kestirimi

Rüzgâr, yer ve hava hızı farkından hesaplanır:

rüzgâr hızı = yer hızı - hava hızı

Hava hızı önce quaternion yönelimiyle gövde FLU ekseninden ENU eksenine çevrilir. Doğu ve kuzey bileşenleri 2,5 s zaman sabitli filtreyle süzülür. Kestirim 7,5 s dolmadan geçerli sayılmaz.

## Rüzgâr düzeltilmiş rota süresi

Rota bacaklara ayrılır. Her bacakta rüzgârın ileri ve çapraz bileşenleri bulunur. Sabit hava hızında bacak yönündeki yer hızı:

Vg = sqrt(Va^2 - w_yan^2) + w_ileri

Toplam süre bütün bacak sürelerinin toplamıdır. Bu hesap dönüş anındaki tek bir hız ölçümüne göre ETA üretmekten daha kararlıdır.

## Hız kontrolü

Zaman hatası model ETA ile plana kalan süre arasındaki farktır:
zaman hatası = ETA - plana kalan süre

Pozitif hata geç kalmayı, negatif hata erken varmayı gösterir. Hava hızı 13 ile 28 m/s arasında tutulur. Komut değişimi 1,5 m/s² ile sınırlıdır ve ±0,5 s ölü bant kullanılır.

## E ve L sınırları

`E` en erken, `L` en geç ulaşılabilir varışı gösterir. Hedef zamanın kontrol edilebilir kalması için:

E <= hedef zaman <= L

Hesap 0 ile 10 m/s arasındaki rüzgâr adaylarını, 30 derecelik yön adımlarını ve hız rampalarını dikkate alır.

## Son yasal kapı ve terminal rezerv

Hedefe 2 km kala havada bekleme yapılmaz. Gerekli bekleme bu sınırın dışındaki son rota girişinde yapılır. Kapı geçildikten sonra erken ve geç kalma rezervleri izlenir. Erken rezerv azalırsa asgari, geç rezerv azalırsa azami hava hızı seçilir.

## Varış tespiti

Varış hedef merkezindeki 5 m çembere giriş anıdır. İki telemetri örneği arasındaki çember kesişimi bulunur ve zaman doğrusal olarak interpolasyonla hesaplanır. Varış bir kez kaydedilir ve ardından RTL gönderilir.

## S-manevrası

S-manevrası geliştirme sırasında denendi ancak nihai çözümden çıkarıldı. Kalkış gecikmesi, hız kontrolü ve son yasal kapı daha sade ve ölçülebilir bir çözüm verdi.
