# Jeodezi

Uzun mesafeler GeographicLib WGS84 ters jeodezik çözümüyle hesaplanır. Rota sapması ve çember kesişimi gibi kısa mesafeli işlemlerde hedef veya bacak başlangıcı merkezli yerel doğu-kuzey düzlemi kullanılır.

Temel işlemler:

- iki koordinat arasındaki mesafe ve kerteriz
- koordinatı yerel doğu-kuzey düzlemine çevirme
- noktanın rota parçasına en kısa uzaklığı
- doğru parçasının hedef çemberine giriş oranı
- rotanın 2 km çemberine son dış giriş noktası

Rota sapması sonsuz doğruya değil sınırlı bacak parçasına göre ölçülür. Bu ayrım waypoint geçildikten sonra yanlış düşük sapma hesaplanmasını önler.

Kod konumu:
- `estimation/geodesy.py`

