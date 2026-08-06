# Peer Yönetimi ve Tazelik

Peer manager diğer araçlardan gelen `VehicleStatus` mesajlarını saklar ve kullanılabilir olup olmadıklarını belirler.

Uygulanan kontroller:

- aracın kendi yayını dikkate alınmaz
- sıra numarası geriye giden mesaj reddedilir
- mesajın yerel alım zamanı kaydedilir
- kayıp veya görevi bitmiş araç rüzgâr kaynağı olmaz

Güncellik sınıfları:
- Mesaj yaşı 0 ile 2 s arasonda ise taze
- 2 ile 5 s ise eski
- 5 s üzeri ise kayıp olarak sınıflandırılır.

Birden fazla geçerli rüzgâr kaynağı varsa kimliği en küçük araç seçilir. Bu kural mesajların geliş sırasına bağlı olmayan aynı sonucu üretir.

Kod konumu:

- `coordination/peer_manager.py`
