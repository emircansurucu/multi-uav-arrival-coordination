# Plan Revizyonu

İlk plan nominal rota süreleriyle kurulur. Rüzgâr kestirimi geçerli olduğunda ulaşılabilir varış zamanı değişebilir. Plan revizyonu bu farkı ortak takvime taşır.

Yeni plan yalnız ileri yönde uygulanır:
- yeni plan = max(mevcut plan, yeni aday)

Bu durum küçük ETA ve rüzgâr değişimlerinin planı ileri geri oynatmasını engeller. Takip eden araçlar kendilerinden önceki araçların taahhütlerini ve 20 saniyelik sıra farkını dikkate alır.

Araç yerdeyse güncellenen plan kalkış zamanına yansıtılır. Araç havalandıktan sonra büyük düzeltme yerine hız kontrolü ve terminal rezervi kullanılır.

Kod konumu:

- `mission_manager.py`
- `coordination/arrival_schedule.py`

