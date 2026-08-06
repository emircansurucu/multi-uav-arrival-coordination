# Merkeziyetsiz Çıpa

Çıpa bütün araçların yetişebileceği ortak zaman başlangıcıdır. Her agent peer mesajlarından aynı değerleri alır ve hesabı kendi içinde yapar.

Araç (`i`) için en erken ulaşılabilir varış `E_i`, sıra farkı `Delta` ise:

A = max(E_i - (i - 1) * Delta)
T_i = A + (i - 1) * Delta

`A` ortak çıpa, `T_i` aracın planlanan varışıdır. Bu hesap en geç yetişebilen aracı temel alır ve diğer araçların ondan önce varmak zorunda kalmasını engeller.

HA-1 ilk sıradaki araçtır. Bütün agentlar aynı araç sıralaması ve taze peer kümesiyle aynı sonucu üretir.

Kod konumu:

- `coordination/arrival_schedule.py`
- `mission_manager.py`
