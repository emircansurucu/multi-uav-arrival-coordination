# Özet

Bu çalışmada üç sabit kanatlı hava aracının farklı pistlerden otonom olarak kalkıp kendilerine verilen rotaları izleyerek ortak bir hedefe belirli sırayla ulaşması ele alındı. Hedef, araçların HA-1, HA-2 ve HA-3 sırasıyla, ardışık varışlar arasında 20 saniye olacak biçimde görev yapmasıdır. Çözüm ArduPlane 4.6.3 SITL, ROS 2 Humble ve ArduPilot'un yerleşik DDS arayüzü üzerinde geliştirildi.

Sistemde merkezi bir karar verici veya yer kontrol istasyonu bulunmuyor. Her araç için bağımsız çalışan bir ROS 2 agent'ı var. Agent kendi ArduPlane örneğinden telemetri alıyor, diğer araçların yayınladığı durumları dinliyor ve kendi kalkış zamanını, hedef varış anını ve hız komutunu hesaplıyor. Araçlar birbirine komut göndermiyor; yalnızca durum ve plan bilgisi paylaşıyor.

Zamanlamada öncelik beklemeyi yerde yapmaya verildi. Rota uzunlukları ve rüzgâr etkisi nedeniyle oluşan büyük zaman farkı kalkış gecikmesiyle gideriliyor. Uçuş sırasında kalan küçük hata hava hızı komutuyla düzeltiliyor. Hedef çevresindeki 2 km alanda loiter yapılmıyor. Gerekli olduğunda bu sınırdan önce belirlenen son yasal kapıda kısa süreli GUIDED bekleme uygulanabiliyor. Terminal faza girerken hız değiştirme yetkisinin tamamen tüketilmemesi için ulaşılabilirlik sınırları ve terminal rezervi kullanılıyor.

Doğrulama sakin hava, sabit 8 m/s rüzgâr ve değişken rüzgâr senaryolarında yapıldı. Proje kayıtlarında her senaryo için iki başarılı koşu bulunuyor. En kötü ardışık zaman farkı hatası 0,17 s, hedefe en uzak kabul edilen geçiş 4,99 m ve en büyük rota sapması 127 m olarak ölçüldü. Son video koşusunda değişken rüzgâr altında varış aralıkları 20,07 s ve 19,94 s oldu; üç araç da 5 m hedef sınırına girdi ve havada bekleme oluşmadı.

Bu raporda önce vaka isterlerini ve aldığım temel kararları, ardından yazılım mimarisini, haberleşmeyi ve zamanlama algoritmasını anlatıyorum. Son bölümde geliştirme sırasında karşılaştığım sorunları, test sonuçlarını ve her vaka maddesinin projedeki karşılığını toplu olarak veriyorum. Kodda kullanılan teknik kavramları da ilk geçtikleri yerde kısa biçimde açıklamaya çalıştım.

# 1. Görev Tanımı ve Tasarım Kararları

## 1.1 Görev ve girdiler

Vaka çalışması üç farklı başlangıç konumu, üç farklı rota ve tek bir ortak hedef veriyor. Araçların manuel müdahale veya GCS komutu olmadan kalkması, verilen rota noktalarını takip etmesi ve hedefe HA-1, HA-2, HA-3 sırasıyla ulaşması gerekiyor. Ardışık varışlar arasındaki istenen süre 20 saniyedir.

| Araç | Başlangıç konumu | Rota noktası sayısı | Ortak hedef |
|---|---|---:|---|
| HA-1 | 47.530002 N, 122.302457 W | 6 | 47.535683 N, 122.228584 W |
| HA-2 | 47.451146 N, 122.317983 W | 4 | 47.535683 N, 122.228584 W |
| HA-3 | 47.492515 N, 122.215659 W | 5 | 47.535683 N, 122.228584 W |

[[IMAGE:routes.png|Şekil 1. Üç aracın başlangıç noktaları, verilen rotaları ve ortak hedefi]]

## 1.2 Kabul ölçütleri

Belgede zaman farkı için sayısal tolerans verilmediği için doğrulamada 20 s ± 1,0 s iç kabul ölçütü kullandım. Bu değer belgeye ait yeni bir şart değil, sonuçları otomatik değerlendirebilmek için belirlenen mühendislik toleransıdır.

| Ölçüt | Kabul değeri | Kaynak |
|---|---:|---|
| Varış sırası | HA-1 → HA-2 → HA-3 | Vaka maddesi 2 |
| Ardışık varış farkı | 20 s, doğrulamada ±1,0 s | Temel görev |
| Hedef kabul yarıçapı | 5 m | Vaka maddesi 2 |
| Rota noktası yarıçapı | 120 m, şart ≤400 m | Vaka maddesi 4 |
| En büyük rota sapması | 500 m | Vaka maddesi 4 |
| Terminal loiter | Hedefe 2 km içinde yasak | Vaka maddesi 6 |
| Seyir irtifası | 400 m MSL | Vaka maddesi 3 |
| Görev sonu | RTL komutunun telemetriden doğrulanması | Vaka maddesi 2 |

Varış anı, aracın hedef merkezli 5 m kabul çemberine ilk giriş anıdır. İki telemetri örneği arasında çember geçilmişse giriş anı doğru parçası ile çemberin kesişiminden interpolasyonla hesaplanır. En yakın geçiş mesafesi de ayrıca kaydedilir. Böylece örnekleme anına bağlı bir veya iki metrelik ölçüm farkının zaman ölçümünü bozması önlenir.

400 m MSL şartını seyir ve terminal yaklaşma fazları için uyguladım. Kalkış sırasında aracın 400 m'ye tırmanması ve RTL sırasında dönüş davranışı bu sabit irtifa yorumunun dışındadır. Aksi hâlde sabit kanatlı bir aracın yerde başlayıp görevi tamamlaması fiziksel olarak mümkün değildir.

## 1.3 Kullanılan ortam

| Bileşen | Sürüm veya seçim |
|---|---|
| İşletim sistemi | Ubuntu 22.04.5 LTS |
| Python | 3.10.12 |
| ROS 2 | Humble |
| Otopilot | ArduPlane 4.6.3, `Plane-4.6.3` |
| Simülasyon | Üç ayrı ArduPlane SITL |
| DDS köprüsü | Micro XRCE-DDS Agent 2.4.2 |
| MAVLink istemcisi | pymavlink 2.4.49 |
| Jeodezi | GeographicLib 1.52 |

ArduPilot kaynak kodunda değişiklik yapılmadı. ArduPlane'in kendi uçuş denetleyicileri ve uçuş dinamiği kullanıldı. Bu projedeki Python kodu rota ve zamanlama seviyesinde karar veriyor; servo, yüzey veya düşük seviye uçuş denetimi üretmiyor.

SITL, gerçek uçuş kontrol yazılımının fiziksel uçak yerine bilgisayar üzerinde çalışan bir uçuş modeliyle birlikte çalıştırılmasıdır. Bu projede üç ayrı SITL süreci üç ayrı uçağı temsil ediyor. ROS 2 node, belirli bir işi yapan ve ROS ağına veri alıp gönderen süreç parçasıdır. Her araç için aynı Python paketi ayrı yapılandırmayla çalıştırıldığı için üç araç birbirinden bağımsız karar döngülerine sahiptir.

DDS, ROS 2'nin dağıtık veri paylaşım altyapısıdır. ArduPilot'un AP_DDS katmanı telemetriyi XRCE-DDS istemcisi üzerinden dışarı verir. Micro XRCE-DDS Agent ise ArduPilot içindeki hafif istemciyle normal DDS ağı arasında köprü kurar. MAVLink bundan farklı olarak görev yükleme, arm, mod ve hız komutlarında kullandığım komut protokolüdür. Telemetriyi DDS üzerinden, AP_DDS sürümünde bulunmayan görev komutlarını ise MAVLink üzerinden taşımamın nedeni budur.

## 1.4 Vaka isterlerine göre seçilen çözüm

Vaka belgesinde tek bir algoritma zorunlu tutulmuyor. Kalkışı geciktirme, hız yönetimi, loiter ve yol uzatma seçenek olarak verilmiş. Ben büyük zaman farkını yerde kalkış gecikmesiyle, uçuşta kalan küçük farkı hava hızıyla ve yalnız gerekli olduğunda hedef dışındaki güvenli kapıda loiter ile çözmeyi seçtim. S-manevrasını denedim ancak nihai sisteme almadım.

| Vaka maddesi | Projedeki uygulama |
|---|---|
| Otomatik kalkış | Mission Python tarafından yükleniyor, AUTO ve arm komutları otomatik veriliyor |
| Sıralı varış ve RTL | 5 m hedef çemberi, HA-1 / HA-2 / HA-3 sırası ve varıştan sonra doğrulanan RTL |
| 400 m MSL | Rota waypoint'leri ve terminal yaklaşma 400 m MSL, kalkış tırmanışı ve RTL ayrı tutuluyor |
| Waypoint ve rota sınırı | Ara waypoint yarıçapı 120 m, hedef 5 m, rota sapması sınırı 500 m |
| 20 saniye zamanlama | Merkeziyetsiz çıpa, kalkış gecikmesi, hız kontrolü ve terminal rezervi |
| 2 km loiter yasağı | Bekleme merkezi 2500 m'de, loiter yarıçapı 80 m, 2200 m'de zorunlu çıkış |
| Değişken rüzgâr | 4–10 m/s değişken profil, vektörel rüzgâr kestirimi ve ulaşılabilirlik zarfı |
| Havada beklemeyi azaltma | Önce yerde bekleme, yalnız kontrol yetkisi yetmezse yasal kapıda tek bekleme |

Tam 20 saniye ifadesi için vaka belgesi tolerans vermiyor. Fiziksel simülasyon, telemetri örnekleme aralığı ve sabit kanatlı aracın dönüş davranışı nedeniyle doğrulamada ±1 saniyelik iç sınır kullandım. Ölçülen en kötü hata 0,17 saniyedir. Bu sonucu tam matematiksel eşitlik olarak değil, belirlenen mühendislik toleransı içinde zamanlama başarısı olarak değerlendiriyorum.

Benzer biçimde robust kelimesini sınırsız rüzgâr altında garanti anlamında kullanmıyorum. Buradaki sağlamlık iddiası normal test profilinin 4–10 m/s hız ve 0,50 derece/s tepe yön değişimiyle sınırlıdır. Daha hızlı extreme profil sistemin sınırını görmek için tutuldu, kabul koşusu olarak kullanılmadı.

# 2. Sistem Mimarisi ve Merkeziyetsiz Yaklaşım

## 2.1 Genel mimari

Her araç için bir ArduPlane SITL, bir XRCE-DDS Agent ve bir `oasy_uav_agent` süreci çalışıyor. Araç agent'ı iki farklı ROS 2 ağına aynı anda bağlıdır. Araca özel ağ yalnız o aracın AP_DDS telemetrisi ve GUIDED komutları için, ortak ağ ise araçlar arası koordinasyon için kullanılır.

Burada agent kelimesi, bir hava aracı adına telemetriyi okuyup karar veren Python sürecini ifade ediyor. Domain ise aynı DDS ağı içinde birbirini görebilen katılımcıların sayısal alanıdır. Farklı domain'lerdeki node'lar aynı konu adını kullansalar bile birbirlerinin mesajını almaz. Bu özellik, üç ArduPlane'in aynı `/ap/...` konu adlarını yayınladığı durumda araç verilerini ayırmak için kullanıldı.

[[IMAGE:architecture.png|Şekil 2. Üç araçlı sistemin süreçleri ve iki katmanlı DDS yapısı]]

| Araç | AP_DDS domain | XRCE UDP port | MAVLink adresi | Koordinasyon domain |
|---|---:|---:|---|---:|
| HA-1 | 1 | 2019 | tcp:127.0.0.1:5760 | 10 |
| HA-2 | 2 | 2020 | tcp:127.0.0.1:5770 | 10 |
| HA-3 | 3 | 2021 | tcp:127.0.0.1:5780 | 10 |

Tek süreç içinde iki `rclpy.Context` açılmasının nedeni domain'leri yalnız ortam değişkeniyle ayırmanın yeterli olmamasıdır. Araç tarafındaki context domain 1, 2 veya 3'te; koordinasyon tarafındaki context domain 10'da çalışır. İki context ayrı executor ile döndürülür. Böylece AP_DDS konuları ortak koordinasyon ağına taşınmaz ve bir agent yanlışlıkla başka aracın telemetrisini okuyamaz.

İkinci koruma olarak ilk geçerli konum aracın YAML dosyasındaki başlangıç konumuyla karşılaştırılır. Mesafe 1000 m'den büyükse domain veya araç eşleşmesi hatalı kabul edilip loglanır. Bu kontrol, DDS bağlantısının teknik olarak kurulup yanlış araca bağlandığı sessiz arızayı görünür hâle getirir.

## 2.2 Araç agent'ının iç yapısı

Bir `oasy_uav_agent` süreci dışarıdan tek ROS 2 node'u gibi başlatılsa da içinde birbirinden ayrılmış üç çalışma parçası bulunur. `VehicleSide`, araca özel DDS domain'inde telemetri aboneliklerini ve `/ap/cmd_gps_pose` yayıncısını oluşturur. `CoordinationSide`, domain 10 üzerinde `VehicleStatus` yayınlar ve diğer araçların aynı mesajını dinler. `MissionManager` ise bu iki tarafın verilerini kullanarak ayrı bir thread içinde 20 Hz görev döngüsünü yürütür.

| Çalışma parçası | Kendi verisi | Dışarıyla bağlantısı |
|---|---|---|
| `VehicleSide` | Son telemetri örneği | Yalnız ilgili ArduPlane AP_DDS domain'i |
| `CoordinationSide` | Peer kayıtları ve yayın sayacı | Yalnız ortak domain 10 |
| `MissionManager` | Durum makinesi, plan, ETA, rüzgâr, hız komutu | Telemetriyi okur; MAVLink ve GUIDED komutu üretir |
| Durum zamanlayıcısı | Görev ve telemetri snapshot'ı | 5 Hz `VehicleStatus` yayını ve ilerleme logu |

İki ROS context ayrı `SingleThreadedExecutor` ile çalıştırılır. Bu executor'ların görevi yalnız ROS mesajlarını almak ve yayınlamaktır. Zamanlama algoritması executor callback'i içinde uzun hesap yapmaz; `MissionManager` thread'i kendi periyodunda güncel snapshot'ları alır. Bu ayrım, telemetri callback'lerinin rüzgâr zarfı hesabı gibi daha pahalı işlemler nedeniyle gecikmesini önler.

MAVLink bağlantısı DDS context'lerinden bağımsızdır. `MavlinkCommander` kendi TCP bağlantısından heartbeat, mission isteği ve komut onayı bekler. Bu nedenle DDS telemetrisi geçici olarak gecikse bile MAVLink protokolünün mission upload veya mod onayı mesajları başka bir ROS kuyruğunda sıkışmaz. Buna karşılık görev yöneticisi 3 saniyeden eski telemetriyi geçerli kabul etmez.

## 2.3 Merkeziyetsizlik

Sistemde bütün araçlara hedef zaman dağıtan bir master node yoktur. Her agent aşağıdaki bilgileri kendi yerel durumunda tutar:

- Kendi telemetrisi ve görev durumu
- Kendi rotası ve kalan mesafesi
- Kendi rüzgâr kestirimi
- Diğer araçların yayınladığı taahhüt edilmiş varış zamanları
- Diğer araçların en erken ulaşılabilir varış sınırları

Her araç aynı deterministik kuralları kullandığı için ortak zamanlama çıpasını bağımsız hesaplar. Görselleştirici ve analiz node'u yalnız dinleyicidir. Bu iki araç kapatılsa bile uçuş ve koordinasyon devam eder. Aynı şekilde GCS, MAVProxy veya başka bir harita uygulaması kontrol döngüsünün parçası değildir.

HA-1'in önce kalkması onu master node yapmaz. HA-1 yalnız kendi planını ve ölçümünü diğer araçlarla aynı mesaj biçiminde yayınlar. HA-2 ve HA-3 bu bilgiyi kullanabilir, ancak HA-1 onlara kalkış veya hız komutu göndermez. Her takipçi kendi rota süresini ve kendi kalkış zamanını hesaplar. HA-1 kaybolursa merkezi bir süreç çökmüş olmaz; kalan araçlar taze peer kümesiyle hesap yapmayı sürdürür. Bununla birlikte kaybolan aracın sıra kısıtı artık görülemediği için görev bütünlüğü garanti edilemez. Bu, merkeziyetsizlik ile hata toleransının aynı kavram olmadığını gösterir.

Üç agent'ın aynı sonuca varabilmesi iki koşula bağlıdır. Birincisi, kullanılan fonksiyonların araç kimliğine göre deterministik olmasıdır. İkincisi, agent'ların aynı taze peer kümesini görmesidir. Ağ bölünmesi iki gruba farklı peer kümeleri gösterirse farklı çıpalar oluşabilir. Mevcut projede split-brain uzlaşması bulunmadığı için bu durum bilinen sınırlama olarak tutuldu.

Merkeziyetsizliğin bir sınırı vardır: bütün süreçler aynı fiziksel bilgisayarda çalıştığı için `time.monotonic_ns()` değerleri karşılaştırılabilir. Agent'lar ayrı bilgisayarlara taşınırsa ortak zaman tabanı için PTP veya yeterli doğrulukta NTP gerekir.

## 2.4 Veri sahipliği ve karar sınırları

Sistemde aynı bilginin birden fazla yerde farklı anlamla tutulmaması için veri sahipliği belirgindir. Ham konum ve hızın sahibi ArduPlane'dir. Agent bu veriyi değiştirmez, yalnız son geçerli örnek olarak saklar. Rota ve yapılandırma YAML dosyasından gelir. Taahhüt edilmiş varış zamanı ilgili agent'ın kendi kararıdır; peer'lar bunu yalnız girdi olarak kullanır. Gerçek varış anı ise `ArrivalDetector` tarafından bir kez üretilir ve sonradan değiştirilmez.

| Bilgi | Üreten | Tüketen | Değiştirme yetkisi |
|---|---|---|---|
| Konum, hız, yönelim | ArduPlane EKF / AP_DDS | Kestirim ve görselleştirme | ArduPlane |
| Mission rotası | YAML + `mission_builder.py` | ArduPlane AUTO ve ETA modeli | Görev başlamadan önce agent |
| Rüzgâr kestirimi | `wind_estimator.py` | ETA, E/L ve peer yayını | İlgili araç agent'ı |
| Planlanan varış | `mission_manager.py` | Hız kontrolü ve diğer araçlar | İlgili araç agent'ı |
| Gerçek varış | `arrival_detector.py` | Analiz, RTL ve peer mesajı | İlk 5 m çember girişiyle mandallanır |
| Uçuş yüzeyi ve gaz | ArduPlane denetleyicileri | Fiziksel/SITL araç | Python katmanı doğrudan erişmez |

Bu sınır özellikle uçuş dinamiği açısından önemlidir. Python kodu roll, pitch, servo veya throttle çıktısı hesaplamaz. Agent yalnız AUTO mission, uçuş modu ve hedef hava hızı gibi üst seviye istekler üretir. TECS, L1 ve ArduPlane'in diğer denetleyicileri bu istekleri uçuş hareketine dönüştürür.

## 2.5 Yazılım katmanları

| Katman | Dosyalar | Sorumluluk |
|---|---|---|
| Başlatma | `run.sh`, `start_all.sh`, `start_sitl.sh` | Ortam kontrolü, üç SITL ve agent'ların başlatılması |
| Ana süreç | `agent_node.py`, `mission_manager.py` | İki DDS context, görev durum makinesi ve kontrol döngüsü |
| ArduPilot adaptörü | `dds_telemetry.py`, `dds_commands.py`, `mavlink_link.py`, `mission_builder.py` | Telemetri, görev yükleme, mod, arm ve hız komutları |
| Koordinasyon | `arrival_schedule.py`, `peer_manager.py`, `status_publisher.py` | Çıpa hesabı, peer tazeliği ve durum yayını |
| Kestirim | `geodesy.py`, `eta_estimator.py`, `wind_estimator.py`, `arrival_detector.py` | Mesafe, rota, ETA, rüzgâr ve varış ölçümü |
| Kontrol | `arrival_controller.py` | Zaman hatasından sınırlı hava hızı komutu üretimi |
| Doğrulama | `analyze_run.py`, `visualize.py` | Kabul analizi ve uçuşun görsel takibi |

`mission_manager.py` çekirdek dosyadır. Bağlantı, görev yükleme, peer bekleme, kalkış planı, seyir, terminal, varış ve RTL geçişlerini yönetir. Diğer modüller hesaplama veya dış sistem bağlantısı gibi daha dar sorumluluklara ayrılmıştır.

Kaynak kodu okurken izlenebilecek en kısa sıra aşağıdaki gibidir:

1. `run.sh` ortamı kontrol eder ve seçilen senaryoyu başlatır.
2. `start_all.sh` üç SITL ve üç XRCE agent sürecini açar.
3. `three_vehicles.launch.py` aynı agent paketini üç YAML dosyasıyla başlatır.
4. `agent_node.py` araç ve koordinasyon context'lerini kurar.
5. `mission_manager.py` görev durumlarını 20 Hz hızla ilerletir.
6. `dds_telemetry.py` son geçerli telemetri snapshot'ını verir.
7. Kestirim modülleri rota, ETA, rüzgâr ve varış değerlerini hesaplar.
8. Koordinasyon modülleri peer mesajlarını süzer ve ortak planı kurar.
9. `arrival_controller.py` gerekiyorsa yeni hava hızı üretir.
10. `mavlink_link.py` görev, mod, arm, hız ve RTL komutlarını ArduPlane'e gönderir.

Snapshot terimi, farklı callback'lerde güncellenen verilerin görev döngüsüne tek bir tutarlı kopya olarak verilmesi anlamına geliyor. MissionManager hesap sırasında canlı nesneleri tek tek okumak yerine bu kopyayı kullanıyor. Böylece bir hesabın ortasında konum yeni, hız eski örnekle değişmiş olmuyor.

## 2.6 Kaynak dosyaların görevleri

| Dosya | Görevi |
|---|---|
| `agent_node.py` | İki ROS context, executorlar, durum yayını ve başlangıç konumu doğrulaması |
| `mission_manager.py` | Görev durum makinesi, plan, kalkış, seyir, terminal, varış ve RTL |
| `config_model.py` | YAML ayarlarını okuyup araç, hız ve yarıçap sınırlarını doğrulama |
| `dds_telemetry.py` | GeoPose, Twist ve Airspeed mesajlarından telemetri snapshot'ı oluşturma |
| `dds_commands.py` | GUIDED konum hedefini `/ap/cmd_gps_pose` üzerinden yayınlama |
| `mavlink_link.py` | Mission protokolü, arm, mod, hız ve RTL komutları |
| `mission_builder.py` | TAKEOFF ve GLOBAL MSL waypoint listesini oluşturma |
| `arrival_schedule.py` | Ortak çıpa, kalkış zamanı ve kapı bırakma penceresi |
| `peer_manager.py` | Diğer araç mesajlarının sıra ve tazelik kontrolü |
| `status_publisher.py` | `VehicleStatus` mesajını 5 Hz yayınlama |
| `eta_estimator.py` | Aktif waypoint, kalan mesafe ve ilerleme hızı |
| `wind_estimator.py` | Rüzgâr vektörü ve rüzgâr altında rota süresi |
| `geodesy.py` | WGS84 mesafe, yerel düzlem, sapma ve çember kesişimi |
| `arrival_detector.py` | 5 m hedef çemberine giriş anını bulma |
| `arrival_controller.py` | Zaman hatasından sınırlı hava hızı komutu üretme |
| `analyze_run.py` | Varış, sapma ve bekleme ölçütlerini otomatik değerlendirme |

Bu ayrımın amacı her dosyayı küçük göstermek değil, uçuş sırasında bir sorun çıktığında hangi hesabın sorumlu olduğunu bulabilmektir. Örneğin yanlış rüzgâr değeri `wind_estimator.py`, doğru rüzgâra rağmen yanlış kalkış anı `arrival_schedule.py`, doğru planın uçağa uygulanamaması ise `mission_manager.py` veya MAVLink katmanında aranır.

## 2.7 Başlatma akışı

Sistem `./run.sh calm`, `./run.sh steady` veya `./run.sh variable` komutuyla başlatılır. Betik önce ROS 2, Python bağımlılıkları, ArduPlane binary'si ve Micro XRCE-DDS Agent kurulumunu kontrol eder. Ardından önceki koşudan kalan süreçleri kapatır ve seçilen rüzgâr senaryosunu hazırlar.

`start_all.sh` üç adet `start_sitl.sh` süreci açar. Her `start_sitl.sh` önce ilgili UDP portunda XRCE agent'ı, sonra doğru instance ve başlangıç konumuyla ArduPlane SITL'i başlatır. SITL logunda DDS veri yazıcısının kurulduğu görüldükten sonra ROS 2 launch dosyası üç bağımsız agent node'unu açar.

[[IMAGE:startup.png|Şekil 3. Tek komuttan üç araçlı göreve kadar başlatma sırası]]

Başlatma sırasındaki DDS hazır kontrolü yalnız sürecin varlığını değil, SITL'in XRCE agent üzerinden DDS entity'lerini oluşturduğunu arar. Üç araç da hazır olmadan ROS agent'ları başlatılmaz. Bu sayede bir aracın geç açılması nedeniyle diğerlerinin eksik peer kümesiyle erken plan taahhüt etmesi azaltılır.

## 2.8 Mimari tercihin sonucu

Bu mimarinin temel avantajı ArduPilot tarafına değişiklik gerektirmemesidir. Her SITL standart ArduPlane 4.6.3 binary'sini çalıştırır. Ayrıştırma `DDS_DOMAIN_ID`, `DDS_UDP_PORT`, ayrı XRCE agent süreçleri ve agent içindeki iki context ile sağlanır. Dolayısıyla koordinasyon algoritması ArduPilot fork'una gömülmeden Python paketinde geliştirilebilir ve birim testle sınanabilir.

Bedeli ise süreç sayısı ve zaman yönetimidir. Üç araç için üç SITL, üç XRCE agent ve üç Python agent bulunur. Ayrıca ortak `monotonic_ns` değerlerinin anlamlı olması aynı makine varsayımına dayanır. Gerçek dağıtık donanımda ağ gecikmesi, saat ofseti ve kayıp paketler ayrıca ölçülmelidir.

# 3. Haberleşme Akışı ve Paket Yapısı

## 3.1 ArduPilot'tan alınan veriler

Telemetri AP_DDS üzerinden doğrudan ROS 2 ortamına gelir. MAVROS kullanılmaz.

AP_DDS ile MAVLink bu projede birbirinin alternatifi değildir. AP_DDS sürekli akan konum, hız, yönelim ve hava hızı verileri için kullanılır. MAVLink ise cevap veya durum doğrulaması gereken görev komutları için kullanılır. Bu ayrım vaka belgesindeki doğrudan DDS tercihini korurken ArduPlane 4.6.3 AP_DDS arayüzünde bulunmayan mission upload ve hava hızı komutlarını da kullanabilmemi sağladı.

| ROS 2 konusu | Mesaj | Kullanılan bilgi |
|---|---|---|
| `/ap/geopose/filtered` | `geographic_msgs/GeoPoseStamped` | Enlem, boylam, MSL irtifası ve yönelim |
| `/ap/twist/filtered` | `geometry_msgs/TwistStamped` | Doğu, kuzey ve düşey yer hızı |
| `/ap/airspeed` | `geometry_msgs/Vector3Stamped` | Gövde FLU eksenindeki hava hızı vektörü |

`dds_telemetry.py` bu üç akışı son geçerli örnekte birleştirir. Sıfıra yakın koordinatlar EKF kurulmadan gelen geçersiz veri olarak elenir. Telemetri yaşı ve rüzgâr hesabında kullanılan örneklerin zaman farkı ayrıca izlenir.

Üç konu aynı anda gelmediği için snapshot içindeki alanların güncellenme anları birebir eşit değildir. Rüzgâr hesabı yer hızı, hava hızı ve yönelimi birlikte kullandığından örnekler arasındaki yayılım 0,2 saniyeyi aşarsa örnek reddedilir. Hava hızı 10 m/s'nin altındaysa veya araç kalkış irtifasına ulaşmadıysa pervane etkisi ve yerdeki geçersiz yönelim nedeniyle rüzgâr filtresi ilerletilmez.

Konumun `GeoPoseStamped` üzerinden alınmasının iki yararı vardır. Birincisi, hedef ve waypoint'lerle aynı küresel koordinat sisteminde çalışılmasıdır. İkincisi, mesajın orientation alanının gövde hava hızını ENU eksenine döndürmek için kullanılabilmesidir. `TwistStamped` doğrusal hızının x ve y alanları doğu ve kuzey bileşenleri olarak değerlendirilir. Bütün hızlar m/s, irtifa metre MSL ve açılar derece biriminde tutulur.

MSL, irtifanın ortalama deniz seviyesine göre verilmesidir. AGL ise yerden yükseklik anlamına gelir ve bu görevde waypoint irtifası olarak kullanılmadı. Vaka belgesi 400 m deniz seviyesi irtifası istediği için mission öğeleri relatif irtifa yerine `MAV_FRAME_GLOBAL` ile yazıldı.

## 3.2 ArduPilot'a gönderilenler

Kontrol yolu iki arayüze ayrılmıştır.

| Arayüz | Gönderilen işlem | Neden |
|---|---|---|
| MAVLink | Mission upload | TAKEOFF ve rota waypoint'lerini yüklemek |
| MAVLink | AUTO, GUIDED, RTL modu | Görev fazını değiştirmek |
| MAVLink | Arm | Kalkışı Python sürecinden otomatik başlatmak |
| MAVLink | `MAV_CMD_DO_CHANGE_SPEED` | Hedef hava hızını değiştirmek |
| AP_DDS | `/ap/cmd_gps_pose` | Son yasal kapının GUIDED merkezini göndermek |

Ana uçuş AUTO mission olarak yürütülür. GUIDED yalnız 2 km sınırının dışındaki son yasal kapıda kısa bekleme gerektiğinde kullanılır. Bekleme bittiğinde AUTO moduna dönülür ve ArduPlane kaldığı mission öğesinden devam eder. Hedef tespit edildiğinde RTL modu istenir ve mod telemetrisinden doğrulanır.

AUTO, uçağın önceden yüklenen mission listesini takip ettiği ArduPlane modudur. GUIDED, dışarıdan verilen geçici konum hedefini izler; sabit kanatlı uçak bu hedefte duramadığı için hedef çevresinde loiter çemberi oluşturur. RTL ise uçağın kalkış noktasına dönüş modudur. Projede varış ölçümü RTL başlamadan hemen önce mandallanır. Bu nedenle dönüşün uzun sürmesi araçlar arası 20 saniye hesabını değiştirmez.

Mission upload standart MAVLink mission protokolüyle yapılır. Agent önce mevcut listeyi temizler, gönderilecek öğe sayısını bildirir ve ArduPlane'in istediği sıra numarasındaki `MISSION_ITEM_INT` öğesini gönderir. İşlem yalnız `MISSION_ACK` sonucu kabul edildiğinde başarılı sayılır. Görev listesi sırasıyla home kaydı, `MAV_CMD_NAV_TAKEOFF` ve verilen `MAV_CMD_NAV_WAYPOINT` öğelerinden oluşur. Waypoint'ler `MAV_FRAME_GLOBAL` ile 400 m MSL olarak yazılır; home irtifasından relatif yükseklik hesabı yapılmaz.

Arm ve mod değişiklikleri de yalnız paketin gönderilmiş olmasına göre başarılı sayılmaz. Arm için `COMMAND_ACK` sonucu ve motorların armed durumu, mod için heartbeat içindeki `custom_mode` değeri beklenir. Hız komutu `MAV_CMD_DO_CHANGE_SPEED` içinde hız tipi airspeed olacak şekilde gönderilir. Bu ayrım önemlidir: rüzgâr değişirken doğrudan yer hızı istemek yerine ArduPlane'e uçuş zarfı içindeki hava hızı hedefi verilir.

GUIDED konum hedefi sabit kanatlı ArduPlane'de ulaşılacak ve durulacak bir nokta değildir; uçak hedef çevresinde dönmeye başlar. Bu davranış S-manevrası denemelerinde sorun oluşturmuş, fakat kapı loiteri için istenen davranış olduğu için yalnız bu amaçla tutulmuştur.

## 3.3 Araçlar arası durum mesajı

Araçlar domain 10 üzerinde `/oasy/vehicle_status` konusunu 5 Hz hızla yayınlar. Aktif koordinasyon sözleşmesi `VehicleStatus.msg` mesajıdır.

Bu mesaj bir komut paketi değildir. Bir araç diğerine hızlan, bekle veya kalk demez. Yalnız kendi durumunu yayınlar. Mesajı alan araç bu bilgiyi kendi rotası ve kendi telemetrisiyle birleştirip karar verir. Merkeziyetsiz yapının yazılım seviyesindeki karşılığı budur.

| Alan grubu | Önemli alanlar | Kullanım |
|---|---|---|
| Kimlik | `vehicle_id`, `seq`, `mission_state` | Kaynak araç ve mesaj sırası |
| Zaman | `monotonic_ns` | Peer mesajının yaşını hesaplamak |
| Telemetri | `latitude`, `longitude`, `altitude_msl`, `groundspeed` | Görselleştirme ve durum takibi |
| Kestirim | `remaining_distance`, `eta_seconds` | Aracın kalan görev durumu |
| Plan | `planned_arrival_monotonic_ns`, `arrival_committed` | Merkeziyetsiz varış sırası |
| Ulaşılabilirlik | `earliest_feasible_arrival_monotonic_ns` | Ortak çıpa hesabı |
| Rüzgâr | `wind_valid`, `wind_speed`, `wind_dir_deg` | Geçerli rüzgâr bilgisini paylaşmak |
| Sonuç | `target_reached`, `actual_arrival_monotonic_ns` | Varışların kesin ölçümü |

Kaynakta ayrıca `MissionEvent.msg` tanımı bulunuyor. Bu mesaj kritik görev olaylarını güvenilir bir kanala ayırmak için hazırlanmış, ancak mevcut uçuş akışında publisher veya subscriber tarafından kullanılmıyor. Bu nedenle sonuçlarda kullanılan aktif mesaj olarak gösterilmedi.

`VehicleStatus` iki farklı zaman bilgisini bilinçli olarak taşır. `header.stamp` ROS araçları, rosbag ve video eşleştirmesi için wall-clock tabanlı kayıttır. `monotonic_ns` ise sistem saati ileri veya geri alınsa bile süre hesabının bozulmaması için koordinasyonda kullanılır. Planlanan ve gerçek varış alanları da monotonic zaman eksenindedir. Bu alanların başka bir bilgisayarda doğrudan karşılaştırılabilmesi için saat tabanlarının senkronize edilmesi gerekir.

Monotonic saat takvim saati değildir. Bilgisayar saati kullanıcı veya NTP tarafından değiştirildiğinde geriye gitmez ve yalnız geçen süreyi ölçmek için kullanılır. Üç agent aynı makinede olduğu için bu değerleri doğrudan karşılaştırabildim. Gerçek araçlarda her bilgisayarın monotonic başlangıcı farklı olacağından ortak PTP veya NTP zamanı gerekir.

`planned_arrival_monotonic_ns` ile `earliest_feasible_arrival_monotonic_ns` aynı şey değildir. Birincisi aracın taahhüt ettiği hedef zamandır. İkincisi, mevcut konum veya planlanan kalkış anından azami hava hızıyla erişilebilen en erken zamandır. Ortak çıpa ikinci alanlardan kurulur; sıra ilişkisi ve hız kontrolü birinci alana göre yürür. Bu ayrım, henüz erişemeyeceği bir zamanı taahhüt eden aracın bütün zinciri bozmasını önler.

## 3.4 QoS ve tazelik

Durum mesajları `BEST_EFFORT`, derinlik 10 profiliyle yayınlanır. Konum, ETA ve hız gibi alanlar hızlı eskiyen verilerdir; gecikmiş bir örneği yeniden göndermek yeni örnekten daha faydalı değildir. Bunun karşılığında uygulama mesaj kaybını ve yaşını kendisi izler.

QoS, ROS 2 mesajının kuyruk, güvenilirlik ve saklama davranışını belirleyen ayarlardır. Best effort profilinde kaybolan bir durum paketi yeniden istenmez; birkaç yüz milisaniye sonra zaten daha güncel paket gelir. Mission upload ve mod değişimi için aynı yaklaşım kullanılmadı, çünkü bu komutların kaybolması görev durumunu değiştirir. Bu işlemler MAVLink ACK veya heartbeat doğrulaması bekler.

Peer verisi 2 saniyeden gençse taze, 2–5 saniye arasındaysa eski, 5 saniyeyi aşmışsa kayıp kabul edilir. Kayıp veya görevini tamamlamış araç rüzgâr kaynağı olarak seçilmez. Birden fazla geçerli kaynak varsa araç kimliği en küçük olan seçilerek geliş sırasından bağımsız deterministik davranış sağlanır.

Eski mesaj tamamen silinmez; durum ekranı ve tanılama için saklanır. Fakat 5 saniye eşiğini geçen peer taahhüt ve ulaşılabilirlik hesabına girmez. Aynı araçtan sıra dışı gelen, daha küçük monotonic zaman damgalı mesaj da son durumu geriye götürmemesi için reddedilir. `seq` alanı kayıp mesaj sayısını izlemek, alım tarafındaki monotonic damga ise mesaj yaşını ölçmek için kullanılır.

`BEST_EFFORT` seçimi komut kanalı için kullanılmaz. Mission upload, arm ve mod değişimi MAVLink'in onaylı protokollerinden geçer. Yani hızlı eskiyen durum yayını kayba toleranslıdır; uçuş durumunu değiştiren işlemler onay bekler. Haberleşme güvenilirliği ihtiyaca göre iki farklı kanala ayrılmıştır.

## 3.5 Tek kontrol çevrimi

[[IMAGE:control_loop.png|Şekil 4. Telemetriden hız komutuna kapalı çevrim veri akışı]]

Bir çevrimde önce ArduPilot konum, yer hızı, hava hızı ve yönelim yayınlar. Agent bu verilerden rüzgârı, aktif rota bacağını, kalan mesafeyi ve ETA'yı hesaplar. Diğer araçların taahhüt edilmiş planları okunur ve kendi hedef varış anı belirlenir. Plan ile beklenen varış arasındaki hata hız kontrolcüsüne girer. Üretilen sınırlı hava hızı komutu MAVLink ile ArduPilot'a gönderilir. ArduPilot'un tepkisi sonraki DDS telemetrisinde yeniden ölçülür.

## 3.6 Görev boyunca mesaj sırası

Bağlantı aşamasında her agent kendi AP_DDS telemetrisinin geçerli olmasını ve MAVLink heartbeat alınmasını bekler. Ardından görev yüklenir ve `WAIT_PEERS` durumunda diğer iki aracın yayınları görülür. HA-1 önünde araç olmadığı için referans planını kurup ilk uygun anda arm olur. HA-2 ve HA-3 önceki araçların taahhütlerini aldıktan sonra kendi kalkış anlarını hesaplar.

Seyirde veri akışı sürekli fakat komut üretimi seyrektir. Telemetri onlarca hertz hızla gelebilir, görev döngüsü 20 Hz ve durum yayını 5 Hz çalışır. Hız kontrolcüsü her görev çevriminde hesap yapabilse de yeni değer önceki gönderilen komuttan 0,1 m/s farklı değilse MAVLink paketi göndermez. Böylece hesap sıklığı ile dış komut sıklığı birbirinden ayrılır.

Varışta `ArrivalDetector` 5 m çember girişini mandallar. Aynı kontrol çevriminde durum `ARRIVED` olur. Bir sonraki durum işlemi RTL modunu ister; `MavlinkCommander` heartbeat'te RTL görülünce durum `RTL`, ardından `DONE` olur. Analiz node'u gerçek varış zamanını `VehicleStatus.actual_arrival_monotonic_ns` alanından alır. Bu nedenle RTL dönüşünün uzun sürmesi 20 saniyelik varış ölçümünü değiştirmez.

## 3.7 Haberleşmenin görev açısından özeti

| Akış | Hız / tetikleme | Kayıp davranışı | Görevdeki etkisi |
|---|---|---|---|
| AP_DDS telemetri | ArduPlane yayın hızı | 3 s sonra geçersiz | ETA, rüzgâr ve varış hesabı durur |
| `VehicleStatus` | 5 Hz | 2 s eski, 5 s kayıp | Peer planı ve rüzgâr kaynağı daralır |
| Mission upload | Görev başında bir kez | İstek/yeniden gönderim ve ACK | Kabul edilmezse AUTO göreve geçilmez |
| Arm / mod | Durum geçişinde | `COMMAND_ACK` veya heartbeat beklenir | Başarısız geçiş loglanır, durum ilerlemez |
| Hız komutu | En az 0,1 m/s değişimde | Sonraki çevrimde yeni komut gelebilir | Küçük zaman hatası kademeli düzeltilir |
| GUIDED konum | Kapı loiteri sırasında tekrarlı | Sonraki gönderim hedefi yeniler | Yasal kapı merkezi korunur |

## 3.8 ArduPilot ile görev yazılımının görev sınırı

ArduPilot'tan aldığım temel girdiler konum, MSL irtifa, yer hızı, hava hızı ve yönelimdir. Python tarafında bunlardan kalan mesafe, ETA, rüzgâr, rota sapması ve varış zamanı üretilir. Diğer araçlardan alınan plan ve ulaşılabilirlik bilgisi de bu yerel hesaba eklenir.

ArduPilot'a verdiğim çıktılar mission listesi, AUTO veya GUIDED veya RTL modu, arm komutu, hedef hava hızı ve gerektiğinde geçici GUIDED konumudur. Roll, pitch, throttle veya servo komutu göndermiyorum. ArduPlane'in L1 rota takipçisi, TECS enerji kontrolü ve kendi uçuş denetleyicileri üst seviye istekleri uçağın hareketine çeviriyor.

Bu sınır vaka açısından önemlidir. Yapılan çalışma yeni bir sabit kanat uçuş kontrolcüsü yazmak değil, hazır otopilotun üstünde merkeziyetsiz görev ve zamanlama katmanı kurmaktır. Simülasyon hareketlerini ArduPlane üretir; Python kodu ne zaman kalkılacağını, hangi mission'ın izleneceğini ve zaman hatasına göre hangi hava hızının isteneceğini belirler.

# 4. Varış Zamanı Kontrolü ve Matematiksel Model

## 4.1 Görev durum makinesi

Her agent aynı 14 durumlu tek yönlü görev akışını yürütür:

Durum makinesi, görevi tek bir büyük döngü içinde çok sayıda koşulla yönetmek yerine bağlantı, görev yükleme, kalkış ve seyir gibi açık aşamalara ayırır. Her durumda yalnız o aşamanın işlemi yapılır. Geçiş koşulu sağlandığında sonraki duruma gidilir ve önceki aşama tekrar çalıştırılmaz.

`INIT → CONNECTING → MISSION_UPLOAD → WAIT_PEERS → WAIT_TAKEOFF_SLOT → ARMING → TAKEOFF → CLIMB → CRUISE → TERMINAL → ARRIVED → RTL → DONE`

Görev döngüsünde yakalanmayan bir istisna `FAILSAFE` durumuna geçişe neden olur. Mevcut `safety_manager.py` içinde ayrıntılı bir failsafe politikası uygulanmamıştır; ArduPlane'in kendi emniyet davranışları ve komut başarısızlığı logları kullanılır. Bu durum raporun sınırlamalar bölümünde ayrıca belirtilmiştir.

Durum makinesi 20 Hz kontrol döngüsünde çalışır. Durum geçişleri tek yöndedir; örneğin TERMINAL'den CRUISE'a geri dönülmez. Bu yaklaşım mission upload veya arm gibi tek seferlik işlemlerin yanlışlıkla tekrarlanmasını engeller.

| Durum grubu | Durumlar | Tamamlanma şartı |
|---|---|---|
| Hazırlık | INIT, CONNECTING, MISSION_UPLOAD | DDS telemetri, MAVLink heartbeat ve mission ACK |
| Koordinasyon | WAIT_PEERS, WAIT_TAKEOFF_SLOT | Gerekli peer planları ve hesaplanan kalkış anı |
| Kalkış | ARMING, TAKEOFF, CLIMB | Arm onayı, 100 m ve ardından 400 m MSL |
| Uçuş | CRUISE, TERMINAL | Rota takibi, zaman kontrolü ve 2 km terminal girişi |
| Tamamlama | ARRIVED, RTL, DONE | 5 m çember girişi ve RTL heartbeat doğrulaması |
| Hata | FAILSAFE | Görev thread'inde yakalanmayan istisna |

Durumlar aynı zamanda `VehicleStatus.msg` içinde sayısal sabit olarak tanımlıdır. Böylece bir agent'ın yayınladığı `mission_state` diğer agent ve görselleştirici tarafından aynı anlamla yorumlanır. Durum makinesi ArduPlane'in uçuş moduyla aynı şey değildir: örneğin CRUISE ve TERMINAL uygulama durumlarıyken araç her ikisinde de AUTO modunda olabilir.

## 4.2 Jeodezi, rota ve sapma

Uzun mesafeler GeographicLib'in WGS84 ters jeodezik çözümüyle hesaplanır. Kısa mesafeli rota sapması ve çember kesişimi için hedef veya bacak başlangıcı merkezli yerel doğu-kuzey düzlemi kullanılır.

WGS84, GPS koordinatlarının dayandığı dünya modelidir. Enlem ve boylam farkını doğrudan metre kabul etmek özellikle doğu-batı yönünde hatalı sonuç verir. Bu nedenle kilometre ölçeğindeki rota uzunluklarını GeographicLib ile, birkaç yüz metrelik çember ve doğru parçası işlemlerini ise yerel düzlemde hesapladım.

Yerel dönüşüm yaklaşık olarak aşağıdaki biçimdedir:

$$x = R\cos(\varphi_0)(\lambda-\lambda_0), \qquad y = R(\varphi-\varphi_0)$$

Bir konumun rota bacağı üzerindeki izdüşüm oranı:

$$u = \operatorname{clamp}\left(\frac{(p-a)\cdot(b-a)}{\lVert b-a\rVert^2},0,1\right)$$

En kısa rota sapması:

$$d_{\mathrm{sapma}}=\left\lVert p-[a+u(b-a)]\right\rVert$$

Bu hesabın doğru parçasına göre yapılması önemlidir. Sonsuz doğruya göre ölçüm yapılırsa araç waypoint'in ötesine geçtiğinde gerçek sapma olduğundan küçük görünür.

Aktif waypoint, araç waypoint kabul yarıçapına girdiğinde veya bacak sonunu izdüşüm olarak geçtiğinde ilerletilir. Kalan mesafe güncel konumdan aktif waypoint'e mesafe ile sonraki bütün bacakların toplamıdır.

## 4.3 Kalan mesafe ve ETA

`EtaEstimator` aktif waypoint'i ve rota üzerindeki ilerlemeyi izleyen hafif kestirim katmanıdır. Aktif waypoint yönündeki birim vektör `ê` ve yatay yer hızı `v_g` için ilerleme hızı:

ETA, aracın mevcut bilgiyle hedefe ulaşmasına kalan tahmini süredir. Bu değer planlanan varış zamanı değildir. Plan, koordinasyon algoritmasının araca verdiği hedef zamandır; ETA ise aracın mevcut hız ve rüzgârla ne zaman ulaşacağını söyler. Kontrolcü bu ikisinin farkını kapatmaya çalışır.

$$V_{\mathrm{ilerleme}}=\vec v_g\cdot\hat e$$

Bu değer dönüş sırasında kısa süreli düşebildiği için 3 saniye zaman sabitli alçak geçiren filtre uygulanır. Tanılama ETA'sı kalan rota mesafesinin en az 3 m/s kabul edilen filtreli ilerleme hızına bölünmesiyle elde edilir. Alt sınır, dönüş sırasında sıfıra yaklaşan izdüşümün ETA'yı sonsuza götürmesini engeller.

Zaman kontrolünde kullanılan `_model_eta_s` ise yalnız bu anlık bölme değildir. Güncel konumdan başlayan kalan nominal rota, komut edilen hava hızı ve geçerli rüzgâr vektörüyle bacak bacak hesaplanır. Böylece araç waypoint dönüşünde olsa bile gelecekteki bütün rotanın süresi tek bir düşük hız örneğine bağlanmaz.

İlk yaklaşımda kalan 2504 m doğrudan 25,7 m/s yer hızına bölündüğünde 97,4 s bulunuyordu. Araç aynı büyüklükte hızla dönüş yaparken bacak doğrultusundaki bileşen 19,7 m/s'ye düştüğünde sonuç 127,1 s oluyordu. Araç fiziksel olarak 30 saniye gecikmediği hâlde kontrolcü böyle bir gecikme görüyordu. Model tabanlı ETA'ya geçişin nedeni bu ölçümdür.

## 4.4 Rüzgâr kestirimi

Temel vektör ilişkisi:

$$\vec v_{\mathrm{yer}}=\vec v_{\mathrm{hava}}+\vec v_{\mathrm{rüzgar}}$$

Hava hızı AP_DDS tarafından gövde FLU ekseninde verildiği için doğrudan yer hızından çıkarılamaz. Önce quaternion yönelimiyle ENU eksenine döndürülür:

FLU ekseninde x ileri, y sol ve z yukarı yönü gösterir. ENU ekseninde x doğu, y kuzey ve z yukarıdır. Quaternion ise uçağın üç boyutlu yönelimini tek bir dönüş nesnesiyle ifade eder. Yalnız pusula yönünü, yani yaw açısını kullanmak tırmanış ve alçalıştaki pitch etkisini kaybettirir.

$$\vec v_{\mathrm{rüzgar}}=\vec v_{\mathrm{yer}}-R(q)\vec v_{\mathrm{hava,gövde}}$$

Bu dönüşümde yaw kadar pitch de önemlidir. Tırmanışta gövde ileri hızının yatay izdüşümü azalır. Pitch ihmal edilirse bu azalma rüzgâr gibi yorumlanır.

Ham doğu ve kuzey rüzgâr bileşenleri 2,5 s zaman sabitli birinci derece filtreyle süzülür:

$$\alpha=\frac{\Delta t}{\tau+\Delta t}, \qquad \hat w_k=\hat w_{k-1}+\alpha(w_k-\hat w_{k-1})$$

Filtre 7,5 s çalışmadan rüzgâr geçerli kabul edilmez. Yön açısı yerine vektör bileşenlerinin filtrelenmesi 359° ile 1° arasındaki sarma problemini önler.

Bir rota bacağında rüzgârın bacak doğrultusundaki ve çapraz bileşenleri ayrılır. Sabit hava hızında bacak boyunca elde edilen yer hızı:

$$V_g=\sqrt{V_a^2-w_{\perp}^2}+w_{\parallel}$$

Rota süresi bütün bacakların toplamıdır:

$$t_{\mathrm{rota}}=\sum_j\frac{L_j}{V_{g,j}}$$

Hız komutu bir anda değişemediği için E/L ulaşılabilirlik hesabında 1,5 m/s² sınırla hız rampası sayısal olarak entegre edilir.

Aynı rüzgâr bütün araçların süresini aynı yönde değiştirmez. Örneğin geliştirme kaydında 3,7 m/s ve 268° rüzgâr HA-1'in nominal süresini 533 s'den 503 s'ye indirirken HA-3'ün süresini 405 s'den 418 s'ye çıkardı. Rotaların doğrultuları farklı olduğu için biri ağırlıklı kuyruk, diğeri karşı veya yan rüzgâr gördü. Bu nedenle tek bir rüzgâr katsayısı yerine her bacak ayrı hesaplanır.

## 4.5 Merkeziyetsiz zamanlama çıpası

Araç `i` için en erken ulaşılabilir mutlak varış anı `E_i`, ardışık hedef farkı `Δ=20 s` olsun. Bütün araçların yetişebileceği ortak çıpa:

Çıpa burada bütün araçların varış sırasını kurduğu ortak zaman başlangıcıdır. Bir araç diğerlerine bu değeri göndermiyor. Her agent, taze peer mesajlarında gördüğü en erken ulaşılabilir varışları aynı formüle koyup çıpayı kendi içinde hesaplıyor.

$$A=\max_i[E_i-(i-1)\Delta]$$

Her aracın hedef varış anı:

$$T_i=A+(i-1)\Delta$$

Bu hesap bütün agent'larda aynı girdiler ve aynı sıralama kuralıyla yapılır. Dolayısıyla ayrıca bir lider seçilmeden aynı sonuç elde edilir. Çıpa, en yavaş veya en geç ulaşabilen aracın gerisine kimsenin düşmemesini sağlar.

Plan taahhüt edildikten sonra önceki araçların yayınladığı taahhütler de dikkate alınır. Takip eden araç, kendinden önceki herhangi bir aracın planından 20 saniyelik kimlik farkını ekleyerek aday üretir ve en geç adayı seçer. Plan revizyonları yalnız ileri yönde yapılır; araç daha erken bir zamana zorlanmaz.

Nominal rota süreleri yaklaşık olarak HA-1 için 533 s, HA-2 için 538 s ve HA-3 için 405 s'dir. HA-1'in beklemeden kalkıp 533. saniyede varacağı kabul edilirse hedefler 533, 553 ve 573 s olur. HA-2'nin kalkış gecikmesi `553-538=15 s`, HA-3'ün gecikmesi `573-405=168 s` çıkar. Son değişken rüzgâr koşusunda ölçülen değerler 14,6 s ve 168,2 s olmuştur. Büyük zaman farkının uçuşa bırakılmadan yerde hesaplanabildiğini bu örnek gösterir.

Plan yalnız ileri taşınabilir:

$$T_{\mathrm{yeni}}=\max(T_{\mathrm{nominal}},T_{\mathrm{çıpa}})$$

Mandal, bir değerin yalnız tek yönde değişmesine izin veren kuraldır. Buradaki mandal peer ETA gürültüsünün planı ileri geri oynatmasını engeller. Bunun karşılığında hatalı biçimde çok ileri taşınmış bir plan kendiliğinden geri alınamaz. Bu nedenle çıpaya giren ulaşılabilirlik değerlerinin filtrelenmesi ve tazelik kontrolü önemlidir.

## 4.6 Kalkış gecikmesi

Rüzgâr düzeltilmiş tahmini uçuş süresi `t̂_uçuş` ise kalkış zamanı:

$$T_{\mathrm{kalkış},i}=T_i-\hat t_{\mathrm{uçuş},i}$$

HA-1 referans aracı olarak ilk uygun anda kalkar. HA-2 ve özellikle HA-3, daha kısa rotalarını havada uzatmak yerine yerde bekler. Yerde bekleme ilave uçuş süresi ve hava sahası kullanmadığı için vaka maddesi 8 açısından tercih edilen çözümdür.

Araç yerde beklerken peer planları ve rüzgâr bilgisi değişirse kalkış zamanı yeniden senkronize edilir. Arm işlemi yalnız hesaplanan zaman geldiğinde başlar; manuel tetikleme yapılmaz.

Kalkış gecikmesi sabit sıra numarası çarpı süre değildir. HA-2'nin rotası HA-1'den biraz daha uzun olduğu için ikinci sırada olmasına rağmen yalnız yaklaşık 15 s bekler. HA-3 en kısa rotaya sahipken en son varmak zorunda olduğu için yaklaşık 168 s bekler. Gecikme, sıra gereksinimi ile her aracın kendi rüzgâr düzeltilmiş rota süresi arasındaki farktır.

## 4.7 Hız kontrolcüsü

Planlanan varışa kalan zaman:

$$t_{\mathrm{kalan}}=T_{\mathrm{plan}}-t_{\mathrm{şimdi}}$$

Modelin tahmin ettiği ETA ile zaman hatası:

$$e_t=ETA-t_{\mathrm{kalan}}$$

Pozitif hata aracın geç kalacağını, negatif hata erken varacağını gösterir. Ölü bant dışında gereken hava hızı mevcut komut üzerinden oranlanır:

$$V_{\mathrm{gerekli}}=V_{\mathrm{komut}}\frac{ETA}{t_{\mathrm{kalan}}}$$

Sonuç 13–28 m/s aralığına doyurulur. Ardından bir kontrol çevrimindeki değişim `1,5·Δt` ile sınırlandırılır. Yeni komut önceki gönderilen değerden en az 0,1 m/s farklı değilse MAVLink trafiği oluşturmamak için gönderilmez. Zaman hatası ±0,5 s ölü bant içindeyse mevcut hız korunur.

Doygunluk, istenen değerin uçuş zarfı sınırında tutulmasıdır. Ölü bant, küçük hata varken kontrolcünün gereksiz komut üretmediği aralıktır. Rate limit ise hız hedefinin saniyede ne kadar değişebileceğini sınırlar. Bu üç koruma olmadan ETA gürültüsü hava hızı komutunu sürekli ileri geri oynatabilir veya uçaktan fiziksel olarak uygulanamayacak bir hız istenebilir.

Örnek olarak model ETA'sı 102,4 s, plana kalan süre 97,4 s ve mevcut komut 22,9 m/s olsun. Hata `+5,0 s` olduğu için araç geç kalmaktadır. Oran hesabı 24,07 m/s ister. Bu değer hız sınırları içinde olsa da 20 Hz çevrimde izin verilen tek adım `1,5×0,05=0,075 m/s` olur. İlk çevrimdeki birikim 0,1 m/s gönderme eşiğini geçmediği için paket gönderilmez; sonraki çevrimde biriken değişim gönderilir. Böylece hız komutu bir anda sıçramaz.

İlk rate limit 0,5 m/s² idi. Bir koşuda 878 kontrol adımının 865'inde, yani yüzde 98,5 oranında limit aktif kaldı. Kontrolcü neredeyse bütün uçuşta istediği hıza ulaşamadan rampa izliyordu. Sınır 1,5 m/s² yapıldığında bu oran düşmüş ve son bacakta kullanılabilir hız yetkisi artmıştır.

## 4.8 E/L sınırları ve terminal rezerv

Yalnız en erken varışı bilmek yeterli değildir. Araç uzun süre asgari hızda giderse daha sonra beklenmeyen kuyruk rüzgârına karşı yavaşlama yetkisi kalmaz. Bu nedenle kalan rota için iki uç süre hesaplanır:

- `E`: azami hız zinciri ve rüzgâr adaylarıyla en erken ulaşılabilir varış
- `L`: asgari hız zinciri ve rüzgâr adaylarıyla en geç ulaşılabilir varış

Hedef zamanın ulaşılabilir olması için aşağıdaki aralık korunmalıdır:

$$E_i\le T_i\le L_i$$

Rota zaman bloklarına ayrılır ve seçilen operasyonel rüzgâr zarfındaki adaylar her blok için değerlendirilir. Hız rampaları iki sınır için ayrı taşınır. Bu hesap gelecekteki rüzgârı bildiğini iddia etmez; tanımlı zarf içinde kalan kontrol yetkisini ölçer.

Bu raporda robust veya sağlam hesap, tek bir ölçülen rüzgâr değerine göre iyi sonuç vermek yerine seçilen rüzgâr adaylarının tamamında kalan ortak kontrol aralığını aramak anlamına gelir. Sınırsız büyüklükte veya anlık değişen rüzgâr için garanti anlamına gelmez.

Kod, rüzgâr hızını 0–10 m/s arasında 2 m/s adımla ve yönü 30° adımla tarar. Rota nominal hızla yaklaşık 180 saniyelik bloklara bölünür. Her blokta azami hıza giden zincirin rüzgârlar arasındaki en geç sonucu robust `E`, asgari hıza giden zincirin en erken sonucu robust `L` tarafını oluşturur. Bu kesişim yaklaşımı, aynı hedef zamanın incelenen bütün bozucu adaylarında kontrol edilebilir kalmasını amaçlar. Zarf aşırı geniş seçilirse `E>L` olabilir; bu durumda ortak garanti penceresi yoktur.

Hedefe 2 km kala loiter yasak olduğu için rotanın bu çembere son dışarıdan giriş noktası görev başında bulunur. Bu nokta son yasal kapıdır. Kapıdan hedefe en erken süre `t_E`, en geç süre `t_L` ise kapıdan geçiş penceresi:

$$T_i-t_L\le T_{kapı}\le T_i-t_E$$

Araç kapıya bu pencereden çok erken gelecekse GUIDED moda geçip kapı çevresinde bekler. Çıkış anında AUTO'ya dönerek rotaya devam eder. Bekleme yalnız bir kez kullanılabilir. Hedef mesafesi veya rota sapması güvenlik payına yaklaşırsa bekleme iptal edilir.

Loiter, sabit kanatlı uçağın bir merkez çevresinde daire çizerek beklemesidir. Uçak havada sabit duramadığı için bekleme bu şekilde yapılır. Kapı merkezi hedefe 2500 m uzaklıkta, loiter yarıçapı 80 m'dir. Böylece çemberin hedefe en yakın noktası yaklaşık 2420 m olur ve vaka belgesindeki 2 km yasağının dışında kalır. Araç hedefe 2200 m yaklaşırsa bekleme ayrıca zorla bırakılır.

Sabit 8 m/s koşusundaki HA-3 örneğinde kapı hedefe düz olarak 2500 m, kapıdan sonraki nominal rota ise yaklaşık 5338 m uzaklıktadır. Ölçülen terminal sınırları `E=252,9 s` ve `L=381,4 s` olmuştur. Erken ve geç marjları toplamı 23 s çıkarıldığında kullanılabilir pencere:

$$W=(381{,}4-252{,}9)-(3+20)=105{,}5\ \mathrm{s}$$

Araç hesaplanan bırakma anından erken geldiği için kapı loiteri uygulanmış, toplam havada bekleme 62 s ölçülmüş ve HA-2→HA-3 farkı 20,09 s olmuştur. Loiter süresinin ilk tahminden farklı çıkması sabit kanatlı aracın çemberi tamamlaması ve AUTO rotasına yeniden bağlanmasıyla ilgilidir.

Kapı geçildikten sonra terminal rezervi ayrı çalışır. Hedef zamana göre en hızlı varışla kalan geç kalma payı ve en yavaş varışla kalan erken gelme payı hesaplanır. Erken rezerv 18 s'nin altına düşerse asgari hız, geç rezerv 10 s'nin altına düşerse azami hız zorlanır. İki rezerv de düşükse zaman hatasının işaretine göre taraf seçilir. 2 s histerezis, sınır çevresinde hızlı mod değişimini önler.

Histerezis, bir moda giriş ve o moddan çıkış için farklı eşik kullanılmasıdır. Rezerv tam sınır çevresinde dolaşırken kontrolün her çevrimde asgari ve azami hız arasında geçiş yapmasını önler.

## 4.9 Varış tespiti

İki ardışık hedef merkezli konum `p_0`, `p_1` ve kabul yarıçapı `r=5 m` için uçuş parçası:

$$p(f)=p_0+f(p_1-p_0), \qquad 0\le f\le1$$

Çember girişi aşağıdaki ikinci derece denklemin uygun köküyle bulunur:

$$\lVert p_0+f(p_1-p_0)\rVert^2=r^2$$

Giriş oranı `f` bulununca varış zamanı iki monotonic zaman damgası arasında doğrusal olarak hesaplanır. Varış mandallanır ve ikinci kez üretilmez. Sonrasında RTL komutu gönderilir.

Örneğin iki konum örneği 100 ms aralıklı ve çember kesişimi parçanın yüzde 35'inde ise varış zamanı ilk örnekten 35 ms sonrası olarak kaydedilir. Aracın ikinci örnekte 5 m çemberinin öbür tarafına geçmiş olması varışın kaçırılmasına yol açmaz. En yakın örneği doğrudan varış kabul etmek yerine kesişim zamanını kullanmak, araçlar arası fark ölçümünün telemetri yayın fazına bağımlılığını azaltır.

## 4.10 S-manevrası kararı

Yol uzatma için S-manevrası geliştirmenin erken aşamalarında denendi. İlk uygulamada manevra mevcut konumdan hedefe göre planlanırken sapma aktif rota bacağına göre ölçülüyordu. Bu referans uyuşmazlığı nedeniyle planda 500 m altında görünen geometri uçuşta 800 m'nin üzerinde sapma oluşturdu.

S-manevrası, uçağın rota doğrultusunun iki yanına dönüşler yaparak aynı hedefe daha uzun yoldan gitmesidir. Amaç hız daha fazla azaltılamadığında zaman kazanmaktır. Vaka belgesi bunu izin verilen yöntemlerden biri olarak veriyor, zorunlu tutmuyor.

Planlayıcı daha sonra kalan rota poligonunu izleyip kendi ürettiği yörüngeyi aynı referansa göre ölçecek biçimde düzeltildi. Bununla birlikte ArduPlane GUIDED hedefini ulaşılacak waypoint yerine çevresinde dönülecek merkez olarak ele aldı. Kayan takip noktası yaklaşımı yürütülebilse de manevranın terminal rezerv ve ETA modeliyle birlikte çalışması sistemi gereksiz karmaşıklaştırdı.

Vaka belgesi S-manevrasını zorunlu tutmuyor; kalkış gecikmesi, hız yönetimi ve yasal kapı loiteri kullanılabilecek yöntemler arasında. Bu nedenle nihai çözümde S-manevrası kaldırıldı. Deneme sonuçları geliştirme sürecinin bir parçası olarak korunuyor, fakat çalışan algoritmanın özelliği olarak sunulmuyor.

# 5. Geliştirme Süreci, Testler ve Sonuçlar

## 5.1 Geliştirme sırası

Çalışma önce ortam ve DDS mimarisinin doğrulanmasıyla başladı. ArduPlane binary'sinin DDS desteği, farklı domain'lerdeki iki SITL'in ayrışması ve tek süreçte iki `rclpy.Context` kullanımı küçük deneylerle kontrol edildi. Ardından ROS mesaj paketi, üç araç konfigürasyonu, görev üretimi ve tek araç AUTO uçuşu oluşturuldu.

Üç araç aynı anda çalıştıktan sonra merkeziyetsiz çıpa ve kalkış gecikmesi eklendi. İlk uçuşlarda yalnız ölçülen yer hızına bölünen ETA kullanıldığı için waypoint dönüşlerinde 15 saniyeye varan salınım görüldü. Model tabanlı rota süresine geçildiğinde bu salınım yaklaşık ±0,7 s düzeyine indi.

Rüzgâr kestiriminde gövde eksenindeki hava hızının yalnız yaw ile döndürülmesi tırmanışta hataya neden oldu. Tam quaternion dönüşümüne geçilerek pitch etkisi dahil edildi. `ARSPD_USE=1` ve `EK3_WIND_P_NSE=1.0` seçimleriyle ArduPlane EKF rüzgâr durumunun hava hızı ölçümünü kullanması ve değişimi daha hızlı izlemesi sağlandı.

Değişken rüzgâr testinde HA-3'ün son yaklaşmada erken kaldığı görüldü. İlk hız rezervi eşikleri sorunu kalıcı çözmedi. Kök neden, sistemin en erken varış sınırını izleyip en geç varış ve kalan yavaşlama yetkisini izlememesiydi. E/L ulaşılabilirlik zarfı, terminal rezerv ve son yasal kapı bu ölçümden sonra eklendi.

## 5.2 Önemli problemler ve çözümler

| Belirti | Kök neden | Uygulanan çözüm |
|---|---|---|
| ETA dönüşlerde yaklaşık 30 s sıçrıyor | Anlık yer hızının tamamına bölme | Rüzgâr ve rota bacağı tabanlı süre modeli |
| Tırmanışta yanlış rüzgâr | Pitch'in dönüşümde ihmal edilmesi | Tam quaternion ile FLU → ENU dönüşümü |
| Peer rüzgârı bazen zararlı | Tamamlanan aracın donmuş rüzgârı geçerli kalıyor | Aktif uçuş durumu ve tazelik kontrolü |
| HA-3 terminalde erken kalıyor | Asgari hız yetkisi daha önce tüketiliyor | E/L sınırları ve terminal rezerv |
| S planı ile ölçülen sapma uyuşmuyor | İki farklı geometrik referans | Tek rota poligonu referansı, ardından S'nin kaldırılması |
| İlk değişken profil sürekli kalıyor | 110°/20 s dönüş fiziksel olarak aşırı | 30° adım ve 60 s zaman sabiti |
| Başarılı koşu videoda başarısız yazıyor | Analiz kalıbı Türkçe/ASCII log farkını okumuyor | Her iki log biçiminin kabul edilmesi |

## 5.3 Doğrulama yöntemi

Doğrulamayı birim testi, entegrasyon deneyi ve tam görev koşusu olarak üç seviyede yaptım. Birim testleri gerçek bir SITL uçuşu başlatmaz. Her test belirli bir giriş hazırlar, ilgili fonksiyonu çalıştırır ve çıkan değeri önceden bilinen sonuçla karşılaştırır. Örneğin HA-2'nin planlanan varışının HA-1'den 20 saniye sonra olması, zaman hatası pozitifken hava hızı komutunun artması, hız komutunun 15 ile 28 m/s arasında kalması ve iki telemetri örneği arasında 5 m çemberine girilmişse varışın yakalanması ayrı ayrı kontrol edilir.

Toplam 161 kontrol durumu 10 birim test dosyasında bulunuyor. Bunlar jeodezik hesapları, çember girişini, ETA ilerlemesini, rüzgâr dönüşümünü ve filtresini, merkeziyetsiz çıpa hesabını, kalkış zamanını, hız sınırlayıcılarını, peer tazeliğini, görev listesini ve görev durum geçişlerini kapsıyor. `python3 -m pytest tests/unit -q` komutu bu kontrolleri çalıştırır. Bütün beklenen değerler sağlanırsa çıkış kodu 0, en az bir karşılaştırma sağlanmazsa sıfırdan farklı olur ve hatalı testin adı gösterilir. Birim testlerin geçmesi hesap ve karar mantığının verilen örneklerde doğru çalıştığını gösterir; DDS bağlantısını veya üç aracın birlikte uçuşunu tek başına kanıtlamaz.

Entegrasyon tarafında üç seviye kullanıldı:

1. `dual_context_poc.py` ile tek süreçte iki ROS context'in çalışması ve araç domaininden koordinasyon domainine mesaj sızmaması
2. Tek araçla görev yükleme, arm, AUTO uçuş ve RTL akışının denenmesi
3. Üç SITL ile sakin, sabit ve değişken rüzgâr senaryolarının baştan sona çalıştırılması

Tam görev koşusunda `scripts/analyze_run.py` koordinasyon domaini 10 üzerinden araçların yayınladığı gerçek varış zamanlarını alır. `agents.log` dosyasından hedefe en yakın geçişi, en büyük rota sapmasını ve yerde veya havada geçen bekleme sürelerini çıkarır. Koşunun geçmesi için üç aracın HA-1, HA-2, HA-3 sırasında varması, iki ardışık zaman farkının 20 s ±1 s aralığında kalması, her aracın hedefe en fazla 5 m yaklaşması ve rota sapmasının 500 m'yi aşmaması gerekir. Vaka belgesi 20 saniye için sayısal tolerans vermediği için ±1 s değeri teslim kabul sınırı olarak benim tarafımdan seçildi. Bu dört koşuldan biri sağlanmazsa betik sıfırdan farklı çıkış kodu üretir ve başarısızlık nedenini yazar.

Havada bekleme süresi bu dört geçme koşulundan biri değildir. Belge havada kalma ve bekleme süresinin en aza indirilmesini istiyor, fakat hedef bölgesinin dışındaki loiter için kesin bir sıfır saniye şartı vermiyor. Bu yüzden süre ayrıca ölçülüp senaryolar arasında karşılaştırılıyor. Hedefin 2 km çevresinde loiter yapılmaması ise görev yöneticisinin terminal geçişi ve loglar üzerinden ayrıca kontrol ediliyor.

## 5.4 Senaryolar ve toplu sonuçlar

| Senaryo | Koşu 1 sapmaları | Koşu 2 sapmaları | Havada bekleme |
|---|---|---|---:|
| Sakin | -0,00 / -0,00 s | -0,00 / -0,01 s | 0 s |
| Sabit 8 m/s | -0,20 / +0,02 s | -0,29 / +0,09 s | 62 s |
| Değişken rüzgâr | -0,13 / +0,10 s | -0,11 / +0,17 s | 0 s |

Tablodaki iki değer sırasıyla HA-1→HA-2 ve HA-2→HA-3 aralıklarının 20 saniyeden farkıdır. Bunlar geliştirme sırasında tutulan koşu özetlerinden alınmıştır. Altı koşunun tamamında varış sırası doğru, hedef geçişi 5 m içinde ve rota sapması 500 m altındadır. Son video koşusunun ölçümleri ayrıca videonun sonuç bölümünde gösterilmiştir.

| Ölçüt | Sınır | En kötü gözlenen |
|---|---:|---:|
| Ardışık zaman farkı hatası | ±1,0 s | +0,17 s |
| Hedefe yaklaşma | ≤5 m | 4,99 m |
| Rota sapması | ≤500 m | 127 m |
| Varış sırası | HA-1/2/3 | Doğru |

[[IMAGE:results.png|Şekil 5. Üç senaryodaki ardışık varış farkı hataları]]

## 5.5 Son video koşusu

Son kayıt değişken rüzgâr senaryosunda alındı. Bu koşuda ölçülen sonuçlar:

| Araç veya aralık | Ölçüm |
|---|---:|
| HA-1 hedefe yaklaşma / rota sapması | 4,43 m / 30 m |
| HA-2 hedefe yaklaşma / rota sapması | 4,92 m / 21 m |
| HA-3 hedefe yaklaşma / rota sapması | 3,83 m / 83 m |
| HA-1 → HA-2 | 20,07 s |
| HA-2 → HA-3 | 19,94 s |
| Toplam yerde bekleme | 183 s |
| Toplam havada bekleme | 0 s |

Video simülasyonun başlatılmasını, mission yüklenmesini, üç kalkışı, seyir ve terminal geçişlerini, hedef varışlarını ve RTL komutlarını kapsar. Uzun uçuş bölümleri 20 kat hızlandırılmış, kritik olaylar normal hızda ve kısa Türkçe altyazılarla gösterilmiştir. Kapanış bölümünde aynı koşuda ölçülen varış farkları ve görev sonuçları yer alır.

## 5.6 Optimal çözüm değerlendirmesi

Vaka belgesindeki optimal ifadesi için kesin bir amaç fonksiyonu verilmemiştir. Bu çalışmada ölçütü havada geçen ilave bekleme süresini en aza indirmek olarak yorumladım. Sakin ve değişken rüzgâr koşularında havada bekleme sıfırdır. Rota süresi farkının büyük bölümü HA-2 ve HA-3'ün yerde farklı zamanlarda kalkmasıyla karşılanır.

Sabit rüzgâr senaryosunda toplam 62 s yasal kapı loiteri görülmüştür. Bu, hedef çevresindeki yasak bölgenin dışında ve yalnız hız kontrolünün ulaşılabilirlik penceresini koruyamadığı durumda uygulanır. Dolayısıyla sistem mutlak matematiksel optimum iddiasında bulunmuyor; verilen kontrol araçları içinde yerde beklemeyi önceleyen ve ölçülebilir havada beklemeyi sınırlayan bir çözüm sunuyor.

## 5.7 Çalıştırma ve teslim dosyaları

Sistemin ana giriş noktası `run.sh` dosyasıdır. Bu betik işletim sistemi üzerindeki ROS 2 kurulumu, Python paketleri, ArduPlane binary'si ve Micro XRCE-DDS Agent varlığını kontrol eder. Çalışma alanı daha önce `colcon build` ile derlenmişse ROS ortamını ve proje kurulumunu kaynak alır, önceki koşudan kalan süreçleri durdurur ve seçilen senaryoyu başlatır.

Kullanılan temel komutlar şunlardır:

```bash
./run.sh calm
./run.sh steady
./run.sh variable
```

`calm` rüzgârsız, `steady` sabit 8 m/s ve `variable` zamanla değişen rüzgâr koşusudur. `./run.sh all` üç ana senaryoyu sırayla çalıştırıp her koşudan sonra analiz yapar. `extreme` seçeneği geliştirme sınırını görmek içindir ve ana kabul senaryolarına dahil değildir.

Teslimde yalnız Python kaynakları değil, ROS paketini derlemek için gereken `package.xml`, `setup.py`, `setup.cfg`, `CMakeLists.txt`, launch, YAML, ArduPlane parametreleri ve özel mesaj dosyaları da bulunur. `ros2_ws/build`, `install` ve `log` dizinleri makineye bağlı üretim çıktıları olduğu için kaynak tesliminin parçası değildir; alıcı çalışma alanını kendi ortamında yeniden derler.

`oasy_interfaces` içinde iki mesaj tanımı vardır. `VehicleStatus.msg` aktif koordinasyon mesajıdır. `MissionEvent.msg` geliştirme sırasında olayları ayrı kanala taşıma düşüncesiyle oluşturuldu ancak çalışan sistemde kullanılmadı. Dosyayı kaynak geçmişinin parçası olarak tutuyorum ve raporda aktifmiş gibi göstermiyorum.

Python bağımlılıkları `requirements.txt` içinde sürümleriyle belirtilmiştir. Aynı paketlerin çevrimdışı kurulabilmesi için `wheelhouse` dizininde `.whl` dosyaları bulunur. ROS 2 paketleri pip üzerinden kurulmaz; Ubuntu 22.04 ve ROS 2 Humble ortamından gelir.

ArduPilot tarafında özel kaynak kod değişikliği yapmadım. Kullanılan kaynak `Plane-4.6.3` etiketi ve `3fc7011a7d3dc047cbb17d8bd98ee94577d144c6` commit'idir. Teslim paketinde gerçek ArduPilot kaynak dizini ve DDS etkin derlenmiş `arduplane` binary'si yer almalıdır. Geliştirme klasöründeki sembolik bağlantı doğrudan teslim edilmemelidir.

Simülasyon videosu kodların başlatılması, görevlerin yüklenmesi, kalkışlar, seyir, terminal girişi, üç varış ve RTL komutlarını kapsar. Uzun seyir bölümleri 20 kat hızlandırılır; kritik olaylar normal hızda kısa altyazılarla gösterilir. Videonun sonunda aynı koşunun sonuçları gösterilir.

# 6. Vaka İsterlerinin Projedeki Karşılığı

Bu bölümde vaka belgesindeki maddeleri son kez tek tek karşılaştırıyorum. Karşılandı ifadesi yalnız kodda ilgili sabitin bulunması anlamına gelmiyor; mümkün olduğunda uçuş kaydı veya otomatik analiz sonucu da kullanılıyor. Yoruma bağlı maddeleri ayrıca belirtiyorum.

## 6.1 Teknik altyapı

| İster | Uygulama ve kanıt | Değerlendirme |
|---|---|---|
| ArduPlane 4.6.3 | `Plane-4.6.3` etiketi, kaynakta değişiklik yok | Karşılandı |
| Üç eş zamanlı SITL | Üç instance, üç başlangıç noktası, ayrı MAVLink portları | Karşılandı |
| Ubuntu 22.04 | Ubuntu 22.04.5 LTS üzerinde geliştirildi | Karşılandı |
| ROS 2 Humble | Bütün ROS paketleri Humble ile derlendi | Karşılandı |
| Python 3.10.12 | Geliştirme ve koşu ortamı Python 3.10.12 | Karşılandı |
| Doğrudan DDS telemetrisi | AP_DDS ve üç Micro XRCE-DDS Agent, MAVROS yok | Karşılandı |
| Her araç için bağımsız Python node | Aynı paket üç ayrı süreç ve üç YAML yapılandırmasıyla çalışıyor | Karşılandı |
| Merkezi GCS veya master olmaması | MAVProxy kapalı, analiz ve görselleştirme yalnız dinleyici | Karşılandı |

Üç araç aynı bilgisayarda çalışsa da karar yazılımı tek bir merkezi nesnede birleştirilmedi. Her agent kendi telemetrisini, kendi rota modelini ve diğer araçlardan aldığı durum mesajlarını kullanıyor. `run.sh` süreçleri başlatan bir orkestrasyon betiğidir; uçuş sırasında hedef zaman veya hız kararı üretmez. Rüzgâr betiği de araçlara görev komutu vermek yerine yalnız simülasyon bozucusunu uygular.

## 6.2 Girdi koordinatları ve rota

Vaka tablosundaki üç başlangıç konumu `start_sitl.sh` ve araç YAML dosyalarında aynen kullanılır. Her aracın rota dizisi kendi ara noktalarını verilen sırayla içerir. Son eleman bütün araçlarda ortak hedef olan 47.535683 N, 122.228584 W koordinatıdır.

Mission oluşturulurken önce TAKEOFF, ardından YAML sırasındaki waypoint'ler eklenir. Ara noktalar 120 m kabul yarıçapına, ortak hedef 5 m kabul yarıçapına sahiptir. Yapılandırma okuyucu ara nokta yarıçapının 0 ile 400 m arasında olmasını ayrıca doğrular.

## 6.3 Sekiz görev kuralı

### Otomatik kalkış

Görev listesi Python tarafından MAVLink mission protokolüyle yüklenir. ArduPlane'in mission isteği ve son ACK cevabı beklenir. Hesaplanan kalkış anı geldiğinde AUTO modu ve arm komutu yine Python tarafından verilir. Manuel tetikleme, GCS düğmesi veya MAVProxy komutu kullanılmaz.

### Hedef varış sırası ve RTL

Her araç hedef merkezindeki 5 m çembere ilk girişte varmış sayılır. Varış bir kez kaydedilir ve `actual_arrival_monotonic_ns` alanında yayınlanır. Son video koşusunda sıra HA-1, HA-2 ve HA-3 olmuş; hedef geçişleri sırasıyla 4,43 m, 4,92 m ve 3,83 m ölçülmüştür. Varıştan sonra RTL istenir ve heartbeat içinde RTL modu görülmeden görev tamamlandı sayılmaz.

### 400 m MSL irtifa

Rota waypoint'leri `MAV_FRAME_GLOBAL` ile 400 m MSL olarak yazılır. TAKEOFF öğesi 100 m MSL'dir; araç bu ilk güvenli tırmanıştan sonra 400 m MSL'ye çıkar ve CRUISE durumuna geçer. Bu nedenle bütün görev ifadesini kalkış tırmanışı ve RTL hariç seyir ile terminal yaklaşma olarak yorumladım. Bu yorum fiziksel zorunluluktan kaynaklanıyor ve raporda gizlenmiyor.

### Waypoint yarıçapı ve rota sapması

Ara waypoint yarıçapı 120 m ile belgedeki 400 m üst sınırın altındadır. Rota sapması, aracın o anda izlediği nominal rota parçasına en kısa uzaklık olarak hesaplanır. Analiz 500 m üzerini başarısız sayar. Geliştirme koşularında en yüksek değer 127 m, son video koşusunda 83 m olmuştur.

### Yirmi saniyelik zamanlama

Zamanlama dört katmanda yürür: ortak çıpa, yerde kalkış gecikmesi, uçuşta hava hızı kontrolü ve terminal rezervi. Sabit rüzgârda bunlara yasal kapı beklemesi de eklenebilir. Son video koşusunda HA-1 ile HA-2 arasında 20,07 s, HA-2 ile HA-3 arasında 19,94 s ölçülmüştür. Vaka tolerans vermediği için otomatik analizde ±1 s iç sınır kullanılır; altı geliştirme koşusundaki en büyük sapma 0,17 s'dir.

### İki kilometre içinde loiter yasağı

Terminal sınırı 2000 m'dir. Bekleme kapısı 2500 m çemberinin rota üzerindeki son girişinde kurulur ve ArduPlane loiter yarıçapı 80 m'dir. Böylece bekleme çemberi 2 km sınırının dışında kalır. Araç hedefe 2200 m yaklaşırsa veya rota sapması 400 m'ye ulaşırsa bekleme iptal edilir. Terminal içinde yol uzatma veya loiter yoktur; yalnız AUTO rota takibi ve hava hızı kontrolü devam eder.

### Değişken rüzgâr ve sağlamlık

Normal profil 4, 6, 9, 10 ve 7 m/s basamaklarını; 270, 300, 330, 0 ve 30 derece yönlerini kullanır. Basamaklar 180 saniye aralıklıdır ve `SIM_WIND_TC=60 s` ile yumuşatılır. Agent rüzgârı doğrudan simülasyon parametresinden okumaz; AP_DDS yer hızı ve hava hızı ölçümlerinden kestirir. Zamanlama modeli bu vektörü her rota bacağına ayrı uygular. E/L hesabı ayrıca 0–10 m/s ve 30 derecelik yön adaylarını tarar.

Bu sonuç tanımlanan zarf içinde sağlamlıktır. Gelecekteki rüzgâr değişiminin hızına hiçbir sınır verilmezse nedensel bir kontrolcünün tam varış zamanını garanti etmesi mümkün değildir. Extreme profil bu sınırı görünür tutmak için projede kalır ancak başarılı kabul kanıtı olarak kullanılmaz.

### Havada beklemenin azaltılması

Rota süresi farkının büyük kısmı kalkıştan önce hesaplanır. Son video koşusunda HA-2 yaklaşık 14,6 s, HA-3 yaklaşık 168,2 s yerde beklemiş ve toplam havada bekleme sıfır olmuştur. Sabit 8 m/s koşusunda terminal ulaşılabilirliğini korumak için 62 s yasal kapı loiteri gerekmiştir. Bu yaklaşım havada ilave beklemeyi azaltır ancak bütün olası kontrol dizileri üzerinde kanıtlanmış küresel optimum iddiası taşımaz.

## 6.4 Beklenen teslim çıktıları

| Beklenen çıktı | Hazırlanan karşılık |
|---|---|
| ArduPilot kaynağı | Plane 4.6.3 gerçek kaynak dizini ve DDS etkin `arduplane` binary'si |
| Geliştirilen Python dosyaları | Agent, kontrol, kestirim, koordinasyon, analiz, rüzgâr ve görselleştirme araçları |
| Tek giriş noktası | `run.sh` ve ROS 2 launch dosyası |
| Kurulum açıklaması | Kök `README.md`, bağımlılıklar ve çalıştırma komutları |
| Kaynak kod haritası | README ve bu raporun 2.5–2.6 bölümleri |
| Özel ROS mesajları | Colcon build'e hazır `oasy_interfaces` paketi |
| Python wheel dosyaları | `wheelhouse` dizini ve `requirements.txt` |
| Teknik rapor | Bu DOCX dosyası ve düzenlenebilir Markdown kaynağı |
| Simülasyon videosu | Başlatmadan üç varış ve RTL komutlarına kadar düzenlenmiş video |
| Kritik olay yazıları | Kalkış, tırmanış, seyir, terminal, varış, RTL ve sonuç altyazıları |

Teslim ZIP'i oluşturulurken geliştirme ortamındaki `.git`, cache, geçici çıktılar, eski koşu ve gigabaytlarca SITL logu alınmamalıdır. Yalnız son video tutulmalıdır. `ardupilot` geliştirme ortamında sembolik bağlantı olduğu için ZIP'e alınmadan önce gerçek kaynak diziniyle değiştirilmelidir.

## 6.5 Son değerlendirme

Teknik altyapı, rota, otonom kalkış, merkeziyetsiz karar, 5 m hedef, RTL, waypoint yarıçapı, 500 m sapma ve 2 km loiter yasağı doğrudan karşılanıyor. Yirmi saniye sonucu belirlenen iç tolerans içinde, rüzgâr sağlamlığı seçilen operasyonel zarf içinde karşılanıyor. İrtifa maddesi kalkış ve RTL istisnasıyla yorumlanıyor. Optimal madde ise havada beklemeyi önce yerde tüketme hedefiyle ele alınıyor; matematiksel küresel optimum kanıtı verilmiyor.

# 7. Sınırlamalar ve Sonuç

## 7.1 Bilinen sınırlamalar

- Bütün agent'lar aynı bilgisayarda çalışıyor. Ayrı donanımlarda saat senkronizasyonu gerekir.
- Sağlamlık iddiası test edilen 4–10 m/s değişken rüzgâr ve 0,50°/s yön değişimi zarfıyla sınırlıdır.
- Zamanda sınırsız hızla değişen bir rüzgâr için nedensel bir kontrolcünün 20 saniyeyi garanti etmesi mümkün değildir.
- `safety_manager.py` içinde uygulamaya özel ayrıntılı failsafe politikası henüz yoktur.
- Ağ bölünmesi durumunda iki grubun farklı çıpa üretmesini önleyen bir split-brain protokolü bulunmuyor.
- `MissionEvent.msg` tanımlı olmasına rağmen aktif olay kanalı olarak kullanılmıyor.
- Görev RTL komutu ve mod doğrulamasıyla tamamlanıyor; iniş beklenmiyor.
- Sonuçlar SITL ortamına aittir. Gerçek uçakta hava hızı sensörü, bağlantı gecikmesi ve otopilot ayarları yeniden doğrulanmalıdır.

## 7.2 Sonuç

Bu çalışmada üç ArduPlane SITL aracını merkezi bir karar verici olmadan ortak hedefe 20 saniye arayla ulaştıran uçtan uca bir sistem kurdum. AP_DDS telemetrisi araç bazında ayrı domain'lerde tutuldu; araçlar arası koordinasyon ikinci bir ortak domain üzerinden yapıldı. AUTO mission, otomatik arm ve kalkış, hız yönetimi, yasal kapı loiteri ve RTL işlemleri Python ROS 2 agent'ları tarafından yürütüldü.

Zamanlama probleminin yalnız hız kontrolüyle çözülemeyeceği görüldü. Büyük farkın yerde gecikmeyle, uçuş sırasındaki küçük hatanın hız kontrolüyle ve kalan erkenliğin yalnız yasal bölgede beklemeyle ele alınması daha kararlı sonuç verdi. Rüzgâr altında en erken varış kadar en geç varış ve kalan kontrol rezervinin de izlenmesi, değişken rüzgâr senaryosundaki son yaklaşma hatasını kapatan temel mimari değişiklik oldu.

Doğrulanan koşularda sıra, hedef geçişi ve rota sapması şartları sağlandı. Ardışık varış farkındaki en büyük hata 0,17 s olarak ölçüldü. Son değişken rüzgâr video koşusu 20,07 s ve 19,94 s aralıklarla, sıfır havada beklemeyle tamamlandı.

# Ek A. Temel Yapılandırma Değerleri

| Değer | Seçim | Açıklama |
|---|---:|---|
| Seyir irtifası | 400 m MSL | Vaka şartı |
| Kalkış mission irtifası | 100 m MSL | İlk tırmanış öğesi |
| Waypoint kabul yarıçapı | 120 m | 400 m üst sınırın altında |
| Hedef yarıçapı | 5 m | Varış kabulü |
| Terminal loiter yasağı | 2000 m | Hedef merkezli |
| Nominal hava hızı | 22,9 m/s | Rüzgârsız seyir |
| Asgari / azami hava hızı | 13 / 28 m/s | Kontrol doygunluğu |
| Hız değişim sınırı | 1,5 m/s² | Komut rampası |
| Zaman ölü bandı | 0,5 s | Gereksiz hız değişimini önler |
| Durum yayını | 5 Hz | Koordinasyon mesajı |
| Peer eski / kayıp | 2 / 5 s | Mesaj tazeliği |
| Rüzgâr filtresi | 2,5 s | EMA zaman sabiti |
| Rüzgâr oturma süresi | 7,5 s | Geçerlilik eşiği |
| Loiter yarıçapı | 80 m | ArduPlane `WP_LOITER_RAD` |

# Ek B. Çalıştırma ve Doğrulama Komutları

```bash
# çalışma alanını derle
cd ros2_ws
colcon build

# sakin, sabit ve değişken rüzgâr
./run.sh calm
./run.sh steady
./run.sh variable

# bütün ana senaryolar
./run.sh all

# birim testler
python3 -m pytest tests/unit -q

# mevcut koşuyu analiz et
python3 scripts/analyze_run.py --log logs/run_YYYYMMDD_HHMMSS/agents.log
```

# Ek C. Kaynaklar

1. BAYKAR OASY Vaka Çalışması, `OASY_Vaka_2026_07_23_R0.pdf`.
2. ArduPilot kaynak deposu, Plane 4.6.3 etiketi, https://github.com/ArduPilot/ardupilot/tree/Plane-4.6.3
3. ArduPilot Plane 4.6.3 sürüm kaydı, https://github.com/ArduPilot/ardupilot/releases/tag/Plane-4.6.3
4. ArduPilot Plane dokümantasyonu, https://ardupilot.org/plane/
5. ArduPilot Plane uçuş modları, AUTO, GUIDED ve RTL, https://ardupilot.org/plane/docs/flight-modes.html
6. ArduPilot ROS 2 arayüzleri ve AP_DDS konuları, https://ardupilot.org/dev/docs/ros2-interfaces.html
7. ArduPilot ROS 2 ile SITL kurulumu, https://ardupilot.org/dev/docs/ros2-sitl.html
8. ArduPilot ROS 2 over Ethernet ve DDS UDP ayarları, https://ardupilot.org/dev/docs/ros2-over-ethernet.html
9. eProsima Micro XRCE-DDS Agent dokümantasyonu, https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html
10. eProsima Micro XRCE-DDS mimari özeti, https://micro-xrce-dds.docs.eprosima.com/en/latest/introduction.html
11. ROS 2 Humble Quality of Service ayarları, https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
12. ROS 2 Humble Domain ID kavramı, https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html
13. MAVLink Mission Protocol, https://mavlink.io/en/services/mission.html
14. MAVLink Command Protocol ve `COMMAND_ACK`, https://mavlink.io/en/services/command.html
15. MAVLink Common Message Set ve `MAV_CMD_DO_CHANGE_SPEED`, https://mavlink.io/en/messages/common.html
16. GeographicLib Python API ve WGS84 ters jeodezik çözümü, https://geographiclib.sourceforge.io/html/python/code.html
