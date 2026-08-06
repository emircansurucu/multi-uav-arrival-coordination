# Terminal Rezerv

Terminal rezervi aracın son yaklaşmada hız değiştirme yetkisini korur. Yalnız anlık zaman hatasına bakmak, aracı erken dönemde asgari veya azami hıza doyurabilir.

İki rezerv izlenir:
- erken gelmeye karşı yavaşlama rezervi
- geç kalmaya karşı hızlanma rezervi

Erken rezerv 18 s altına düşerse asgari hız, geç rezerv 10 s altına düşerse azami hız zorlanır. İki taraf da düşükse zaman hatasının işareti kullanılır. 2 s histerezis hızlı mod değişimini önler.

Rezerv hesabı kalan rotanın en hızlı ve en yavaş uçuş sürelerine dayanır. Böylece son 300 m içinde öğrenilen rüzgâr değişimine karşı bir miktar kontrol yetkisi korunur.

Kod konumu:
- `mission_manager.py`
- `estimation/wind_estimator.py`