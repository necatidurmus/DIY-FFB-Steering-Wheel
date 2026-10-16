# FFB seri test araci

`ffb_test.py`, ESP32-C3 firmware'indeki `f -100..100` tork kanalini oyun
olmadan test eder. Yalnizca Linux ve Python standart kutuphanesini kullanir.

Direksiyonu fiziksel merkeze getirip guvenli bir ilk test calistirmak icin:

```bash
python tools/ffb_test.py --port /dev/ttyACM0 --initialize constant --force 22 --duration 2
```

Temel efektler:

```bash
python tools/ffb_test.py --port /dev/ttyACM0 sine --amplitude 25 --frequency 0.5 --duration 5
python tools/ffb_test.py --port /dev/ttyACM0 impact --force 30 --pulse-ms 120
python tools/ffb_test.py --port /dev/ttyACM0 corner --force 25 --ramp 1 --hold 2 --release 1
python tools/ffb_test.py --port /dev/ttyACM0 watchdog --force 22
```

Varsayilan guvenlik siniri `%30`'dur. Daha yuksek bir test iki ayri secenek
gerektirir:

```bash
python tools/ffb_test.py --port /dev/ttyACM0 --max-force 50 --allow-high-force constant --force 40 --duration 1
```

`%100` yalnizca mekanik sistem, surucu sicakligi ve acil durdurma hazirken
kisa sureyle denenmelidir:

```bash
python tools/ffb_test.py --port /dev/ttyACM0 --max-force 100 --allow-high-force impact --force 100 --pulse-ms 80
```

Serial Monitor ve test araci ayni portu ayni anda acamaz. Erisim reddedilirse
kullanicinin Arch/CachyOS seri port grubuna dahil oldugunu kontrol et.
