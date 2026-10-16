# STM32F401RCT6 Güvenli Direksiyon Denetleyicisi

Bu proje STM32F401RCT6, AS5600, BTS7960 ve RS-775 için güvenli başlangıçlı bir
direksiyon kontrol altyapısıdır. Firmware derlenebilir durumdadır; donanımda
motor, USB sinyal bütünlüğü ve yön doğrulaması yapılmadan gerçek kullanıma hazır
kabul edilmemelidir.

Şu anda çalışan firmware özellikleri:

- AS5600 ham açı, wraparound ve çok turlu konum izleme
- Filtreli açısal hız, I2C süre/hata/yeniden bağlanma telemetrisi
- 20 kHz TIM4 PWM ve tek noktada yüzde → 8-bit duty dönüşümü
- Yön değişiminde dead-time, manuel 500 ms timeout ve seri FFB 100 ms watchdog
- Merkezleme/seri FFB için varsayılan 30 saniyelik güvenli test oturumu sınırı
- Yumuşatılmış PD merkezleme ve 360/900/1080 derece sanal son durak hesabı
- Yapılandırılabilir genel, merkezleme ve endstop PWM limitleri
- TIM5 tabanlı 50 ms ana-döngü gözetimi ve bağımsız IWDG reset watchdog'u
- CRC/sürüm kontrollü Flash yapılandırması
- Native USB CDC (`SerialUSB`) ve ayrı UART tanılama derlemesi
- Donanımdan bağımsız PWM, sensör unwrap, dead-time, PD, endstop, timeout,
  yapılandırma ve HID ölçekleme testleri

## Güvenlik sınırı

Bu firmware işlevsel güvenlik sertifikalı değildir. `s` komutu fiziksel acil
durdurma değildir; bilgisayar, USB, kesmeler ve MCU çalışıyor olmalıdır.

BTS7960 Enable hattında **harici pull-down**, motor beslemesinde erişilebilir
fiziksel E-stop/kontaktör ve uygun sigorta kullanılmalıdır. Gelecekte aktif LOW
bir E-stop GPIO'su için `Pins::kEmergencyStop` ayrılmıştır, fakat pin fiziksel
donanım kesinleşmediği için varsayılanı `-1` ve devre dışıdır. En güvenli çözüm,
E-stop'un BTS7960 enable/besleme hattını MCU'dan bağımsız kesmesidir.

12 V / 17 A kaynağın yükte kapanması aşırı akım, gerilim çökmesi, kablolama veya
sürücü termal/akım koruması anlamına gelebilir. Akım sensörü bulunmadığı için bu
firmware gerçek motor akımı veya torku ölçmez. Sorun çözülmeden gücü artırmayın.

## Pinler

| STM32F401RCT6 | İşlev | Açıklama |
|---|---|---|
| PB6 | I2C1 SCL | AS5600 SCL, 400 kHz aday hız |
| PB7 | I2C1 SDA | AS5600 SDA |
| PB8 | TIM4_CH3 | BTS7960 RPWM, 20 kHz |
| PB9 | TIM4_CH4 | BTS7960 LPWM, 20 kHz |
| PB5 | GPIO çıkış | BTS7960 R_EN ve L_EN; harici pull-down gerekli |
| PA11 | USB D- | Native USB FS |
| PA12 | USB D+ | Native USB FS |
| PA2 | USART2 TX | Yalnız `f401rct6_uart` yapısında konsol |
| PA3 | USART2 RX | Yalnız `f401rct6_uart` yapısında konsol |

AS5600 modülünün 3,3 V uyumluluğunu doğrulayın. STM32, AS5600, BTS7960 lojik
GND ve 12 V kaynak GND ortak olmalıdır. 12 V hiçbir STM32/AS5600 pinine
bağlanmamalıdır.

Pedal, vites, düğme ve akım sensörü için rastgele pin atanmadı. `WheelInputState`
gelecekte gaz/fren/debriyaj, düğme ve vites verisini taşıyabilir; mevcut firmware
bu donanımlar varmış gibi sahte veri okumaz.

## USB ve saat doğrulaması

Kurulu STM32duino 2.12.0'da doğru CDC API'si `USBSerial` / global `SerialUSB`'dır.
Projede olmayan `USBCDC.h` kullanılmaz. `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC`
çekirdeğin USB Device/CDC katmanını ve PA11/PA12 OTG FS çevre birimini açar.

Generic F401 variantı PLLQ ile **nominal 48 MHz** USB saatini üretir. Ancak
variantın varsayılanı 16 MHz dahili HSI'dır; HSI doğruluğu USB FS toleransını her
kart/sıcaklık koşulunda garanti etmez. ST'nin USB gereksinimi hassas 48 MHz
saattir. Kart üzerindeki HSE kristal frekansı bilinmeden rastgele HSE ayarı
eklenmemiştir. CDC'nin Linux/Windows üzerinde enumerate olduğu ve uzun süre
hatasız kaldığı donanımda doğrulanmalıdır. Güvenilir ürün için kartın gerçek HSE
değerine göre `SystemClock_Config()` hazırlanmalıdır.

CDC DTR bağlantısı daha önce kurulmuşken kaybolursa bütün hareketler durur. UART
fallback yapısı USB'yi kullanmaz:

```bash
pio run -e f401rct6_uart
```

### HID durumu

STM32duino 2.12.0'ın hazır HID seçeneği klavye/fare içindir; güvenilir bir özel
direksiyon joystick API'si veya CDC + özel HID composite API'si sağlamaz. Bu
firmware HID ekseni için test edilmiş `-32767..32767` ölçekleme ve gelecekteki
giriş veri modelini içerir, fakat henüz PC'ye HID joystick raporu göndermez.

USB HID PID Force Feedback descriptor/report handler ve oyun effect engine'i
**uygulanmış değildir**. `f` komutu yalnız seri porttan güvenlik katmanını test
eden imzalı PWM talebidir; gerçek tork, HID PID veya oyun FFB'si değildir.

## Derleme ve yükleme

Gereksinimler: PlatformIO Core, `ststm32` 19.5.0 ve STM32duino 2.12.0.

```bash
cd firmware/stm32f401rct6
pio run -e f401rct6
pio run -e f401rct6 -t upload
pio device monitor -b 115200
```

Varsayılan yükleme yolu STM32'nin ROM'daki USB DFU bootloader'ıdır. Motorun
12 V beslemesini firmware yüklenirken kapalı tutun. DFU'ya giriş sırası:

1. USB'yi ve motorun 12 V beslemesini ayırın.
2. `BOOT0` pinini 3,3 V seviyesine (HIGH) alın.
3. Kartı yeniden besleyin veya `RESET`/`NRST` darbesi uygulayın.
4. USB-C veri hattını bağlayın. `lsusb` çıktısında `0483:df11` ve
   `dfu-util -l` çıktısında STM32 bootloader görülmelidir.
5. `pio run -e f401rct6 -t upload` komutunu çalıştırın.
6. Yükleme bittikten sonra `BOOT0` pinini LOW seviyesine geri alın ve uygulamayı
   başlatmak için yeniden RESET uygulayın. `:leave` isteği uygulamaya geçişi
   dener; BOOT0'ın LOW yapılması sonraki resetler için yine zorunludur.

ROM USB DFU, PA11/PA12 yanında bootloader'ın desteklediği ve çalışan bir
harici saat kaynağı da gerektirir. Kartın HSE kristali/clock donanımı
doğrulanmalıdır. Yalnız güç taşıyan USB-C kablosu kullanılamaz.

Board tanımı uygulamanın USB kimliğini `0483:5740`, ROM DFU kimliğini
`0483:df11` olarak eşler. DFU indirme adresi açıkça `0x08000000`'dır. Komut
yalnız oluşan binary uzunluğunu bu adresten itibaren yazar; `mass-erase`
kullanmaz.

ST-Link varsayılan yükleyici değildir. Yalnız SWD debug veya BOOT0/USB/Flash
sorunlarında kurtarma için kullanılır:

```bash
pio run -e f401rct6_stlink -t upload
```

ROM DFU modunda yükleme yapılabilmesi, normal firmware'in USB HID direksiyon
veya HID PID FFB olarak çalıştığı anlamına gelmez. Bunlar ayrı USB aygıt
katmanlarıdır; HID/HID PID desteği henüz tamamlanmamıştır.

Kalıcı ayarlar F401'in son 128 KB Flash sektöründedir (sector 5,
`0x08020000`). Özel board tanımı uygulamayı ilk 128 KB ile sınırlar; böylece
firmware yapılandırma sektörünün üzerine linklenmez. `cfg save` motoru önce
durdurur, tek buffered erase/program yapar ve kaydı geri okuyup CRC ile doğrular.

## İlk güvenli çalıştırma

İlk testte motorun 12 V hattı **kapalı** olmalıdır.

1. Firmware'i yukarıdaki USB DFU adımlarıyla yükleyin; PB5'in LOW, PB8/PB9
   duty'nin sıfır olduğunu
   osiloskop/lojik analizörle doğrulayın.
2. CDC enumerate olmuyorsa saat/kablo/PA11/PA12 donanımını inceleyin veya UART
   derlemesini kullanın. USB çalışıyor diye yalnızca derlemeden sonuç çıkarmayın.
3. AS5600 `raw`, `sensor_us` ve hata telemetrisini kontrol edin. Mıknatıs zayıf,
   güçlü veya yok hatası bulunmamalıdır.
4. Direksiyonu fiziksel merkeze alın ve `z` gönderin.
5. 12 V hâlâ kapalıyken `r 1`, `l 1`, `r 10`, `l 10` komutlarında PB8/PB9'un
   hiçbir zaman birlikte aktif olmadığını ve Enable davranışını ölçün.
6. Mekanik alan boş ve `s` hazırken 12 V'yi açın. `mx 10` seçin; `r 1`, `l 1`
   ile başlayıp yalnız gerektiği kadar yükseltin. Her komut 500 ms'de durur.
7. Enkoderin pozitif yönü yanlışsa `e`; motor geri besleme yönü yanlışsa `d`
   kullanın. Sonra tekrar `z`, düşük güçte iki yön testi ve yalnız doğruladıktan
   sonra `v` gönderin.
8. `cfg save` ile ayarları kaydedin. Yeniden başlatmada AS5600 tek turlu olduğu
   için güvenlik gereği fiziksel merkezde `z` yine zorunludur.
9. Önce düşük `kp`, düşük `cp` ile `c on` deneyin. Kuvvet merkezden uzağa
   gidiyorsa hemen `s`; otomatik modu tekrar açmadan `d/e` ve mekaniği düzeltin.

Kalibrasyonsuz merkezleme, hedef konum veya seri FFB başlamaz. `a` eski sürümün
aksine yönü/sıfırı bozmaz ve motoru otomatik çalıştırmaz; yalnız güvenli varsayılan
kazanç/limitleri RAM'e yükler.

## Seri komutlar

| Komut | İşlev |
|---|---|
| `s` | Anında yazılımsal stop; fiziksel E-stop değildir |
| `r [1-50]`, `l [1-50]` | Manuel yön testi, varsayılan %10, 500 ms timeout |
| `z` | Mevcut konumu merkez kabul et; otomatik mod kilidini açan ilk adım |
| `d` | Motor yön işaretini tersle; yön onayını iptal eder |
| `e` | Enkoder yön işaretini tersle; merkez ve yön onayını iptal eder |
| `v` | Kullanıcının düşük güç motor/enkoder yön testini onaylaması |
| `c on`, `c off` | PD merkezleme aç/kapat |
| `f <-100..100>` | Seri kuvvet/PWM talebi; 100 ms watchdog, HID PID değildir |
| `g <-90..90>` | Sınırlı hedef konum testi |
| `m 360/900/1080` | Dönüş aralığı ve sanal son durak |
| `kp <0..2>` | Merkezleme Kp |
| `kd <0..0.2>` | Filtreli hız Kd |
| `rt <5..300>` | Çıkış slew-rate, yüzde/saniye |
| `mx <1..50>` | Merkezi genel PWM güvenlik limiti; varsayılan %30 |
| `cp <1..50>` | Merkezleme PWM limiti; `mx` ile sınırlandırılır |
| `ef <1..50>` | Endstop PWM limiti; `mx` ile sınırlandırılır |
| `a` | Güvenli varsayılan kazanç/limitleri yükle; motoru açmaz |
| `cfg show` | Çalışan ayarları göster |
| `cfg save` | Motor kapalıyken version+CRC kaydı oluştur |
| `cfg load` | Kaydı yükle; yeniden `z` ister |
| `cfg reset` | Güvenli varsayılanları yalnız RAM'e al |
| `t on/off` | 250 ms telemetriyi aç/kapat |
| `x` | Nedeni giderildikten sonra güvenlik kilidini temizle |
| `h` | Yardım |

Örnek telemetri:

```text
TEL angle=1.23 vel=-4.5 raw=1722 hid=90 mode=OFF pwm=0 req=0.0 cal=OK sensor_us=180/245 samples/errors/reconnect=500/0/0 missed/late_us=0/32
```

`sensor_us` son/maksimum I2C okuma süresidir. `missed/late_us`, 2 ms aday
kontrol periyodunun kaçırılan çevrimlerini ve en büyük gecikmesini gösterir.
Firmware 500 Hz'e zamanlanmıştır; gerçek donanımın bu hızı sürdürdüğü ancak bu
ölçümler uzun testte sıfır/uygun kaldığında söylenebilir.

Başlıca hata metinleri:

- `AS5600 I2C haberlesmesi basarisiz`: kablo/adres/bus timeout
- `miknayt algilamiyor/cok zayif/cok guclu`: AS5600 MD/ML/MH durumu
- `... 'z' gerekli`: çok-turlu konum güvenilir değil
- `... 'v' onayi gerekli`: motor/enkoder geri besleme yönü doğrulanmadı
- `FFB watchdog 100 ms`: yeni `f` komutu gelmedi, motor kapandı
- `30 saniye guvenli test oturumu`: akım/sıcaklık geri beslemesi olmadığı için
  sürekli motor testi bilinçli olarak sonlandırıldı
- `Guvenlik kilidi aktif`: E-stop veya 50 ms ana-döngü gözetimi tetiklendi

## Kontrol ve güvenlik akışı

```text
Seri test talebi / gelecekte oyun efekti
        -> istek işleme
        -> kalibrasyon + yön + sensör kontrolü
        -> sanal endstop
        -> slew-rate + merkezi PWM limiti
        -> yön değişim dead-time
        -> HAL (tek yüzde->duty dönüşümü)
        -> BTS7960
```

Merkezleme `output = -(Kp * angle + Kd * filteredVelocity)` kullanır. Merkezde
deadband ve smoothstep soft-zone vardır. Eski sabit minimum %22 yükseltmesi
kaldırıldı; ölçülmemiş sürtünme kompanzasyonu uygulanmaz. Bu nedenle çok düşük
çıkışta motor hareket etmeyebilir; bu, ani %22 sıçramadan daha güvenli varsayımdır.

Sanal endstop son 20 derecede yumuşak artar, `ef` ve `mx` ile sınırlıdır ve fiziksel
sınırdan içeri yönlü kuvvet uygular. Fiziksel mekanik stopun yerini tutmaz.

## Testler

```bash
pio test -e native
pio run -e f401rct6
pio run -e f401rct6_uart
```

Host testleri şu konuları kapsar: yüzde→duty için 0/1/10/22/30/50/75/100,
iki motor yönü ve stop, merkezi limit, 4095↔0 unwrap, pozitif/negatif ve çok
turlu birikim, kopuş sonrası güven kaybı, dead-time, wrap-safe timeout/FFB
watchdog temeli, PD işareti, endstop yönü/limiti, slew-rate, yön işareti,
CRC/bozuk ayar ve HID eksen ölçekleme.

## Henüz uygulanmayanlar

- Gerçek USB HID direksiyon transport/descriptor'ı
- CDC + joystick composite USB cihazı
- HID PID descriptor, output report handler ve oyun effect pool'u
- Constant/Spring/Damper/Friction/Periodic/Ramp oyun efektleri
- Linux `hidraw/evdev` ve Windows oyun içi doğrulaması
- Fiziksel pedal, vites, düğme ve akım sensörü sürücüleri
- Donanım üzerinde PWM/dead-time, USB disconnect, watchdog ve uzun süreli 500 Hz
  zamanlama doğrulaması

Gerçek HID PID çalışmasına, mevcut motor güvenlik katmanını değiştirmeden ayrı
bir USB Device modülü ve host-side descriptor/report testleriyle devam
edilmelidir.
