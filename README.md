# CARLOS

Repozytorium projektowe robota mobilnego z napędem mecanum, zrealizowanego w ramach pracy dyplomowej. Zawiera schematy elektroniczne i modele 3D (Altium Designer), pliki produkcyjne PCB (Gerber) oraz projekt oprogramowania wbudowanego (ESP-IDF).

## Struktura repozytorium

```
CARLOS/
├── design/     # schematy Altium Designer, modele 3D oraz dokumentacja PDF 3D
├── gerber/     # pliki produkcyjne PCB (Gerber)
└── software/   # oprogramowanie wbudowane (ESP-IDF)
```

## `design/` -- Altium Designer

Folder zawiera schematy elektroniczne, pliki 3D płytek oraz ich podgląd w formacie PDF 3D.

### Główny obwód drukowany

- `schemat.SchDoc` -- Schemat główny, integrujący wszystkie moduły
  - `ARGB.SchDoc` -- Sterowanie diodami adresowalnymi ARGB
  - `MCU_unit.SchDoc` -- Jednostka mikrokontrolera Waveshare ESP32-P4-Module
  - `connectors.SchDoc` -- Złącza zewnętrzne oraz zworki konfiguracyjne
  - `audio.SchDoc` -- Obsługa buzzera, głośnika oraz mikrofonu
  - `IMU.SchDoc` -- Konfiguracja żyroskopu oraz magnetometru
  - `Motor_SCH.SchDoc` -- Blok łączący funkcjonalność dla 4 silników szczotkowych
    - `DRV8837CDSGR.SchDoc` -- Konfiguracja mostka H -- DRV8837CDSGR
  - `microsd_card.SchDoc` -- Interfejs karty microSD
  - `USB_HS.SchDoc` -- Interfejs USB High-Speed
  - `CH342F.SchDoc` -- Konwerter USB-UART CH342F
  - `Power_block.SchDoc` -- LDO dla 5V oraz przetwornica BUCK dla 3,3V

### Doker baterii

- `main_IP5353.SchDoc` -- Schemat główny dokera

### Pliki 3D

- Modele 3D płytek PCB
- Dokumentacja PDF 3D -- interaktywny podgląd płytek, możliwy do otwarcia w Adobe Acrobat Reader

## `gerber/` -- Pliki wykonawcze

Zweryfikowane zamówienie w [JLCPCB](https://jlcpcb.com).

- `carlos_v0.1` -- Pliki gerber dla głównego PCB
- `slim_v0.1` -- Pliki gerber dla dokera baterii

## `software/` -- Oprogramowanie ESP-IDF

- `CMakeLists.txt` -- główny plik konfiguracyjny projektu
- `sdkconfig.defaults` -- domyślna konfiguracja projektu (docelowy mikrokontroler, dane sieci WiFi, adres i port agenta micro-ROS, przypisanie pinów silników)
- `partitions.csv` -- niestandardowa tabela partycji pamięci flash
- `main/CMakeLists.txt` -- plik konfiguracyjny komponentu głównego
- `main/Kconfig.projbuild` -- definicja opcji konfiguracyjnych projektu dostępnych w `menuconfig` (piny GPIO oraz grupy MCPWM dla czterech silników)
- `main/idf_component.yml` -- deklaracja zależności projektu, w tym komponentu `micro_ros_espidf_component`
- `main/robot_main.c` -- kod źródłowy realizujący obsługę odbiornika RC (protokół CRSF), kinematykę odwrotną napędu mecanum, sterowanie silnikami (PWM) oraz integrację z systemem micro-ROS

### Konfiguracja pinów silników

Przypisanie pinów nie jest zapisane na stałe w kodzie -- definiuje je `main/Kconfig.projbuild`, a wartości trafiają do `sdkconfig` jako makra `CONFIG_WHEEL_*`. Zmiana pinów nie wymaga edycji kodu źródłowego:

```bash
idf.py menuconfig   # menu: CARLOS - piny silników
```

| Koło | IN1 | IN2 | Grupa MCPWM |
|------|-----|-----|-------------|
| Przednie lewe (FL) | GPIO 5  | GPIO 20 | 0 |
| Przednie prawe (FR) | GPIO 32 | GPIO 33 | 0 |
| Tylne lewe (BL) | GPIO 3  | GPIO 4  | 1 |
| Tylne prawe (BR) | GPIO 1  | GPIO 2  | 1 |

> Zmiany w `sdkconfig.defaults` są uwzględniane dopiero po usunięciu pliku `sdkconfig` lub wykonaniu `idf.py fullclean`.

### Wymagania

- ESP-IDF v5.5 z toolchainem dla ESP32-P4
- Linux lub WSL (Ubuntu)

Pakiety systemowe (kompilator hosta, potrzebny do zbudowania narzędzi micro-ROS):

```bash
sudo apt update
sudo apt install build-essential git
```

Pakiety Pythona dla micro-ROS (colcon i zależności), instalowane w środowisku Pythona ESP-IDF:

```bash
. $IDF_PATH/export.sh
python -m pip install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4
```

> Wersja `empy==3.3.4` jest wymagana -- nowsze wersje (4.x) nie są kompatybilne z budowaniem micro-ROS.

Weryfikacja instalacji:

```bash
python -c "import catkin_pkg, lark, em, colcon_core; print('OK')"
```

### Budowanie projektu

```bash
cd software
idf.py set-target esp32p4
idf.py menuconfig   # konfiguracja WiFi, adresu agenta micro-ROS oraz pinów silników
idf.py build flash monitor
```

Pierwsze budowanie trwa kilka minut -- komponent micro-ROS jest pobierany automatycznie do `managed_components/` i kompilowany jako biblioteka `libmicroros.a`. Kolejne buildy korzystają z gotowej biblioteki.

> Główny `CMakeLists.txt` zawiera poprawkę zmiennej `PATH`, która usuwa z niej katalogi z asemblerami toolchainów ESP. Bez niej systemowy `gcc` używany przez micro-ROS wybierałby niewłaściwy asembler (`as: unrecognized option '--64'`).

W razie ponownego budowania micro-ROS od zera:

```bash
rm -rf build managed_components/micro-ros__micro_ros_espidf_component/micro_ros_dev managed_components/micro-ros__micro_ros_espidf_component/micro_ros_src
idf.py build
```
