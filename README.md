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
- `sdkconfig.defaults` -- domyślna konfiguracja projektu (docelowy mikrokontroler, dane sieci WiFi, adres i port agenta micro-ROS)
- `partitions.csv` -- niestandardowa tabela partycji pamięci flash
- `main/CMakeLists.txt` -- plik konfiguracyjny komponentu głównego
- `main/idf_component.yml` -- deklaracja zależności projektu, w tym komponentu `micro_ros_espidf_component`
- `main/robot_main.c` -- kod źródłowy realizujący obsługę odbiornika RC (protokół CRSF), kinematykę odwrotną napędu mecanum, sterowanie silnikami (PWM) oraz integrację z systemem micro-ROS

### Budowanie projektu

```bash
cd software
idf.py set-target esp32p4
idf.py menuconfig   # konfiguracja WiFi oraz adresu agenta micro-ROS
idf.py build flash monitor
```
