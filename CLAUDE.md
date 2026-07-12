# CoffeeScale

Arduino sketch for a coffee scale: HX711 load cell, SSD1306 OLED display,
rotary encoder + push button for input, running on an Arduino Nano
(atmega328 with old bootloader — CH340 USB-serial clone board).

## Build / compile

Uses `arduino-cli` (installed via Homebrew). Core and libraries are already
installed locally.

```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old CoffeeScale.ino
```

To upload (adjust port as needed, check with `arduino-cli board list`):

```
arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn arduino:avr:nano:cpu=atmega328old CoffeeScale.ino
```

If upload fails with a sync/timeout error, try `cpu=atmega328` (new bootloader)
instead of `atmega328old`.

## Libraries

Installed via `arduino-cli lib install`:

- `ssd1306` (Alexey Dynda) — OLED display driver. Note: API-incompatible with
  Adafruit_SSD1306, must match this specific library.
- `HX711` (Bogdan Necula) — load cell amplifier
- `Button` (Michael Adams)
- `RotaryEncoder` (Matthias Hertel)
- `TimerOne` (Paul Stoffregen)
- `EEPROM` is built into the AVR core, no separate install needed

## Git / GitHub

This repo pushes to a personal GitHub account (`cdaniel42`), separate from a
work account also configured on this machine. The remote uses a dedicated SSH
host alias to force the right key:

```
origin  git@github-personal:cdaniel42/CoffeeScale.git
```

(`~/.ssh/config` defines `github-personal` → `id_ed25519_personal`, and
`github-work` → `id_ed25519`, the work key.) Local repo `user.name`/`user.email`
are set to the personal identity (`c.daniel42@gmail.com`) so commit authorship
doesn't need fixing after the fact.
