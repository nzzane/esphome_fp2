# Flashing ESPHome on the Aqara FP2

Useful background: the official ESPHome guide on connecting to devices,
https://esphome.io/guides/physical_device_connection/, and the protocol notes in
https://github.com/hansihe/AqaraPresenceSensorFP2ReverseEngineering.

## 1. Update in the Aqara app first

The radar chip has its own firmware, pushed by the stock ESP firmware. ESPHome
cannot update it, so OTA-update the FP2 in the Aqara app before flashing. Do not
factory-reset afterwards; it is not needed.

## 2. Open the case

Remove the 4 rubber plugs on the back and the 4 screws, then pull the halves
apart. The rear half carries the USB-C board, which unplugs from an 8-pin header.

![back_screws.jpg](images/back_screws.jpg)

The board can stay in the front housing; all connection points are reachable.

## 3. Connect

![flashing_connections.jpg](images/flashing_connections.jpg)

| Signal | Where | Connect to |
|---|---|---|
| ESP TX | `TP8` (round pad bottom right, next to the button) | adapter **RX** |
| ESP RX | `TP9` (next to `TP8`) | adapter **TX** |
| GPIO0 | `TP28` (pad up-left of the ESP module) | GND while entering the bootloader |
| GND | pad next to the button, or header `GND` | adapter GND |
| EN (reset) | castellation on the right edge of the ESP module — optional | RTS, or a GND pulse |

Adapter on 3.3 V logic. Leave the adapter's VCC/5V unconnected.

**Power the board from its own USB-C board** (plug it back onto the header, mind
the orientation) with a normal charger. The radar's inrush browns out a supply
fed through the adapter's 5 V pin (`Brownout detector was triggered`,
`flash read err, 1000`, reset loops).

Test pads take pogo pins well; `EN` is fiddly and not needed — power-cycling with
`GPIO0` held low does the same.

![wires_connected.jpg](images/wires_connected.jpg)

![wires_and_usb_connected.jpg](images/wires_and_usb_connected.jpg)

## 4. Optional: record the stock boot log

```bash
stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
timeout 45 cat /dev/ttyUSB0 > stock-bootlog.raw &   # then power on
```

Shows the ESP app version, partition table and the `aqara>` console. The
`Base MAC address from BLK0 of EFUSE CRC error` line is normal on every FP2.
Console commands worth knowing: `help`, `version`, `nvs_list nvs`,
`nvs_namespace user` + `nvs_get mcu_ver blob` (radar firmware version, `63` = 99).

## 5. Enter the bootloader

Hold `GPIO0` low, then reset (unplug/replug USB, pulse `EN`, or type `restart`
on the stock console). The boot banner must say

```
boot:0x3 (DOWNLOAD_BOOT(UART0/UART1/SDIO_REI_REO_V2))
```

`boot:0x13 (SPI_FAST_FLASH_BOOT)` means `GPIO0` was not low — re-seat the probe.
Release `GPIO0` once `esptool` reports `Stub flasher running`.

## 6. Back up

```bash
pip install esptool
esptool -p /dev/ttyUSB0 --before no-reset --after no-reset flash-id
espefuse -p /dev/ttyUSB0 --before no-reset summary > efuse-summary.txt
esptool -p /dev/ttyUSB0 --before no-reset --after no-reset \
        read-flash 0x0 0x1000000 aqara_fp2_<serial-or-mac>.bin
sha256sum aqara_fp2_<serial-or-mac>.bin
```

`--before no-reset` because nothing drives DTR/RTS; `--after no-reset` keeps the
chip in the bootloader for the next step. Stay at 115200 baud unless your
adapter is known to handle the switch (a clone CP2102 accepted `-b 460800` and
then went silent). 16 MB takes ~25 min; only the first ~8.6 MB (through the
`fctry` partition at `0x833000`) is populated.

The `fctry` partition holds the per-unit calibration and HomeKit keys; there is
no flash encryption or secure boot, so the dump is a complete restore image.

## 7. Flash

Mandatory `esp32:` block — single-core chip with a bad eFuse MAC CRC:

```yaml
esp32:
  board: esp32-solo1
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_FREERTOS_UNICORE: "y"
      CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE: "y"
    advanced:
      ignore_efuse_mac_crc: true
      ignore_efuse_custom_mac: true
```

Do not add `psram:` (boot loop). Use `mounting_position: wall` — radar firmware
99 never starts tracking in the corner modes. Start from
[`example_config.yaml`](example_config.yaml), then:

```bash
esphome compile fp2.yaml
esptool -p /dev/ttyUSB0 --before no-reset --after no-reset \
        write-flash 0x0 .esphome/build/fp2/build/firmware.factory.bin
```

Lift the `GPIO0` probe off the pad, power-cycle, and watch the log at 115200.
A good boot shows `Heartbeat received. Starting initialization sequence...`
followed by Wi-Fi connecting. Everything after this is OTA.

## Restore stock

```bash
esptool -p /dev/ttyUSB0 --before no-reset --after hard-reset \
        write-flash 0x0 aqara_fp2_<serial-or-mac>.bin
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Brownout d…` / `flash read err, 1000` / reset loop | Power from the USB-C board and a real charger. |
| `boot:0x13` when you wanted download mode | `GPIO0` probe not low; re-seat. |
| esptool `Serial data stream stopped` | Chip is running stock (console echo). Enter the bootloader. |
| esptool `No more data to read` after `Changing baud rate` | Adapter can't follow the change. Power-cycle, use `-b 115200`. |
| esptool `No serial data received` | ESP TX probe (`TP8`) not making contact, or the adapter dropped off USB. |
| ESPHome boot-loops with `quad_psram: Not a valid or known package id` | Remove the `psram:` block. |
| Radar heartbeats, never any targets/presence | `mounting_position` is a corner mode; use `wall`. |
| A previously paired unit shows no targets on the bench | The radar kept the app's exclude map; draw a new one (or reset it) from the card. |

## Reassemble

Remove the probes, plug the USB-C board back on, reverse the disassembly.
