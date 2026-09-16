# Aqara FP2 ESPHome & Home Assistant Integration

> **⚠️ WARNING: EXPERIMENTAL SOFTWARE ⚠️**
> This project is in early development and **not well tested**. Use at your own risk!
> Features may be incomplete, unstable, or change without notice.
> Please report issues and contribute improvements if you encounter problems.

Custom ESPHome components and Home Assistant visualization card for the Aqara FP2 Presence Sensor.

![Card screenshot](images/card_screenshot.png)

See [FLASHING.md](FLASHING.md) for flashing instructions.

## Overview

This project provides two main components for working with the Aqara FP2 mmWave presence sensor:

1. **ESPHome Components** - Custom components for flashing and controlling the FP2 hardware directly with ESPHome
2. **Home Assistant Card** - Interactive visualization card for viewing real-time presence data, zones, and radar tracking

## Features

- Direct UART communication with FP2 radar sensor
- Real-time target tracking with position and velocity
- Customizable detection zones with individual sensitivity settings
- Interference grid and entry/exit zone configuration
- Visual representation of sensor coverage area
- Zone occupancy and motion detection
- Interactive web card with zone editing (zones, interference, exclude and entry/exit areas)
- Radar settings as Home Assistant select/switch entities
- Ambient light sensor (OPT3001) and RGB LED

---

## Part 1: ESPHome Components

The ESPHome components allow you to flash custom firmware to the Aqara FP2 and integrate it directly with Home Assistant via the ESPHome API.

### Components

- **`aqara_fp2`** - Main component for FP2 radar sensor control and data collection
- **`aqara_fp2_accel`** - Accelerometer interface for mounting position detection

### Installation

For remote installation (once published):

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/hansihe/esphome_fp2
      ref: main
    components: [aqara_fp2, aqara_fp2_accel]
```

### Configuration Example

See [example_config.yaml](example_config.yaml) for a complete working configuration.

### Options

`aqara_fp2`:

| Option | Default | Description |
|--------|---------|-------------|
| `mounting_position` | `wall` | `wall`, `left_corner`, `right_corner`. Radar firmware 99 only tracks in `wall` mode. |
| `proximity` | `medium` | `far` / `medium` / `close` |
| `detection_direction` | `default` | `default` / `left_right` |
| `ai_person_detection` | `true` | Ignore robot vacuums / pets |
| `people_counting` | `true` | Enable people count reports |
| `fall_detection` | `false` | With `fall_detection_sensitivity` |
| `derive_presence` | `true` | Derive presence/motion/zone occupancy from targets when the radar sends no events (`absence_timeout`: 30s) |
| `presence_event`, `people_count` | – | Text / numeric sensors |
| `zones[].presence_sensitivity` | `medium` | Per-zone sensitivity |
| `zones[].event` | – | Text sensor: `enter` / `move` / `exit` |
| `*_select`, `*_switch` | – | HA entities for each setting, e.g. `mounting_position_select`, `sensitivity_select`, `ai_person_detection_switch` (see example) |
| `sleep` | – | Experimental sleep monitoring (raw values, changes radar mode) |

`aqara_fp2_accel`: `illuminance` exposes the on-board OPT3001 light sensor.

Zones can be left without a `grid` and drawn from the card; grids, interference,
exclude and entry/exit maps set from the card are stored on the device
(`set_zone` / `set_map` api actions, see the example config).

### Grid Configuration

Grids are the radar's 20×16 map (8 m × 10 m, 0.5 m cells) in wall mode; corner modes use the 14×14 view (a 14-row grid in YAML is placed at columns 2-15):
- `.` = Active detection cell
- `X` = Zone coverage
- Grids available: `interference_grid`, `exit_grid`, `edge_label_grid`

### Flashing Instructions

See [FLASHING.md](FLASHING.md) for flashing instructions.

---

## Part 2: Home Assistant Card

An interactive visualization card for viewing FP2 sensor data in Home Assistant.

### Installation via HACS

This is the primary and recommended installation method.

1. **Add Custom Repository** (if not yet published to HACS default):
   - Open HACS in Home Assistant
   - Go to "Frontend"
   - Click the three dots (⋮) in the top right
   - Select "Custom repositories"
   - Add repository URL: `https://github.com/hansihe/esphome_fp2`
   - Category: `Dashboard`

2. **Install the Card**:
   - Search for "FP2" or "Aqara FP2 Presence Sensor Card"
   - Click "Download"
   - Restart Home Assistant

3. **Add to Dashboard**:
   ```yaml
   type: custom:aqara-fp2-card
   entity_prefix: sensor.fp2_living_room
   title: Living Room FP2
   ```

### Card Features

- **Real-time target tracking**: Shows detected people/objects with trails and velocity arrows
- **Zone visualization**: Color-coded zones with sensitivity and occupancy status
- **Grid overlays**: Exclude area, interference sources, entry/exit zones, field of view
- **Zone editing**: Pencil / *Edit* opens an editor - drag rectangles, pick sensitivity, *Save* stores on the device, *Reset* reverts to YAML
- **Per-zone sensitivity**: Change it from the zone list
- **Sensor position marker**: Shows FP2 mounting location

### Card Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `entity_prefix` | string | **required** | Entity prefix (e.g., `sensor.fp2`) |
| `device` | string | auto | ESPHome device name, only if it differs from the entity prefix (e.g. `fp2-test2` vs `sensor.fp2_test_2`) |
| `title` | string | "Aqara FP2 Presence Sensor" | Card title |
| `show_grid` | boolean | `true` | Show grid lines |
| `show_fov` | boolean | `true` | Shade the radar's field of view |
| `show_trails` | boolean | `true` | Draw each target's recent path |
| `show_velocity` | boolean | `true` | Draw a velocity arrow on each target |
| `show_axes` | boolean | `true` | Metre axes |
| `view` | list | from device | Crop of the 16×20 grid as `[col, row, cols, rows]`, e.g. `[4, 0, 8, 8]` |
| `auto_crop` | boolean | `false` | Crop to the drawn zones/maps plus one cell |
| `cell_size` | number | `0` | Cell size in px (`0` fits the card width, limited by `max_height`) |
| `max_height` | number | `520` | Maximum canvas height in px |
| `debug` | boolean | `false` | Console logging |

---

## Project Structure

```
esphome_fp2/
├── components/
│   ├── aqara_fp2/           # Main FP2 component
│   │   ├── __init__.py
│   │   ├── fp2_component.cpp
│   │   ├── fp2_component.h
│   │   ├── binary_sensor.py
│   │   └── text_sensor.py
│   └── aqara_fp2_accel/     # Accelerometer component
│       ├── __init__.py
│       ├── aqara_fp2_accel.cpp
│       └── aqara_fp2_accel.h
├── card.js                   # Home Assistant visualization card
├── hacs.json                 # HACS integration metadata
├── example_config.yaml       # Example ESPHome configuration
└── README.md                 # This file
```

---

## Requirements

### ESPHome Components
- ESPHome 2025.x or later
- ESP32 (ESP32-SOLO-1 in stock FP2 hardware)
- ESP-IDF framework
- Radar firmware 99 (update in the Aqara app before flashing)

### Home Assistant Card
- Home Assistant 2024.x or later
- HACS (recommended) or manual installation
- ESPHome integration configured with FP2 device

---

## Contributing

This is an experimental project and contributions are welcome!

Documentation around usage, improvements, or general feedback is welcome. I have made this mostly for myself, so I haven't put a massive amount of effort into making it easy to use.

See https://github.com/hansihe/AqaraPresenceSensorFP2ReverseEngineering for more technical details.

---

## Disclaimer

This project is not affiliated with or endorsed by Aqara. Use at your own risk. The authors are not responsible for any damage to your hardware or any issues arising from the use of this software.
