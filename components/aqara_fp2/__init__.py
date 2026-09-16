import json

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, sensor, switch, select, uart
from esphome.components import text_sensor as text_sensor_
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_DEVICE_ID,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    CONF_NAME,
    CONF_SECOND,
    CONF_MOTION,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_OCCUPANCY,
    DEVICE_CLASS_MOTION,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    ICON_MOTION_SENSOR,
)
from esphome.core import CORE
from esphome.util import Registry

from ..aqara_fp2_accel import AqaraFP2Accel

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor", "switch", "select", "json"]

aqara_fp2_ns = cg.esphome_ns.namespace("aqara_fp2")
FP2Component = aqara_fp2_ns.class_("FP2Component", cg.Component, uart.UARTDevice)
FP2SettingSelect = aqara_fp2_ns.class_("FP2SettingSelect", select.Select, cg.Component)
FP2SettingSwitch = aqara_fp2_ns.class_("FP2SettingSwitch", switch.Switch, cg.Component)
Setting = aqara_fp2_ns.enum("Setting", is_class=True)

# HA-controllable settings: key -> (Setting enum, options)
SETTING_SELECTS = {
    "mounting_position_select": ("MOUNTING_POSITION", ["wall", "left_corner", "right_corner"], "mdi:wall"),
    "proximity_select": ("PROXIMITY", ["far", "medium", "close"], "mdi:map-marker-distance"),
    "detection_direction_select": ("DETECTION_DIRECTION", ["default", "left_right"], "mdi:arrow-left-right"),
    "sensitivity_select": ("SENSITIVITY", ["low", "medium", "high"], "mdi:tune"),
    "fall_sensitivity_select": ("FALL_SENSITIVITY", ["low", "medium", "high"], "mdi:tune"),
}
SETTING_SWITCHES = {
    "left_right_reverse_switch": ("LEFT_RIGHT_REVERSE", "mdi:swap-horizontal"),
    "ai_person_detection_switch": ("AI_PERSON_DETECTION", "mdi:robot-vacuum"),
    "people_counting_switch": ("PEOPLE_COUNTING", "mdi:account-group"),
    "fall_detection_switch": ("FALL_DETECTION", "mdi:human-handsdown"),
    "sleep_monitoring_switch": ("SLEEP_MONITORING", "mdi:sleep"),
}

FP2LocationSwitch = aqara_fp2_ns.class_("FP2LocationSwitch", switch.Switch, cg.Component)
FP2Zone = aqara_fp2_ns.class_("FP2Zone", cg.Component)

CONF_FP2_ID = "fp2_id"

CONF_MOUNTING_POSITION = "mounting_position"
CONF_LEFT_RIGHT_REVERSE = "left_right_reverse"
CONF_INTERFERENCE_GRID = "interference_grid"
CONF_EXIT_GRID = "exit_grid"
CONF_EDGE_GRID = "edge_grid"
CONF_ZONES = "zones"
CONF_GRID = "grid"
CONF_SENSITIVITY = "sensitivity"

# New Options
CONF_RADAR_RESET_PIN = "radar_reset_pin"
CONF_PRESENCE_SENSITIVITY = "presence_sensitivity"
CONF_FALL_DETECTION_SENSITIVITY = "fall_detection_sensitivity"
CONF_PEOPLE_COUNTING_REPORT_ENABLE = "people_counting_report_enable"
CONF_PEOPLE_NUMBER_ENABLE = "people_number_enable"
CONF_TARGET_TYPE_ENABLE = "target_type_enable"
CONF_DWELL_TIME_ENABLE = "dwell_time_enable"
CONF_WALKING_DISTANCE_ENABLE = "walking_distance_enable"
CONF_TARGET_TRACKING = "target_tracking"
CONF_LOCATION_REPORT_SWITCH = "location_report_switch"
CONF_RADAR_TEMPERATURE = "radar_temperature"
CONF_PRESENCE = "presence"
CONF_GLOBAL_ZONE = "global_zone"
CONF_RADAR_SOFTWARE_VERSION = "radar_software_version"

MOUNTING_POSITIONS = {
    "wall": 0x01,
    "left_corner": 0x02,
    "right_corner": 0x03,
}

SENSITIVITY_LEVELS = {
    "low": 1,
    "medium": 2,
    "high": 3,
}

PROXIMITY_LEVELS = {"far": 0, "medium": 1, "close": 2}
DETECTION_DIRECTIONS = {"default": 0, "left_right": 1}

CONF_PROXIMITY = "proximity"
CONF_DETECTION_DIRECTION = "detection_direction"
CONF_AI_PERSON_DETECTION = "ai_person_detection"
CONF_PEOPLE_COUNTING = "people_counting"
CONF_FALL_DETECTION = "fall_detection"
CONF_PRESENCE_EVENT = "presence_event"
CONF_PEOPLE_COUNT = "people_count"
CONF_SLEEP = "sleep"
CONF_EVENT = "event"

SLEEP_SCHEMA = cv.Schema(
    {
        # Raw values as written by the stock firmware; meanings not verified.
        cv.Optional("mount_position", default=1): cv.int_range(min=0, max=255),
        cv.Optional("bed_width", default=120): cv.int_range(min=0, max=65535),   # cm
        cv.Optional("bed_length", default=180): cv.int_range(min=0, max=65535),  # cm
        cv.Optional("presence"): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_OCCUPANCY, icon="mdi:bed"
        ),
        cv.Optional("state"): sensor.sensor_schema(
            icon="mdi:sleep", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional("in_out"): sensor.sensor_schema(
            icon="mdi:bed-outline", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional("event"): sensor.sensor_schema(
            icon="mdi:sleep", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional("data"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    }
)


def parse_ascii_grid(value):
    """
    Parses a 14x14 ASCII grid into a 40-byte (320-bit) protocol blob.
    An empty string yields an empty (disabled) zone that can be drawn later
    from the Home Assistant card.
    Protocol Grid: 20 rows x 16 cols.
    Active Area: Centered 14x14 (Rows 3-16, Cols 1-14).

    Chars: 'x', 'X' = Active. '.', ' ' = Inactive.
    """
    if not str(value).strip():
        return [0] * 40
    lines = value.strip().splitlines()
    # Filter out empty lines or comments if needed, but strict 14 lines is better for now
    lines = [li.strip() for li in lines if li.strip()]

    # 14 x 14 (corner view, placed at columns 2-15) or the full 20 x 16 grid
    if len(lines) == 14:
        width, offset_col = 14, 2
    elif len(lines) == 20:
        width, offset_col = 16, 0
    else:
        raise cv.Invalid(f"Grid must have 14 rows (14x14 corner view) or 20 rows (20x16 full grid), got {len(lines)}")

    for i, line in enumerate(lines):
        # Remove whitespace
        clean_line = line.replace(" ", "")
        if len(clean_line) != width:
            raise cv.Invalid(
                f"Row {i + 1} must have {width} characters (excluding spaces), got {len(clean_line)}: '{clean_line}'"
            )

    # Initialize 20x16 grid (320 bits -> 40 bytes)
    # 20 rows * 16 cols
    grid_data = bytearray(40)

    # Map 14x14 input to 20x16 output
    # Input Row 0 -> Output Row 3
    # Input Col 0 -> Output Col 1

    offset_row = 0

    for r in range(len(lines)):
        line = lines[r].replace(" ", "")
        out_r = r + offset_row

        # In the protocol:
        # Each row is 2 bytes (16 bits) Big Endian.
        # byte[2*r] is High Byte (Cols 0-7)
        # byte[2*r + 1] is Low Byte (Cols 8-15)
        # Bit 15 = Col 0 ... Bit 0 = Col 15

        row_val = 0

        for c in range(width):
            char = line[c]
            if char in ("x", "X"):
                out_c = c + offset_col
                # Set bit at out_c
                # Standard convention: MSB is index 0.
                # So Col 0 is 1 << 15
                bit_mask = 1 << (15 - out_c)
                row_val |= bit_mask

        # Write row_val to buffer (Big Endian)
        grid_data[out_r * 2] = (row_val >> 8) & 0xFF
        grid_data[out_r * 2 + 1] = row_val & 0xFF

    gd = list(grid_data)
    return gd


def grid_to_hex_string(grid_data):
    """Convert a 40-byte grid to a compact hex string for storage."""
    return "".join(f"{b:02x}" for b in grid_data)

ZONE_BASE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PRESENCE_SENSITIVITY, default="medium"): cv.enum(SENSITIVITY_LEVELS),
        cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_OCCUPANCY,
            filters=[{"settle": cv.TimePeriod(milliseconds=1000)}],
        ),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOTION,
            icon=ICON_MOTION_SENSOR,
        ),
    }
)

ZONE_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(FP2Zone),
            cv.Optional(CONF_GRID, default=""): parse_ascii_grid,
            cv.Optional("zone_map_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional(CONF_EVENT): text_sensor_.text_sensor_schema(icon="mdi:motion-sensor"),
        }
    ).extend(ZONE_BASE_SCHEMA)
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FP2Component),
            cv.Required("accel"): cv.use_id(AqaraFP2Accel),

            cv.Optional(CONF_RADAR_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_MOUNTING_POSITION, default="left_corner"): cv.enum(
                MOUNTING_POSITIONS
            ),

            cv.Optional(CONF_LEFT_RIGHT_REVERSE, default=False): cv.boolean,
            cv.Optional(CONF_PROXIMITY, default="medium"): cv.enum(PROXIMITY_LEVELS),
            cv.Optional(CONF_DETECTION_DIRECTION, default="default"): cv.enum(DETECTION_DIRECTIONS),
            cv.Optional(CONF_AI_PERSON_DETECTION, default=True): cv.boolean,
            cv.Optional(CONF_PEOPLE_COUNTING, default=True): cv.boolean,
            cv.Optional(CONF_FALL_DETECTION, default=False): cv.boolean,
            cv.Optional(CONF_FALL_DETECTION_SENSITIVITY, default="medium"): cv.enum(SENSITIVITY_LEVELS),
            cv.Optional(CONF_PRESENCE_EVENT): text_sensor_.text_sensor_schema(icon="mdi:motion-sensor"),
            cv.Optional(CONF_PEOPLE_COUNT): sensor.sensor_schema(
                icon="mdi:account-group", accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_SLEEP): SLEEP_SCHEMA,
            # Debug only: report a fixed orientation to the radar
            cv.Optional("debug_force_direction"): cv.int_range(min=0, max=8),
            cv.Optional("debug_replay_stock_init", default=False): cv.boolean,
            cv.Optional("debug_force_angle", default=45): cv.int_range(min=0, max=360),
            **{
                cv.Optional(k): select.select_schema(FP2SettingSelect, icon=icon, entity_category=ENTITY_CATEGORY_CONFIG)
                for k, (_, _, icon) in SETTING_SELECTS.items()
            },
            **{
                cv.Optional(k): switch.switch_schema(FP2SettingSwitch, icon=icon, entity_category=ENTITY_CATEGORY_CONFIG, default_restore_mode="DISABLED")
                for k, (_, icon) in SETTING_SWITCHES.items()
            },
            cv.Optional(CONF_INTERFERENCE_GRID): parse_ascii_grid,
            cv.Optional(CONF_EXIT_GRID): parse_ascii_grid,
            cv.Optional(CONF_EDGE_GRID): parse_ascii_grid,

            cv.Optional(CONF_TARGET_TRACKING): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional(CONF_LOCATION_REPORT_SWITCH): switch.switch_schema(
                FP2LocationSwitch, default_restore_mode="RESTORE_DEFAULT_ON"
            ),
            # Derive presence/motion/zone occupancy from the target stream when
            # the radar does not report them (FW 99 in wall mode).
            cv.Optional("derive_presence", default=True): cv.boolean,
            cv.Optional("absence_timeout", default="30s"): cv.positive_time_period_milliseconds,

            cv.Optional("edge_label_grid_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("entry_exit_grid_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("interference_grid_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("mounting_position_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),

            cv.Optional(CONF_GLOBAL_ZONE): ZONE_BASE_SCHEMA,
            cv.Optional(CONF_ZONES): cv.ensure_list(ZONE_SCHEMA),

            cv.Optional(CONF_RADAR_SOFTWARE_VERSION): text_sensor_.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RADAR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

SENSOR_MAP = {
    CONF_RADAR_TEMPERATURE: (sensor.new_sensor, "set_radar_temperature_sensor"),
    CONF_RADAR_SOFTWARE_VERSION: (text_sensor_.new_text_sensor, "set_radar_software_sensor"),
    CONF_LOCATION_REPORT_SWITCH: (switch.new_switch, "set_location_report_switch"),
    CONF_TARGET_TRACKING: (text_sensor_.new_text_sensor, "set_target_tracking_sensor"),
    CONF_PRESENCE_EVENT: (text_sensor_.new_text_sensor, "set_presence_event_sensor"),
    CONF_PEOPLE_COUNT: (sensor.new_sensor, "set_people_count_sensor"),

    # Text config sensors
    "edge_label_grid_sensor": (text_sensor_.new_text_sensor, "set_edge_label_grid_sensor"),
    "entry_exit_grid_sensor": (text_sensor_.new_text_sensor, "set_entry_exit_grid_sensor"),
    "interference_grid_sensor": (text_sensor_.new_text_sensor, "set_interference_grid_sensor"),
    "mounting_position_sensor": (text_sensor_.new_text_sensor, "set_mounting_position_sensor"),
}

ZONE_SENSOR_MAP = {
    CONF_PRESENCE: (binary_sensor.new_binary_sensor, "set_presence_sensor"),
    CONF_MOTION: (binary_sensor.new_binary_sensor, "set_motion_sensor"),

    # Text config sensors
    "zone_map_sensor": (text_sensor_.new_text_sensor, "set_map_sensor"),
    CONF_EVENT: (text_sensor_.new_text_sensor, "set_event_sensor"),
}

SLEEP_SENSOR_MAP = {
    "presence": (binary_sensor.new_binary_sensor, "set_sleep_presence_sensor"),
    "state": (sensor.new_sensor, "set_sleep_state_sensor"),
    "in_out": (sensor.new_sensor, "set_sleep_inout_sensor"),
    "event": (sensor.new_sensor, "set_sleep_event_sensor"),
    "data": (text_sensor_.new_text_sensor, "set_sleep_data_sensor"),
}

async def to_code(config):
    zones = []
    if CONF_ZONES in config:
        for i, zone_conf in enumerate(config[CONF_ZONES]):
            var = cg.new_Pvariable(
                zone_conf[CONF_ID],
                i + 1,
                zone_conf[CONF_GRID],
                zone_conf[CONF_PRESENCE_SENSITIVITY],
            )
            await cg.register_component(var, zone_conf)

            # Create sensors if provided
            for key, (new, funcName) in ZONE_SENSOR_MAP.items():
                if key in zone_conf:
                    sens = await new(zone_conf[key])
                    cg.add(getattr(var, funcName)(sens))

            zones.append(var)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RADAR_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RADAR_RESET_PIN])
        cg.add(var.set_radar_reset_pin(reset_pin))

    cg.add(var.set_mounting_position(config[CONF_MOUNTING_POSITION]))
    cg.add(var.set_left_right_reverse(config[CONF_LEFT_RIGHT_REVERSE]))
    cg.add(var.set_proximity(config[CONF_PROXIMITY]))
    cg.add(var.set_detection_direction(config[CONF_DETECTION_DIRECTION]))
    cg.add(var.set_ai_person_detection(config[CONF_AI_PERSON_DETECTION]))
    cg.add(var.set_people_counting(config[CONF_PEOPLE_COUNTING]))
    cg.add(var.set_fall_detection(config[CONF_FALL_DETECTION]))
    cg.add(var.set_fall_detection_sensitivity(config[CONF_FALL_DETECTION_SENSITIVITY]))

    for key, (setting, options, _) in SETTING_SELECTS.items():
        if key in config:
            sel = await select.new_select(config[key], options=options)
            await cg.register_component(sel, config[key])
            cg.add(sel.set_parent(var, getattr(Setting, setting)))
            cg.add(var.add_setting_select(sel))
    for key, (setting, _) in SETTING_SWITCHES.items():
        if key in config:
            sw = await switch.new_switch(config[key])
            await cg.register_component(sw, config[key])
            cg.add(sw.set_parent(var, getattr(Setting, setting)))
            cg.add(var.add_setting_switch(sw))

    cg.add(var.set_replay_stock_init(config["debug_replay_stock_init"]))
    cg.add(var.set_derive_presence(config["derive_presence"]))
    cg.add(var.set_absence_timeout(config["absence_timeout"].total_milliseconds))
    if "debug_force_direction" in config:
        cg.add(var.set_force_direction(config["debug_force_direction"], config["debug_force_angle"]))

    if CONF_SLEEP in config:
        sleep_conf = config[CONF_SLEEP]
        cg.add(var.set_sleep_enabled(True))
        cg.add(var.set_sleep_mount_position(sleep_conf["mount_position"]))
        cg.add(var.set_sleep_bed_size(sleep_conf["bed_width"], sleep_conf["bed_length"]))
        for key, (new, funcName) in SLEEP_SENSOR_MAP.items():
            if key in sleep_conf:
                sens = await new(sleep_conf[key])
                cg.add(getattr(var, funcName)(sens))

    if CONF_GLOBAL_ZONE in config:
        global_zone_conf = config[CONF_GLOBAL_ZONE]

        cg.add(var.set_presence_sensitivity(global_zone_conf[CONF_PRESENCE_SENSITIVITY]))

        for key, (new, funcName) in ZONE_SENSOR_MAP.items():
            if key in global_zone_conf and key in (CONF_PRESENCE, CONF_MOTION):
                sens = await new(global_zone_conf[key])
                cg.add(getattr(var, funcName)(sens))


    if CONF_INTERFERENCE_GRID in config:
        cg.add(var.set_interference_grid(config[CONF_INTERFERENCE_GRID]))

    if CONF_EXIT_GRID in config:
        cg.add(var.set_exit_grid(config[CONF_EXIT_GRID]))

    if CONF_EDGE_GRID in config:
        cg.add(var.set_edge_grid(config[CONF_EDGE_GRID]))

    cg.add(var.set_zones(zones))

    for key, (new, funcName) in SENSOR_MAP.items():
        if key in config:
            sens = await new(config[key])
            if key == CONF_LOCATION_REPORT_SWITCH:
                await cg.register_component(sens, config[key])
            cg.add(getattr(var, funcName)(sens))

    # Generate map config JSON data at compile time
    map_config_data = {
        "mounting_position": config[CONF_MOUNTING_POSITION],
        "left_right_reverse": config[CONF_LEFT_RIGHT_REVERSE],
    }

    # Add grids if present
    if CONF_INTERFERENCE_GRID in config:
        map_config_data["interference_grid"] = grid_to_hex_string(
            config[CONF_INTERFERENCE_GRID]
        )

    if CONF_EXIT_GRID in config:
        map_config_data["exit_grid"] = grid_to_hex_string(config[CONF_EXIT_GRID])

    if CONF_EDGE_GRID in config:
        map_config_data["edge_grid"] = grid_to_hex_string(config[CONF_EDGE_GRID])

    # Add zones
    if CONF_ZONES in config:
        zones_data = []
        for zone_conf in config[CONF_ZONES]:
            zone_data = {
                "sensitivity": zone_conf[CONF_PRESENCE_SENSITIVITY],
                "grid": grid_to_hex_string(zone_conf[CONF_GRID]),
            }
            zones_data.append(zone_data)
        map_config_data["zones"] = zones_data

    # Store as JSON string constant
    map_config_json = json.dumps(map_config_data, separators=(",", ":"))
    cg.add(var.set_map_config_json(map_config_json))

    accel = await cg.get_variable(config["accel"])
    cg.add(var.set_fp2_accel(accel))
