import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
)

CONF_ILLUMINANCE = "illuminance"

CONF_UPDATE_INTERVAL = "update_interval"

aqara_fp2_accel_ns = cg.esphome_ns.namespace("aqara_fp2_accel")
AqaraFP2Accel = aqara_fp2_accel_ns.class_(
    "AqaraFP2Accel", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AqaraFP2Accel),
        cv.Optional(CONF_UPDATE_INTERVAL, default="100ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # ESPHome >= 2026.9 prunes unused ESP-IDF components; we use the
    # i2c_master driver directly, so pull it back in.
    try:
        from esphome.components.esp32 import include_builtin_idf_component
        include_builtin_idf_component("esp_driver_i2c")
    except ImportError:
        pass

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set update interval in milliseconds
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL].total_milliseconds))

    if CONF_ILLUMINANCE in config:
        sens = await sensor.new_sensor(config[CONF_ILLUMINANCE])
        cg.add(var.set_illuminance_sensor(sens))
