import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, binary_sensor
from esphome.const import CONF_ID, CONF_TEMPERATURE, CONF_HUMIDITY

CODEOWNERS = ["@esphome-user"]
DEPENDENCIES = ["sensor", "binary_sensor"]
MULTI_CONF = True

tm1628_sniffer_ns = cg.esphome_ns.namespace("tm1628_sniffer")
TM1628SnifferComponent = tm1628_sniffer_ns.class_("TM1628SnifferComponent", cg.Component)

CONF_STB_PIN = "stb_pin"
CONF_CLK_PIN = "clk_pin"
CONF_DIO_PIN = "dio_pin"
CONF_POWER = "power_sensor"
CONF_ION = "ion_sensor"
CONF_HEAT = "heat_sensor"
CONF_NO_WATER = "no_water_sensor"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(TM1628SnifferComponent),
    cv.Required(CONF_STB_PIN): cv.uint8_t,
    cv.Required(CONF_CLK_PIN): cv.uint8_t,
    cv.Required(CONF_DIO_PIN): cv.uint8_t,
    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=0,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=0,
        device_class="humidity",
        state_class="measurement",
    ),
    cv.Optional(CONF_ION): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_HEAT): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_POWER): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_NO_WATER): binary_sensor.binary_sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_stb_pin(config[CONF_STB_PIN]))
    cg.add(var.set_clk_pin(config[CONF_CLK_PIN]))
    cg.add(var.set_dio_pin(config[CONF_DIO_PIN]))
    
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temp_sensor(sens))
    
    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humi_sensor(sens))
    
    if CONF_ION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_ION])
        cg.add(var.set_ion_sensor(sens))
    
    if CONF_HEAT in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_HEAT])
        cg.add(var.set_heat_sensor(sens))
    
    if CONF_POWER in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))
    
    if CONF_NO_WATER in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_NO_WATER])
        cg.add(var.set_no_water_sensor(sens))
