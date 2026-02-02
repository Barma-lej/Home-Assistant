#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>

namespace esphome {
namespace tm1628_sniffer {

class TM1628SnifferComponent : public Component {
public:
  void set_stb_pin(uint8_t pin) { stb_pin_ = pin; }
  void set_clk_pin(uint8_t pin) { clk_pin_ = pin; }
  void set_dio_pin(uint8_t pin) { dio_pin_ = pin; }

  void set_temp_sensor(sensor::Sensor *s) { temp_sensor_ = s; }
  void set_humi_sensor(sensor::Sensor *s) { humi_sensor_ = s; }

  void set_heat_sensor(binary_sensor::BinarySensor *s) { heat_sensor_ = s; }
  void set_ion_sensor(binary_sensor::BinarySensor *s) { ion_sensor_ = s; }
  void set_power_sensor(binary_sensor::BinarySensor *s) { power_sensor_ = s; }
  void set_no_water_sensor(binary_sensor::BinarySensor *s) { no_water_sensor_ = s; }

  void setup() override;
  void loop() override;

private:
  uint8_t stb_pin_;
  uint8_t clk_pin_;
  uint8_t dio_pin_;
  uint32_t last_active_time_{0};
  uint32_t last_no_water_time_{0};
  int no_water_counter_{0};

  // Debounce для POWER
  uint8_t power_on_counter_;
  uint8_t power_off_counter_;

  // Debounce для HEAT
  uint8_t heat_on_counter_;
  uint8_t heat_off_counter_;

  // Debounce для ION
  uint8_t ion_on_counter_;
  uint8_t ion_off_counter_;

  sensor::Sensor *temp_sensor_{nullptr};
  sensor::Sensor *humi_sensor_{nullptr};

  binary_sensor::BinarySensor *heat_sensor_{nullptr};
  binary_sensor::BinarySensor *ion_sensor_{nullptr};
  binary_sensor::BinarySensor *power_sensor_{nullptr};
  binary_sensor::BinarySensor *no_water_sensor_{nullptr};

  std::vector<uint8_t> current_data_;

  uint8_t read_byte();
  void process_transaction();
  void decode_stream(const std::vector<uint8_t> &data);
  int decode_pair(uint8_t b1, uint8_t b2);

  // Кэш для фильтрации дублей
  float last_temp_{-999.0};
  float last_humi_{-999.0};
  bool last_heat_{false};
  bool last_ion_{false};
  bool last_power_{false};

  uint32_t last_publish_time_{0};

};

}
}
