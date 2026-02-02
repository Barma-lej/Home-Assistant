#include "tm1628_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h"
#include "esp_timer.h"

namespace esphome {
namespace tm1628_sniffer {

static const char *const TAG = "tm1628_sniffer";

inline void delay_micros(uint32_t us) {
    uint32_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {}
}

void TM1628SnifferComponent::setup() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << stb_pin_) | (1ULL << clk_pin_) | (1ULL << dio_pin_);
    gpio_config(&io_conf);

    last_active_time_ = millis();
    last_no_water_time_ = 0;
    no_water_counter_ = 0;
    power_on_counter_ = 0;
    power_off_counter_ = 0;
    heat_on_counter_ = 0;
    heat_off_counter_ = 0;
    ion_on_counter_ = 0;
    ion_off_counter_ = 0;

    // ===== ИНИЦИАЛИЗАЦИЯ СЕНСОРОВ С ДЕФОЛТНЫМИ ЗНАЧЕНИЯМИ =====
    // Binary sensors (TRUE = вкл, FALSE = выкл)
    if (power_sensor_ != nullptr) {
        power_sensor_->publish_state(false);
    }
    if (heat_sensor_ != nullptr) {
        heat_sensor_->publish_state(false);
    }
    if (ion_sensor_ != nullptr) {
        ion_sensor_->publish_state(false);
    }
    if (no_water_sensor_ != nullptr) {
        no_water_sensor_->publish_state(false);
    }
    // Temperature sensor (в градусах Цельсия)
    if (temp_sensor_ != nullptr) {
        temp_sensor_->publish_state(20.0);
    }
    // Humidity sensor (в процентах)
    if (humi_sensor_ != nullptr) {
        humi_sensor_->publish_state(20.0);
    }
}

    void TM1628SnifferComponent::loop() {
    if (!gpio_get_level((gpio_num_t)stb_pin_)) {
        process_transaction();
    }

    // Watchdog: если 60 секунд не было паттерна работы -> POWER OFF
    if (millis() - last_active_time_ > 60000) {
        if (power_sensor_ != nullptr && power_sensor_->state) {
            power_sensor_->publish_state(false);
        }
    }

    // Watchdog NO_WATER: если 3 секунды не было паттерна NO_WATER -> сбросить
    if (millis() - last_no_water_time_ > 3000) {
        if (no_water_sensor_ != nullptr && no_water_sensor_->state) {
            no_water_sensor_->publish_state(false);
            no_water_counter_ = 0;
        }
    }
}

uint8_t TM1628SnifferComponent::read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        while (!gpio_get_level((gpio_num_t)clk_pin_)) {}
        if (gpio_get_level((gpio_num_t)dio_pin_)) byte |= (1 << i);
        while (gpio_get_level((gpio_num_t)clk_pin_)) {}
    }
    return byte;
}

void TM1628SnifferComponent::process_transaction() {
    current_data_.clear();
    uint32_t timeout_start = esp_timer_get_time();
    while (!gpio_get_level((gpio_num_t)stb_pin_)) {
        if ((esp_timer_get_time() - timeout_start) > 50000) break;
        uint8_t byte = read_byte();
        current_data_.push_back(byte);
        delay_micros(2);
    }

    if (!current_data_.empty()) {
        decode_stream(current_data_);
    }
}

int TM1628SnifferComponent::decode_pair(uint8_t b1, uint8_t b2) {
    if (b1 == 0xE0 && b2 == 0x07) return 0;
    if (b1 == 0xC0 && b2 == 0x00) return 0;
    if (b1 == 0x00 && b2 == 0x03) return 1;
    if (b1 == 0xC0 && b2 == 0x00) return 1;
    if (b1 == 0xC0 && b2 == 0x01) return 1;
    if (b1 == 0xD0 && b2 == 0x06) return 2;
    if (b1 == 0x60 && b2 == 0x03) return 2;
    if (b1 == 0x30 && b2 == 0x05) return 3;
    if (b1 == 0x80 && b2 == 0x0A) return 3;
    if (b1 == 0x90 && b2 == 0x07) return 3;
    if (b1 == 0x30 && b2 == 0x03) return 4;
    if (b1 == 0xB0 && b2 == 0x05) return 5;
    if (b1 == 0xF0 && b2 == 0x05) return 6;
    if (b1 == 0x00 && b2 == 0x07) return 7;
    if (b1 == 0xF0 && b2 == 0x07) return 8;
    if (b1 == 0xB0 && b2 == 0x07) return 9;
    return -1;
}

std::vector<float> temp_buffer;
std::vector<float> hum_buffer;

void TM1628SnifferComponent::decode_stream(const std::vector<uint8_t> &data) {
    if (data.size() < 10) return;

    // Flood: ограничение частоты обработки
    uint32_t now = millis();
    if (now - last_publish_time_ < 100) {  // Не чаще 10 раз в секунду
        return;
    }
    last_publish_time_ = now;

    // ========================================
    // 🚨 СНАЧАЛА ПРОВЕРЯЕМ NO_WATER ОТДЕЛЬНО
    // ========================================
    bool is_no_water = false;
    float zero_ratio = 0.0f;

    // Считаем % нулей в ПЕРВЫХ 15 байтах
    int zero_count = 0;
    int total = std::min((int)data.size(), 15);
    for (int i = 0; i < total; i++) {
        if (data[i] == 0x00) zero_count++;
    }

    zero_ratio = (float)zero_count / total;

    // ESP_LOGD(TAG, "ZEROS: %.0f%% (%d/%d bytes)", zero_ratio * 100, zero_count, total);
    // ESP_LOGD(TAG, "FIRST5: %02X %02X %02X %02X %02X",
    //     data.size() > 0 ? data[0] : 0,
    //     data.size() > 1 ? data[1] : 0,
    //     data.size() > 2 ? data[2] : 0,
    //     data.size() > 3 ? data[3] : 0,
    //     data.size() > 4 ? data[4] : 0);

    // NO_WATER если:
    // 1. Нет цифр (strict_offset == -1)
    // 2. >80% нулей
    // 3. Power ON (иначе это просто выключено)
    
    // Ищем пакет с цифрами (с 0x05)
    int strict_offset = -1;
    for (size_t i = 2; i < data.size() - 8; i++) {
        if (data[i - 1] == 0x05) {
        int t10 = decode_pair(data[i], data[i + 1]);
        int h1 = decode_pair(data[i + 6], data[i + 7]);
            if (t10 != -1 && h1 != -1) {
                strict_offset = i;
                break;
            }
        }
    }

    // ДИАГНОСТИКА
    // ESP_LOGD(TAG, "PACKET: size=%d, strict_offset=%d", (int)data.size(), strict_offset);

    // ========================================
    // NO_WATER ОБРАБОТКА (ОТДЕЛЬНО!)
    // ========================================
    is_no_water = (strict_offset == -1) && (zero_ratio > 0.6f);
    
    if (is_no_water) {
        no_water_counter_++;
        last_no_water_time_ = millis();
        // ESP_LOGI(TAG, "🚨 NO_WATER: %.0f%% zeros, counter=%d/3", zero_ratio * 100, no_water_counter_);
        
        if (no_water_counter_ >= 3) {
            if (no_water_sensor_ != nullptr && !no_water_sensor_->state) {
                no_water_sensor_->publish_state(true);
                // ESP_LOGW(TAG, "🚨🚨 NO_WATER ON!");
            }
        }
        // ВЫХОДИМ - дальше не обрабатываем нулевые пакеты
        return;
    } else {
        // Если вернулись к нормальным пакетам - сбрасываем счётчик
        if (no_water_counter_ > 0) {
            no_water_counter_--;
        }
    }

    // ========================================
    // ОБРАБОТКА ЦИФР И FLAGS (только если есть 0x05)
    // ========================================
    if (strict_offset == -1) {
        // Нет цифр - выходим
        return;
    }

    // Нашли пакет с цифрами
    uint8_t flags = data[strict_offset - 2];

    // Логирование для отладки
    // ESP_LOGD(TAG, "FLAGS=0x%02X, size=%d", flags, (int)data.size());

    // ===== РАСШИРЕННАЯ ВАЛИДАЦИЯ FLAGS =====
    bool is_valid = (flags == 0x20 || flags == 0x21 ||
                    flags == 0x22 || flags == 0x28 ||
                    flags == 0x2A || flags == 0x2E ||
                    flags == 0x30 || flags == 0x40 ||
                    flags == 0x44 || flags == 0x50 ||
                    flags == 0x54 || flags == 0x60 ||
                    flags == 0x80 || flags == 0xC0 ||
                    flags == 0x00);  // Пустое состояние

    if (!is_valid) {
        // ESP_LOGW(TAG, "Unknown FLAGS value: 0x%02X, skipping", flags);
        return;
    }

    // ===== ОПРЕДЕЛЕНИЕ STATE ПО FLAGS =====
    // Основано на реальных паттернах:
    // 0x30, 0x60 = OFF, 0x20, 0x40 = ON только, 0x22, 0x44 = ON+HEAT, 0x28, 0x50 = ON+ION, 0x2A, 0x54 = ON+HEAT+ION
    bool power_on = (flags != 0x30 && flags != 0x60);  // Всё кроме 0x30 и 0x60 = Power ON
    bool ion_on = (flags == 0x28 || flags == 0x50 ||
                    flags == 0x2A || flags == 0x54);
    bool heat_on = (flags == 0x2A || flags == 0x54 ||
                    flags == 0x22 || flags == 0x44);

    // ===== ПУБЛИКУЕМ POWER С DEBOUNCE =====
    if (power_on) {
        power_on_counter_++;
        power_off_counter_ = 0;
        // Включаем Power после 2 подряд идущих ON
        if (power_on_counter_ >= 2) {
            if (power_sensor_ != nullptr && !power_sensor_->state) {
                power_sensor_->publish_state(true);
                last_power_ = true;
                // ESP_LOGI(TAG, "Power changed to: ON");
                last_active_time_ = millis();
                no_water_counter_ = 0;
                if (no_water_sensor_ != nullptr && no_water_sensor_->state) {
                    no_water_sensor_->publish_state(false);
                }
            }
        }
    } else {
        power_off_counter_++;
        power_on_counter_ = 0;
        // Выключаем Power после 2 подряд идущих OFF
        if (power_off_counter_ >= 2) {
            if (power_sensor_ != nullptr && power_sensor_->state) {
                power_sensor_->publish_state(false);
                last_power_ = false;
                // ESP_LOGI(TAG, "Power changed to: OFF");
            }
        }
    }

    // ===== ПУБЛИКУЕМ ION С DEBOUNCE (только если Power ON) =====
    if (power_on) {
        if (ion_on) {
        ion_on_counter_++;
        ion_off_counter_ = 0;
        // Включаем ION после 2 последовательных ON
            if (ion_on_counter_ >= 2) {
                if (ion_sensor_ != nullptr && !ion_sensor_->state) {
                    ion_sensor_->publish_state(true);
                    last_ion_ = true;
                }
            }
        } else {
            ion_off_counter_++;
            ion_on_counter_ = 0;
            // Выключаем ION после 2 последовательных OFF
            if (ion_off_counter_ >= 2) {
                if (ion_sensor_ != nullptr && ion_sensor_->state) {
                    ion_sensor_->publish_state(false);
                    last_ion_ = false;
                }
            }
        }
    }

    // ===== ПУБЛИКУЕМ HEAT С DEBOUNCE (только если Power ON) =====
    if (power_on) {
        if (heat_on) {
        heat_on_counter_++;
        heat_off_counter_ = 0;
        // Включаем HEAT после 2 последовательных ON
        if (heat_on_counter_ >= 2) {
            if (heat_sensor_ != nullptr && !heat_sensor_->state) {
                heat_sensor_->publish_state(true);
                last_heat_ = true;
            }
        }
        } else {
        heat_off_counter_++;
        heat_on_counter_ = 0;
        // Выключаем HEAT после 2 последовательных OFF
            if (heat_off_counter_ >= 2) {
                if (heat_sensor_ != nullptr && heat_sensor_->state) {
                    heat_sensor_->publish_state(false);
                    last_heat_ = false;
                }
            }
        }
    }

    // ===== ДЕКОДИРУЕМ ТЕМПЕРАТУРУ И ВЛАЖНОСТЬ =====
    int t10 = decode_pair(data[strict_offset], data[strict_offset + 1]);
    int t1 = decode_pair(data[strict_offset + 2], data[strict_offset + 3]);
    int h10 = decode_pair(data[strict_offset + 4], data[strict_offset + 5]);
    int h1 = decode_pair(data[strict_offset + 6], data[strict_offset + 7]);

    float t = (t10 == -1 ? 0 : t10) * 10 + (t1 == -1 ? 0 : t1);
    float h = (h10 == -1 ? 0 : h10) * 10 + (h1 == -1 ? 0 : h1);

    if (t > 0 && t < 60 && temp_sensor_ != nullptr) {
        if (abs(t - last_temp_) > 0.5) {
            temp_sensor_->publish_state(t);
            last_temp_ = t;
        }
    }

    if (h > 10 && h < 99 && humi_sensor_ != nullptr) {
        if (abs(h - last_humi_) > 1.0) {
            humi_sensor_->publish_state(h);
            last_humi_ = h;
        }
    }
}

}  // namespace tm1628_sniffer
}  // namespace esphome
