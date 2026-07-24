# 🌱 Bewässerung 2.1 — переработка автоматизации полива для Home Assistant

Полная переработка конфигурации полива (Sonoff 4CH/Tasmota + Sonoff POW + MiFlora +
датчик дождя) с сохранением **всей** функциональности, современными улучшениями из 2.0
и **шестью расширениями** версии 2.1. Пакет сверен с вашим реальным дашбордом —
все используемые им сущности покрыты (см. раздел «Дашборд»).

Синтаксис Home Assistant 2024.10+: `trigger`/`condition`/`action`, синхронные вызовы
скриптов, `repeat.for_each`, `input_datetime`, `weather.get_forecasts`, `telegram_callback`.

---

## 📁 Структура пакета

| Файл | Содержимое |
|---|---|
| `irrigation_01_helpers.yaml` | все `input_*`-хелперы, `timer`, пороги, параметры расхода + **новые хелперы 2.1** |
| `irrigation_02_devices.yaml` | template-сенсоры/свитчи, группа вентилей, `pump_active`, customize |
| `irrigation_03_engine.yaml` | **ядро**: автоматизации + скрипты (расписание, погода, дни недели, адаптив, Telegram-кнопки) |
| `irrigation_04_statistics.yaml` | `history_stats`, `utility_meter`, расход м³/€, уведомление об окончании |
| `irrigation_05_dashboard.yaml` | Lovelace-хелперы, скрипт вкладок, пример дашборда |
| `irrigation_06_ui_helpers.yaml` | `_view`-хелперы раскрытия деталей + вкладка «Einstellungen» (нужны реальному дашборду) |
| `irrigation_07_pump_priming.yaml` | **регулярная подкачка воды** (Regelmässige Pumpenfunktion) + длительность |
| `blueprints/automation/irrigation_cycle_blueprint.yaml` | **Blueprint** ядра для повторного использования |

Подключение через **packages** в `configuration.yaml`:

```yaml
homeassistant:
  packages:
    irrigation: !include_dir_named packages/irrigation
```

Положите файлы `irrigation_0X_*.yaml` в `config/packages/irrigation/`.
Blueprint — в `config/blueprints/automation/` (отдельно, по желанию).

---

## 🆕 Шесть расширений версии 2.1

### [W] Прогноз погоды — пропуск полива при ожидаемом дожде
- `input_boolean.irrigation_use_weather_forecast` — вкл/выкл функцию.
- `input_text.irrigation_weather_entity` — ваша weather-сущность (по умолч. `weather.home`).
- `input_number.irrigation_forecast_hours` — окно прогноза (1–24 ч, по умолч. 12).
- `input_number.irrigation_forecast_rain_threshold` — порог вероятности осадков (по умолч. 60 %).
- Логика: перед запуском скрипт `irrigation_check_sensor_data` вызывает
  `weather.get_forecasts` (hourly), берёт **максимальную** вероятность осадков в окне
  и пропускает цикл, если она ≥ порога.
- Бонус для UI: `automation.irrigation_update_forecast` каждые 30 мин обновляет
  `input_number.irrigation_forecast_rain_now` (текущая макс. вероятность).

> Требуется интеграция `weather` с поддержкой почасового прогноза и поля
> `precipitation_probability` (OpenWeatherMap, Met.no, AccuWeather и др.).

### [D] Позоны по дням недели
- `input_select.irrigation_cycleX_schedule_mode` — режим: **Intervall** (как раньше)
  или **Wochentage** (конкретные дни).
- 7 переключателей на цикл: `input_boolean.irrigation_cycleX_day_mon … _sun`.
- Расчёт следующего запуска (`irrigation_update_next_runtime`) сканирует 14 дней вперёд
  и выбирает первый подходящий день. Режим «Intervall» сохраняет прежнее поведение
  (Täglich / Alle 2 / Alle 3 / Einmal).

### [S] Отдельные датчики влажности на зону
- `input_text.irrigation_zoneX_moisture_sensor` — entity_id датчика влажности для зоны
  (по умолч. у всех `sensor.rasen_moisture`).
- Скрипт зоны читает `states(нужный_датчик)` — можно поставить свой MiFlora на каждую зону.
- Пороги влажности по-прежнему отдельные: `input_number.irrigation_zoneX_moisture_threshold`.

### [A] Адаптивная длительность (ET-эвристика)
- `input_boolean.irrigation_adaptive_duration` — вкл/выкл.
- Формула фактора: `f = (Temp / Basistemp) · (1 + 0.5 · Feuchtedefizit/Schwelle)`,
  ограниченная `[min_scale, max_scale]`.
  - `input_number.irrigation_adaptive_base_temp` (по умолч. 20 °C)
  - `input_number.irrigation_adaptive_min_scale` (по умолч. 0.5×)
  - `input_number.irrigation_adaptive_max_scale` (по умолч. 1.5×)
- Итоговая длительность = базовая × фактор. Используется и для `delay`/таймера,
  и для Tasmota Pulsetime (аппаратный failsafe согласован).
- Это прозрачная эвристика, а не строгий расчёт ET₀ (Hargreaves/FAO-56); при желании
  формулу в скрипте `irrigation_irrigate_a_zone` (переменная `scale`) можно заменить
  на более точную модель.

### [N] Actionable-уведомления (кнопки в Telegram)
- `input_boolean.irrigation_actionable_notify` — вкл/выкл кнопки.
- `input_number.irrigation_extend_minutes` — шаг продления (по умолч. 10 мин).
- При старте зоны приходит сообщение с inline-кнопками **▶️ +N min** и **⏹️ Stopp**.
- Обработка кнопок — автоматизации `irrigation_telegram_verlangern` /
  `irrigation_telegram_abbrechen` (триггер `telegram_callback`):
  - **Stopp** → `script.irrigation_abort` (вентили + насос + скрипты + сброс).
  - **Verlängern** → `script.irrigation_extend_zone`: остаток таймера + N мин,
    с обновлением Tasmota Pulsetime (чтобы аппаратный failsafe не закрыл вентиль раньше).
- Уведомление о **старте** теперь отправляет ядро (с кнопками или без);
  `irrigation_04_statistics.yaml` сообщает только об **окончании** зоны с длительностью
  за день — без дублей.

> Требуется настроенная интеграция `telegram_bot` и секрет `telegram_channel`.

### [R] Трекинг активного прогона
- `input_text.irrigation_active_run` хранит `«цикл:зона»` (напр. `1:2`) во время полива
  и `-` в простое — используется кнопками «Продлить/Прервать» и виден в UI.

---

## 💧 Регулярная подкачка воды (Regelmässige Pumpenfunktion)

Ваша существующая автоматизация `Irrigation - Regular pumping` перенесена в пакет
(`irrigation_07_pump_priming.yaml`) с сохранением логики и небольшими улучшениями:

- Каждые 12 ч (в `00:02` и `12:02`) насос кратковременно включается — защита от
  простоя/закисания и поддержание давления.
- Только когда насос **выключен** (во время полива насос и так включён — подкачка
  не мешает; дополнительно стоит явная проверка, что ни один цикл не выполняется).
- **Работает независимо от мастер-выключателя** (это защита самого насоса). Чтобы
  полностью отключить — выключите автоматизацию `irrigation_regular_pumping` в UI
  (она видна на дашборде в блоке мастера и на вкладке «Einstellungen»).
- **Улучшение:** длительность толчка теперь настраивается через
  `input_number.irrigation_prime_seconds` (по умолч. 15 с, диапазон 5–60 с) вместо
  жёстко зашитых 15 с.

---

## 📱 Дашборд (Dashboard_2.1.yml)

Ваш реальный `Dashboard.yml` (layout `sections`, кастомная `landroid-card`, плитки
`tile`, бейджи) проанализирован и мигрирован в **`Dashboard_2.1.yml`** (оригинал не
изменяется). Что сделано:

- **Миграция времени старта:** плитки `Startzeit` переведены с удалённого
  `input_select.irrigation_cycleX_schedule_time` на
  `input_datetime.irrigation_cycleX_start_time` (время задаётся в more-info плитки).
- **Новая вкладка «Einstellungen»** (кнопка добавлена к существующим вкладкам
  Morgen / Abend / Verlauf & Co. / Verbrauch; работает через тот же
  `script.irrigation_views_toggle`). На вкладке — 41 плитка всех настроек 2.1:
  Wettervorhersage, Adaptive Dauer, Benachrichtigungen, Feuchtesensoren je Zone,
  Wochentage (Morgen/Abend), Pumpe (подкачка + длительность), Status.
- **Добавлены `_view`-хелперы** (`irrigation_06_ui_helpers.yaml`), которые использует
  ваш дашборд для раскрытия деталей: `irrigation_cycleX_enable_view`,
  `irrigation_cycleX_use_sensor_data_view`, а также `irrigation_view_settings`.
- **Всё остальное сохранено без изменений:** секция газонокосилки (landroid-card),
  блок мастера (picture-glance), плитки вентилей/насоса, растение, статистика,
  потребление воды и электричества, бейджи, тема и `visible` (права пользователей).

> Аппаратные переключатели `switch.irrigation_pump` и `switch.irrigation_zoneX_valve`
> создаются интеграцией Tasmota/MQTT, а не пакетом — пакет ими только управляет.
> Они есть в вашей системе, поэтому в пакете не определяются.

Импорт: в UI дашборда создайте/замените представление и вставьте содержимое
`Dashboard_2.1.yml` (Raw-редактор), либо положите его как YAML-дашборд.

---

## ✅ Что сохранено (функциональность ≥ исходной)

- 2 цикла (утро/вечер), ручной запуск/отмена, учёт температуры/влажности/дождя
- Управление насосом по вентилям (вкл/выкл с задержками), Tasmota Pulsetime как HW-failsafe
- Каскад failsafe (master-off, HA-старт, взаимное исключение вентилей, авто-отключение 2 ч)
- **Регулярная подкачка воды** (каждые 12 ч, настраиваемая длительность)
- Статистика: реальное время насоса (>100 Вт), расход м³ и стоимость по зонам
- MiFlora (`sensor.rasen_*`) и датчик дождя газонокосилки
- Data-driven зоны (`repeat.for_each`) — Zone 3/Hahn активируется списком `[1, 2, 3]`

---

## 🆙 Улучшения версии 2.0 (напоминание)

1. `input_datetime` вместо `input_select` на 96 опций
2. Data-driven зоны через `repeat.for_each`
3. Синхронные вызовы скриптов (без `wait_template`-polling)
4. Улучшенная обработка дождя (останов всего + уведомление)
5. Failsafe всегда активны
6. Исправлен баг Tasmota Pulsetime (сравнение строки с числом)
7. Упрощён ручной запуск (убран «танец» с `*_enable_saved_state`)

---

## 🔄 Новые / изменённые entity_id в 2.1

| Тип | Entity | Назначение |
|---|---|---|
| input_boolean | `irrigation_use_weather_forecast` | [W] вкл прогноз |
| input_text | `irrigation_weather_entity` | [W] weather-сущность |
| input_number | `irrigation_forecast_hours` / `_rain_threshold` / `_rain_now` | [W] параметры/индикатор |
| input_select | `irrigation_cycleX_schedule_mode` | [D] Intervall/Wochentage |
| input_boolean | `irrigation_cycleX_day_mon…sun` (×14) | [D] дни недели |
| input_text | `irrigation_zoneX_moisture_sensor` (×3) | [S] датчик на зону |
| input_boolean | `irrigation_adaptive_duration` | [A] вкл адаптив |
| input_number | `irrigation_adaptive_min_scale/_max_scale/_base_temp` | [A] параметры |
| input_boolean | `irrigation_actionable_notify` | [N] вкл кнопки |
| input_number | `irrigation_extend_minutes` | [N] шаг продления |
| input_text | `irrigation_active_run` | [R] цикл:зона |
| automation | `irrigation_update_forecast` | [W] обновление индикатора |
| automation | `irrigation_telegram_abbrechen` / `_verlangern` | [N] обработка кнопок |
| script | `irrigation_abort`, `irrigation_extend_zone` | [N] действия кнопок |
| automation | `irrigation_regular_pumping` | 💧 регулярная подкачка |
| input_number | `irrigation_prime_seconds` | 💧 длительность подкачки |
| input_boolean | `irrigation_cycleX_enable_view` / `_use_sensor_data_view` (×4) | 📱 раскрытие деталей дашборда |
| input_boolean | `irrigation_view_settings` | 📱 вкладка «Einstellungen» |

Все entity_id из 2.0 сохранены. Новые функции по умолчанию **выключены**
(`use_weather_forecast`, `adaptive_duration`, `actionable_notify` = off;
`schedule_mode` = Intervall) — поведение идентично 2.0, пока вы их не включите.

---

## 🧩 Blueprint (альтернатива пакету)

`blueprints/automation/irrigation_cycle_blueprint.yaml` — само-contained автоматизация
одного цикла (до 2 зон) с выбором дней недели, проверками температуры/влажности/дождя,
пропуском по прогнозу и уведомлением. Удобно, если вы предпочитаете настраивать полив
через UI (Einstellungen → Automatisierungen → Blueprints), а не YAML-пакетом.

> Blueprint намеренно проще пакета (фиксированная длительность, без Tasmota-Pulsetime,
> адаптива и Telegram-кнопок). Полный набор функций — в YAML-пакете.

---

## 🚀 Быстрый старт после установки

1. Скопируйте файлы в `config/packages/irrigation/`, перезапустите HA
   (или «Инструменты разработчика → YAML → Проверить и перезагрузить»).
2. Задайте стартовое время в `input_datetime.irrigation_cycleX_start_time`.
3. Импортируйте `Dashboard_2.1.yml` в UI дашборда (Raw-редактор).
4. При необходимости укажите `input_text.irrigation_weather_entity`
   и свои датчики влажности в `input_text.irrigation_zoneX_moisture_sensor`.
5. Включите нужные функции (погода / адаптив / дни недели / Telegram-кнопки).
6. Проверьте конфигурацию: **Инструменты разработчика → YAML → Проверить конфигурацию**.

Все файлы проверены на корректность YAML и согласованность ссылок между entity_id,
а дашборд — на совместимость с пакетом (ячейки валидации в ноутбуке проекта).
