// ===========================================================================
// ПОДКЛЮЧАЕМЫЕ БИБЛИОТЕКИ
// ===========================================================================

// Стандартные библиотеки C для ввода-вывода и работы со строками
#include <stdio.h>      // printf, snprintf
#include <string.h>     // strlen, strcmp и другие строковые функции

// Библиотеки FreeRTOS — операционной системы реального времени для микроконтроллеров
#include "freertos/FreeRTOS.h"     // Ядро FreeRTOS: типы данных, макросы, функции задержки
#include "freertos/task.h"         // Управление задачами (создание, удаление, приостановка)
#include "freertos/event_groups.h" // Группы событий для синхронизации между задачами

// Библиотеки ESP-IDF для работы с Wi-Fi и системными событиями
#include "esp_wifi.h"              // Настройка и управление Wi-Fi модулем (подключение к точке доступа)
#include "esp_event.h"             // Система событий ESP-IDF (обработка подключения/отключения Wi-Fi)
#include "esp_log.h"               // Удобное логирование: ESP_LOGI, ESP_LOGE и т.д.
#include "nvs_flash.h"             // NVS — энергонезависимое хранилище (сохраняет настройки Wi-Fi между перезагрузками)

// Библиотеки для работы с датчиками
#include "driver/i2c.h"            // Драйвер I2C для общения с датчиком AGS02MA
#include "esp_adc/adc_oneshot.h"   // Драйвер АЦП для чтения аналогового сигнала с MQ-135

// Библиотеки для сетевого взаимодействия (lwIP — облегчённый TCP/IP стек)
#include "lwip/sockets.h"          // Сокеты для создания TCP-сервера
#include "lwip/netdb.h"            // Сетевые функции (преобразование адресов)

// ===========================================================================
// КОНСТАНТЫ И НАСТРОЙКИ
// ===========================================================================

static const char *TAG = "AIR_MONITOR"; // Тег для логов (будет видно в мониторе порта как "[AIR_MONITOR]")

// --- Настройки Wi-Fi ---
#define WIFI_SSID      "Beeline_2G_F19238" // Имя вашей Wi-Fi сети
#define WIFI_PASS      "oksana25"         // Пароль вашей Wi-Fi сети
#define WIFI_MAX_RETRY 5                  // Максимальное количество попыток переподключения при обрыве

// --- Настройки I2C для датчика AGS02MA ---
#define I2C_MASTER_SCL_IO   22        // Пин для тактовой линии I2C (SCL) — GPIO22 на вашей плате
#define I2C_MASTER_SDA_IO   21        // Пин для линии данных I2C (SDA) — GPIO21 на вашей плате
#define I2C_MASTER_NUM      I2C_NUM_0 // Используем первый аппаратный модуль I2C (всего их 2 на ESP32)
#define I2C_MASTER_FREQ_HZ  20000     // Частота шины I2C: 20 кГц (датчик AGS02MA требует не более 30 кГц!)
#define AGS02MA_ADDR        0x1A      // 7-битный адрес датчика AGS02MA на шине I2C

// --- Настройки АЦП для датчика MQ-135 ---
#define ADC_CHANNEL         ADC_CHANNEL_0 // Канал АЦП: GPIO36 (он же ADC1_CH0)
#define ADC_ATTEN           ADC_ATTEN_DB_12 // Ослабление входного сигнала: 12 дБ (диапазон измерения 0-3.3В)

// --- Настройки веб-сервера ---
#define WEB_SERVER_PORT     80          // Стандартный порт HTTP (браузеры открывают его по умолчанию)

// ===========================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (общие для всех задач FreeRTOS)
// ===========================================================================

static EventGroupHandle_t wifi_event_group; // Группа событий для сигнализации о подключении к Wi-Fi
#define WIFI_CONNECTED_BIT  BIT0           // Бит 0 в группе означает "Wi-Fi подключён"

static int s_retry_num = 0;     // Счётчик попыток переподключения к Wi-Fi
static int mq135_raw = 0;       // Сырое значение АЦП с датчика MQ-135 (0-4095)
static float mq135_voltage = 0.0; // Напряжение на выходе MQ-135 в вольтах
static uint32_t tvoc_ppb = 0;   // Концентрация TVOC от датчика AGS02MA в ppb (parts per billion)
static bool tvoc_valid = false;  // Флаг: true — данные AGS02MA корректны, false — ошибка чтения
static uint32_t uptime_seconds = 0; // Счётчик времени работы системы в секундах

// ===========================================================================
// ОБРАБОТЧИК СОБЫТИЙ WI-FI
// ===========================================================================

/**
 * Эта функция вызывается автоматически при любых событиях Wi-Fi и IP.
 * ESP-IDF передаёт нам тип события (event_base) и его ID (event_id).
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    // Событие: Wi-Fi модуль запустился и готов к подключению
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Начинаем процедуру подключения к заданной сети
    }
    // Событие: соединение с Wi-Fi разорвано (потеря сигнала, перезагрузка роутера и т.д.)
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) { // Если не превысили лимит попыток
            esp_wifi_connect();             // Пробуем переподключиться
            s_retry_num++;                  // Увеличиваем счётчик попыток
        }
    }
    // Событие: получен IP-адрес от роутера! Подключение успешно!
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data; // Извлекаем данные о IP
        ESP_LOGI(TAG, "Wi-Fi подключён! IP-адрес: " IPSTR, IP2STR(&event->ip_info.ip)); // Выводим IP в лог
        s_retry_num = 0; // Сбрасываем счётчик попыток
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT); // Устанавливаем флаг "подключено"
    }
}

// ===========================================================================
// ИНИЦИАЛИЗАЦИЯ WI-FI В РЕЖИМЕ СТАНЦИИ (STA)
// ===========================================================================

void wifi_init_sta(void)
{
    // 1. Создаём группу событий для сигнала "подключение установлено"
    wifi_event_group = xEventGroupCreate();

    // 2. Инициализируем низкоуровневый TCP/IP стек (lwIP)
    ESP_ERROR_CHECK(esp_netif_init());

    // 3. Создаём стандартный цикл обработки системных событий
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 4. Создаём сетевой интерфейс "Wi-Fi Станция" (подключение к роутеру, а не создание своей точки доступа)
    esp_netif_create_default_wifi_sta();

    // 5. Инициализируем Wi-Fi драйвер с параметрами по умолчанию
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 6. Регистрируем нашу функцию-обработчик на ВСЕ Wi-Fi события (ESP_EVENT_ANY_ID)
    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));

    // 7. Регистрируем отдельный обработчик на событие "получен IP-адрес"
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    // 8. Заполняем структуру с именем и паролем вашей Wi-Fi сети
    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };

    // 9. Переводим модуль в режим "Станция" (STA)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // 10. Применяем конфигурацию с именем и паролем сети
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // 11. Запускаем Wi-Fi драйвер! С этого момента начнутся попытки подключения
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Подключение к Wi-Fi сети %s...", WIFI_SSID);
}

// ===========================================================================
// ЧТЕНИЕ ДАТЧИКА AGS02MA (TVOC) ПО ПРОТОКОЛУ I2C
// ===========================================================================

/**
 * Протокол общения с AGS02MA:
 * 1. Мастер (ESP32) посылает команду 0x00 0x08 — "начать измерение"
 * 2. Ждём минимум 1500 мс (датчик выполняет замер)
 * 3. Читаем 5 байт ответа
 * 4. Проверяем контрольную сумму (CRC) для надёжности
 * 5. Собираем 4 байта данных в 32-битное число TVOC
 *
 * Возвращает ESP_OK при успехе, иначе — код ошибки
 */
esp_err_t ags02ma_read_tvoc(uint32_t *tvoc_ppb)
{
    uint8_t cmd[2] = {0x00, 0x08}; // Магическая последовательность для команды измерения

    // Шаг 1: Посылаем команду на начало измерения
    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_NUM,      // Какой модуль I2C используем (I2C_NUM_0)
        AGS02MA_ADDR,        // Адрес датчика на шине (0x1A)
        cmd,                 // Указатель на массив с командой
        sizeof(cmd),         // Сколько байт отправляем (2)
        pdMS_TO_TICKS(100)   // Таймаут: 100 миллисекунд на отправку
    );
    if (ret != ESP_OK) return ret; // Если ошибка при отправке — сразу выходим

    // Шаг 2: Ждём, пока датчик выполнит измерение (минимум 1500 мс по даташиту)
    vTaskDelay(pdMS_TO_TICKS(1550)); // Ждём 1550 мс для надёжности

    // Шаг 3: Читаем 5 байт ответа от датчика
    uint8_t data[5] = {0}; // Массив для принятых данных (инициализирован нулями)
    ret = i2c_master_read_from_device(
        I2C_MASTER_NUM, AGS02MA_ADDR, data, 5, pdMS_TO_TICKS(100)
    );
    if (ret != ESP_OK) return ret; // Если ошибка при чтении — выходим

    // Шаг 4: Проверяем CRC (последний, 5-й байт ответа)
    // Алгоритм CRC-8 с полиномом 0x31 (специфичен для AGS02MA)
    uint8_t crc = 0xFF; // Начальное значение CRC
    for (int i = 0; i < 4; i++) {          // Обходим первые 4 байта (статус + 3 байта данных)
        crc ^= data[i];                    // XOR с текущим байтом
        for (int j = 0; j < 8; j++) {      // Обрабатываем каждый бит
            if (crc & 0x80)                // Если старший бит = 1
                crc = (crc << 1) ^ 0x31;   // Сдвигаем влево и XOR с полиномом
            else
                crc <<= 1;                 // Иначе просто сдвигаем
        }
    }
    if (crc != data[4]) return ESP_FAIL; // Если CRC не совпала — данные повреждены, возвращаем ошибку

    // Шаг 5: Декодируем значение TVOC (4 байта, big-endian → 32-битное число)
    // data[1] — старший байт, data[4] — младший
    *tvoc_ppb = ((uint32_t)data[1] << 24) | // Старший байт сдвигаем на 24 бита влево
                ((uint32_t)data[2] << 16) | // Второй байт — на 16 бит
                ((uint32_t)data[3] << 8)  | // Третий байт — на 8 бит
                ((uint32_t)data[4]);        // Младший байт остаётся на месте

    return ESP_OK; // Успешное чтение
}

// ===========================================================================
// ГЕНЕРАЦИЯ HTML-СТРАНИЦЫ ДЛЯ ОТОБРАЖЕНИЯ В БРАУЗЕРЕ
// ===========================================================================

/**
 * Формирует полный HTTP-ответ с HTML-кодом и актуальными показаниями датчиков.
 * buf — буфер, куда будет записана строка (выделен в задаче веб-сервера).
 * buf_size — размер буфера (чтобы не выйти за границы).
 */
void generate_html(char *buf, size_t buf_size)
{
    // Формируем строку со значением TVOC для удобного вывода
    char tvoc_str[32];
    if (tvoc_valid) {
        // Если данные корректны: "82 ppb (~82 µg/m³)"
        snprintf(tvoc_str, sizeof(tvoc_str), "%lu ppb (~%lu µg/m³)", tvoc_ppb, tvoc_ppb);
    } else {
        // Если ошибка чтения
        snprintf(tvoc_str, sizeof(tvoc_str), "Ошибка чтения");
    }

    // Определяем текстовую оценку качества воздуха на основе TVOC
    const char *air_quality;
    if (!tvoc_valid) air_quality = "❓ Нет данных";
    else if (tvoc_ppb < 200) air_quality = "🟢 Отличный";     // Норма для жилых помещений
    else if (tvoc_ppb < 500) air_quality = "🟡 Средний";      // Желательно проветрить
    else if (tvoc_ppb < 1000) air_quality = "🟠 Плохой";      // Точно проветрить!
    else air_quality = "🔴 Очень плохой!";                    // Опасно для здоровья

    // Собираем весь HTTP-ответ с HTML-кодом
    // %s, %d, %.2f, %lu — это плейсхолдеры, которые заменятся на реальные значения датчиков
    snprintf(buf, buf_size,
        // --- HTTP-заголовки ---
        "HTTP/1.1 200 OK\r\n"                       // Код 200 = успех
        "Content-Type: text/html; charset=utf-8\r\n" // Тип контента: HTML в кодировке UTF-8
        "Connection: close\r\n"                      // Закрыть соединение после отправки страницы
        "Refresh: 5\r\n"                            // Автообновление страницы каждые 5 секунд
        "\r\n"                                      // Пустая строка — конец заголовков

        // --- Начало HTML-кода ---
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"                    // Кодировка для поддержки русских букв и эмодзи
        "<meta name='viewport' content='width=device-width, initial-scale=1'>" // Адаптация для мобильных устройств
        "<title>Мониторинг воздуха</title>"         // Заголовок вкладки браузера

        // --- CSS-стили для красивого отображения ---
        "<style>"
        "body{font-family:Arial;margin:20px;background:#f5f5f5;}" // Серый фон страницы
        ".card{background:white;border-radius:10px;padding:20px;margin:10px 0;box-shadow:0 2px 5px rgba(0,0,0,0.1);}" // Белые карточки с тенью
        ".value{font-size:2em;font-weight:bold;}"                 // Крупный жирный текст для значений
        ".label{color:#666;font-size:0.9em;}"                     // Серый текст для подписей
        "</style></head><body>"

        // --- Тело страницы ---
        "<h1>🌬️ Мониторинг воздуха</h1>"           // Главный заголовок

        // --- Карточка датчика AGS02MA (TVOC) ---
        "<div class='card'><div class='label'>AGS02MA — TVOC</div>"
        "<div class='value'>%s</div>"              // Место для значения TVOC (заменится на tvoc_str)
        "<div>Качество: <b>%s</b></div></div>"     // Оценка качества воздуха (заменится на air_quality)

        // --- Карточка датчика MQ-135 (АЦП) ---
        "<div class='card'><div class='label'>MQ-135 — АЦП</div>"
        "<div class='value'>%d</div>"              // Сырое значение АЦП (заменится на mq135_raw)
        "<div>Напряжение: %.2f В</div></div>"      // Напряжение с двумя знаками после запятой (заменится на mq135_voltage)

        // --- Карточка системной информации ---
        "<div class='card'><div class='label'>Система</div>"
        "<div>Работает: %lu сек</div></div>"       // Время работы системы (заменится на uptime_seconds)

        // --- Подвал ---
        "<p style='color:#999;font-size:0.8em;'>Обновление каждые 5 сек</p>"
        "</body></html>",

        // --- Аргументы для подстановки в плейсхолдеры (%s, %d, %.2f, %lu) ---
        tvoc_str, air_quality,    // Для первой карточки: строка TVOC и оценка качества
        mq135_raw, mq135_voltage, // Для второй карточки: сырые данные АЦП и напряжение
        uptime_seconds            // Для третьей карточки: время работы
    );
}

// ===========================================================================
// ЗАДАЧА ВЕБ-СЕРВЕРА (FreeRTOS Task)
// ===========================================================================

/**
 * Эта задача работает в бесконечном цикле:
 * 1. Ждёт подключения к Wi-Fi
 * 2. Создаёт TCP-сокет на порту 80
 * 3. В цикле принимает входящие HTTP-запросы от браузера
 * 4. Отправляет сгенерированную HTML-страницу в ответ
 */
void web_server_task(void *pvParameters)
{
    // Ждём, пока Wi-Fi подключится (ждём флаг WIFI_CONNECTED_BIT)
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Создаём TCP-сокет (IPv4, потоковый, TCP)
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    // Разрешаем переиспользовать адрес (чтобы не ждать после перезагрузки)
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Заполняем структуру адреса сервера
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;           // Используем IPv4
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Принимать подключения на всех интерфейсах (Wi-Fi)
    server_addr.sin_port = htons(WEB_SERVER_PORT);   // Порт 80

    // Привязываем сокет к адресу и порту (bind)
    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    // Переводим сокет в режим прослушивания (до 5 клиентов в очереди)
    listen(listen_sock, 5);

    ESP_LOGI(TAG, "Веб-сервер запущен на порту %d", WEB_SERVER_PORT);

    // Бесконечный цикл обработки запросов
    while (1) {
        struct sockaddr_in client_addr;       // Здесь будет адрес подключившегося клиента
        socklen_t client_len = sizeof(client_addr);

        // accept() блокируется, пока не придёт новое подключение (запрос от браузера)
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);

        if (client_sock >= 0) {
            // Читаем HTTP-запрос (нам он не важен, просто очищаем буфер)
            char recv_buf[256];
            recv(client_sock, recv_buf, sizeof(recv_buf) - 1, 0);

            // Генерируем HTML-страницу с актуальными данными датчиков
            char html[2048];
            generate_html(html, sizeof(html));

            // Отправляем ответ браузеру
            send(client_sock, html, strlen(html), 0);

            // Закрываем соединение с этим клиентом
            close(client_sock);
        }
    }
}

// ===========================================================================
// ЗАДАЧА ОПРОСА ДАТЧИКОВ (FreeRTOS Task)
// ===========================================================================

/**
 * Эта задача работает параллельно с веб-сервером.
 * Она:
 * 1. Настраивает I2C и АЦП при запуске
 * 2. В бесконечном цикле (каждые 5 секунд) читает оба датчика
 * 3. Записывает результаты в глобальные переменные (mq135_raw, tvoc_ppb и т.д.)
 */
void sensor_task(void *pvParameters)
{
    // --- Настройка I2C для датчика AGS02MA ---
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                 // ESP32 — мастер на шине I2C
        .sda_io_num = I2C_MASTER_SDA_IO,         // Пин данных: GPIO21
        .scl_io_num = I2C_MASTER_SCL_IO,         // Пин тактирования: GPIO22
        .sda_pullup_en = GPIO_PULLUP_ENABLE,     // Включаем встроенную подтяжку к 3.3В на SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,     // Включаем встроенную подтяжку к 3.3В на SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // Частота шины: 20 кГц (датчик требует ≤ 30 кГц)
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf)); // Применяем конфигурацию
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0)); // Установка драйвера I2C

    // --- Настройка АЦП для датчика MQ-135 ---
    adc_oneshot_unit_handle_t adc_handle; // "Ручка" для управления модулем АЦП

    // Инициализируем первый модуль АЦП (ADC1)
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,                   // Используем ADC1 (GPIO36 к нему относится)
        .ulp_mode = ADC_ULP_MODE_DISABLE,        // Не использовать сверхнизкое потребление
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle));

    // Конфигурируем канал (пин GPIO36 = ADC_CHANNEL_0)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,                      // Ослабление: 12 дБ (диапазон до 3.3В)
        .bitwidth = ADC_BITWIDTH_DEFAULT,        // Разрешение по умолчанию (12 бит = 0..4095)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));

    // Бесконечный цикл опроса датчиков
    while (1) {
        // === Чтение датчика MQ-135 (аналоговый сигнал через АЦП) ===
        adc_oneshot_read(adc_handle, ADC_CHANNEL, &mq135_raw);     // Получаем сырое значение (0-4095)
        mq135_voltage = (mq135_raw / 4095.0) * 3.3;    // Переводим в вольты

        // === Чтение датчика AGS02MA (цифровой сигнал по I2C) ===
        tvoc_valid = (ags02ma_read_tvoc(&tvoc_ppb) == ESP_OK); // true, если чтение успешно

        // Выводим показания в лог (видно в мониторе порта)
        ESP_LOGI(TAG, "MQ-135: %d (%.2fV) | TVOC: %lu ppb", mq135_raw, mq135_voltage, tvoc_ppb);

        // Ждём 5 секунд до следующего измерения
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ===========================================================================
// ЗАДАЧА СЧЁТЧИКА ВРЕМЕНИ РАБОТЫ (FreeRTOS Task)
// ===========================================================================

/**
 * Простейшая задача: каждую секунду увеличивает uptime_seconds на 1.
 * Нужна для отображения "времени работы системы" на веб-странице.
 */
void uptime_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Ждём ровно 1 секунду
        uptime_seconds++;                // Увеличиваем счётчик
    }
}

// ===========================================================================
// ГЛАВНАЯ ФУНКЦИЯ — ТОЧКА ВХОДА В ПРОГРАММУ
// ===========================================================================

/**
 * app_main() — это аналог функции main() в обычных программах.
 * Она выполняется при старте прошивки на ESP32.
 * Здесь мы инициализируем всё необходимое и запускаем задачи FreeRTOS.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Запуск системы мониторинга воздуха");

    // --- Инициализация NVS (энергонезависимого хранилища) ---
    // Wi-Fi драйвер хранит там калибровочные данные и настройки.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||    // Если память NVS повреждена
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)  // Или версия изменилась
    {
        ESP_ERROR_CHECK(nvs_flash_erase());    // Стираем всё
        ret = nvs_flash_init();                // Инициализируем заново
    }
    ESP_ERROR_CHECK(ret); // Проверяем, что инициализация прошла успешно

    // --- Запускаем Wi-Fi ---
    wifi_init_sta();

    // --- Создаём задачи FreeRTOS ---
    // Задачи работают ПАРАЛЛЕЛЬНО: FreeRTOS сама переключает процессор между ними.

    // Задача счётчика времени (приоритет 1 — низкий)
    xTaskCreate(uptime_task, "uptime", 2048, NULL, 1, NULL);

    // Задача опроса датчиков (приоритет 2 — средний)
    xTaskCreate(sensor_task, "sensors", 4096, NULL, 2, NULL);

    // Задача веб-сервера (приоритет 3 — высокий, чтобы быстро отвечать браузеру)
    xTaskCreate(web_server_task, "webserver", 4096, NULL, 3, NULL);

    // app_main() завершается, но задачи продолжают работать в бесконечных циклах
}