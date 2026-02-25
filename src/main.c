/*************************************************************
 * ESP32 Puzzle House (ESP-IDF / FreeRTOS) + Simple IoT Dashboard
 *
 * Modules:
 *  - PIR motion sensor  -> LED + Servo control
 *  - RC522 RFID reader  -> Logs UID when card is detected
 *  - Wi-Fi + HTTP server-> Simple dashboard in browser
 *
 * Pins (match wiring):
 *  PIR OUT   -> GPIO27
 *  LED       -> GPIO25
 *  SERVO SIG -> GPIO33
 *
 *  RC522 (SPI):
 *   SDA/SS -> GPIO5   (Tip: if you get issues, change to GPIO21)
 *   SCK    -> GPIO18
 *   MOSI   -> GPIO23
 *   MISO   -> GPIO19
 *   RST    -> GPIO22
 *   3.3V   -> 3V3 (NOT 5V)
 *   GND    -> GND
 *************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

/* Wi-Fi + Web server */
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "esp_http_server.h"

/* ===================== WIFI CREDENTIALS ===================== */
#define WIFI_SSID "SSID"
#define WIFI_PASS "password"

/* ===================== GPIO DEFINES ===================== */
#define PIR_GPIO        GPIO_NUM_27
#define LED_GPIO        GPIO_NUM_25
#define SERVO_GPIO      GPIO_NUM_33

/* RC522 pins (VSPI default on ESP32 for SCK/MISO/MOSI) */
#define RC522_SDA_SS    GPIO_NUM_5
#define RC522_SCK       GPIO_NUM_18
#define RC522_MOSI      GPIO_NUM_23
#define RC522_MISO      GPIO_NUM_19
#define RC522_RST       GPIO_NUM_22
/* IRQ not used */

#define HOLD_MS         500

static const char *TAG = "PUZZLE";
static SemaphoreHandle_t motionSemaphore;

/* ===================== DASHBOARD STATUS (globals) ===================== */
/* These are updated by your tasks and shown on the dashboard */
volatile int g_pir_state = 0;
volatile int g_led_state = 0;
volatile int g_servo_angle = 0;
char g_last_uid[32] = "NONE";

/* ===================== SERVO CONFIG ===================== */
#define SERVO_FREQ_HZ   50
#define SERVO_TIMER     LEDC_TIMER_0
#define SERVO_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL   LEDC_CHANNEL_0
#define SERVO_RES       LEDC_TIMER_16_BIT

#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000

static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = SERVO_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = SERVO_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static uint32_t us_to_duty(uint32_t us)
{
    uint32_t period_us = 1000000 / SERVO_FREQ_HZ;         // 20,000 us at 50Hz
    uint32_t max_duty  = (1U << SERVO_RES) - 1U;          // 0..65535
    return (uint32_t)(((uint64_t)us * max_duty) / period_us);
}

static void servo_set_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulse = SERVO_MIN_US +
                     ((SERVO_MAX_US - SERVO_MIN_US) * (uint32_t)angle) / 180U;

    uint32_t duty = us_to_duty(pulse);

    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));

    g_servo_angle = angle; // update dashboard status
}

/* ===================== RC522 (MFRC522) ===================== */
static spi_device_handle_t rc522_dev;

/* RC522 registers */
#define RC522_REG_COMMAND      0x01
#define RC522_REG_COMMIRQ      0x04
#define RC522_REG_ERROR        0x06
#define RC522_REG_FIFO_DATA    0x09
#define RC522_REG_FIFO_LEVEL   0x0A
#define RC522_REG_CONTROL      0x0C
#define RC522_REG_BIT_FRAMING  0x0D
#define RC522_REG_MODE         0x11
#define RC522_REG_TXASK        0x15
#define RC522_REG_TXCONTROL    0x14
#define RC522_REG_TMODE        0x2A
#define RC522_REG_TPRESCALER   0x2B
#define RC522_REG_TRELOAD_H    0x2C
#define RC522_REG_TRELOAD_L    0x2D
#define RC522_REG_VERSION      0x37

/* RC522 commands */
#define RC522_CMD_IDLE         0x00
#define RC522_CMD_TRANSCEIVE   0x0C
#define RC522_CMD_RESETPHASE   0x0F

/* PICC commands */
#define PICC_CMD_REQA          0x26
#define PICC_CMD_SEL_CL1       0x93

static esp_err_t rc522_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)((reg << 1) & 0x7E), value };
    spi_transaction_t t = {0};
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;
    return spi_device_transmit(rc522_dev, &t);
}

static esp_err_t rc522_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(((reg << 1) & 0x7E) | 0x80), 0x00 };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {0};
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(rc522_dev, &t);
    if (err == ESP_OK) *value = rx[1];
    return err;
}

static esp_err_t rc522_set_bits(uint8_t reg, uint8_t mask)
{
    uint8_t v;
    ESP_ERROR_CHECK(rc522_read_reg(reg, &v));
    return rc522_write_reg(reg, (uint8_t)(v | mask));
}

static esp_err_t rc522_clear_bits(uint8_t reg, uint8_t mask)
{
    uint8_t v;
    ESP_ERROR_CHECK(rc522_read_reg(reg, &v));
    return rc522_write_reg(reg, (uint8_t)(v & (uint8_t)(~mask)));
}

static void rc522_reset_hw(void)
{
    gpio_set_level(RC522_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(RC522_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void rc522_antenna_on(void)
{
    uint8_t v = 0;
    ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_TXCONTROL, &v));
    if ((v & 0x03) != 0x03) {
        ESP_ERROR_CHECK(rc522_set_bits(RC522_REG_TXCONTROL, 0x03));
    }
}

static void rc522_init(void)
{
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << RC522_RST,
        .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&rst));

    rc522_reset_hw();

    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_RESETPHASE));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Basic recommended timing */
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_TMODE, 0x8D));
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_TPRESCALER, 0x3E));
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_TRELOAD_L, 30));
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_TRELOAD_H, 0));

    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_TXASK, 0x40));
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_MODE, 0x3D));

    rc522_antenna_on();

    uint8_t ver = 0;
    ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_VERSION, &ver));
    ESP_LOGI(TAG, "RC522 Version: 0x%02X", ver);
}

static esp_err_t rc522_transceive(uint8_t *send, int send_len,
                                  uint8_t *back, int *back_len_bits)
{
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE));
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_COMMIRQ, 0x7F));
    ESP_ERROR_CHECK(rc522_set_bits(RC522_REG_FIFO_LEVEL, 0x80));

    for (int i = 0; i < send_len; i++) {
        ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_FIFO_DATA, send[i]));
    }

    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE));
    ESP_ERROR_CHECK(rc522_set_bits(RC522_REG_BIT_FRAMING, 0x80)); // StartSend=1

    for (int i = 0; i < 25; i++) {
        uint8_t irq = 0;
        ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_COMMIRQ, &irq));
        if (irq & 0x30) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_ERROR_CHECK(rc522_clear_bits(RC522_REG_BIT_FRAMING, 0x80));

    uint8_t err = 0;
    ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_ERROR, &err));
    if (err & 0x1B) return ESP_FAIL;

    uint8_t fifo_level = 0;
    ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_FIFO_LEVEL, &fifo_level));
    if (fifo_level == 0 || fifo_level > 16) return ESP_FAIL;

    uint8_t control = 0;
    ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_CONTROL, &control));
    int last_bits = control & 0x07;

    *back_len_bits = (fifo_level * 8) - (last_bits ? (8 - last_bits) : 0);

    for (int i = 0; i < fifo_level; i++) {
        ESP_ERROR_CHECK(rc522_read_reg(RC522_REG_FIFO_DATA, &back[i]));
    }

    return ESP_OK;
}

static bool rc522_read_uid4(uint8_t uid[4])
{
    uint8_t back[16];
    int back_bits = 0;

    uint8_t reqa[1] = { PICC_CMD_REQA };
    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_BIT_FRAMING, 0x07));
    if (rc522_transceive(reqa, 1, back, &back_bits) != ESP_OK) return false;
    if (back_bits < 16) return false;

    ESP_ERROR_CHECK(rc522_write_reg(RC522_REG_BIT_FRAMING, 0x00));
    uint8_t anticoll[2] = { PICC_CMD_SEL_CL1, 0x20 };
    if (rc522_transceive(anticoll, 2, back, &back_bits) != ESP_OK) return false;
    if (back_bits < 40) return false;

    uint8_t bcc = back[0] ^ back[1] ^ back[2] ^ back[3];
    if (bcc != back[4]) return false;

    memcpy(uid, back, 4);
    return true;
}

/* ===================== TASKS ===================== */
void led_task(void *pv)
{
    int64_t last_motion_us = 0;
    int state = 0;

    while (1) {
        if (xSemaphoreTake(motionSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            last_motion_us = esp_timer_get_time();
        }

        int64_t now = esp_timer_get_time();
        int active = (last_motion_us != 0) &&
                     ((now - last_motion_us) < (HOLD_MS * 1000));

        if (active != state) {
            state = active;
            gpio_set_level(LED_GPIO, state);
            g_led_state = state;

            if (state) {
                ESP_LOGI(TAG, "Motion ACTIVE -> LED ON, Servo 90");
                servo_set_angle(90);
            } else {
                ESP_LOGI(TAG, "Motion INACTIVE -> LED OFF, Servo 0");
                servo_set_angle(0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void pir_task(void *pv)
{
    int last = 0;

    while (1) {
        int pir = gpio_get_level(PIR_GPIO);
        g_pir_state = pir;

        if (pir && !last) {
            xSemaphoreGive(motionSemaphore);
        }

        last = pir;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void rfid_task(void *pv)
{
    uint8_t uid[4];

    while (1) {
        if (rc522_read_uid4(uid)) {
            snprintf(g_last_uid, sizeof(g_last_uid),
                     "%02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);

            ESP_LOGI(TAG, "RFID UID: %s", g_last_uid);

            vTaskDelay(pdMS_TO_TICKS(800)); // debounce
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ===================== WIFI (connect + get IP) ===================== */
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Open dashboard: http://" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done. Connecting to SSID: %s", WIFI_SSID);
}

/* ===================== HTTP SERVER (dashboard) ===================== */
static const char *DASH_HTML =
"<!doctype html><html><head><meta charset='utf-8'/>"
"<meta name='viewport' content='width=device-width,initial-scale=1'/>"
"<title>ESP32 PuzzleHouse Dashboard</title>"
"<style>"
":root{--bg:#0b1220;--card:#111a2e;--text:#e9eefc;--muted:#a9b4d0;--good:#22c55e;--bad:#ef4444;--accent:#60a5fa;}"
"*{box-sizing:border-box} body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;"
"background:radial-gradient(1200px 600px at 10% 0%, #1b2a52 0%, var(--bg) 50%) fixed;color:var(--text);}"
".wrap{max-width:980px;margin:0 auto;padding:22px;}"
".top{display:flex;gap:16px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin-bottom:14px;}"
".title{font-size:22px;font-weight:800;letter-spacing:0.3px;}"
".pill{padding:8px 12px;border:1px solid rgba(255,255,255,.12);border-radius:999px;color:var(--muted);}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px;}"
".card{background:linear-gradient(180deg, rgba(255,255,255,.05), rgba(255,255,255,.02));"
"border:1px solid rgba(255,255,255,.10);border-radius:16px;padding:16px;box-shadow:0 12px 28px rgba(0,0,0,.35);}"
".label{color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:1.2px;margin-bottom:8px;}"
".value{font-size:20px;font-weight:800;display:flex;align-items:center;gap:10px;}"
".dot{width:10px;height:10px;border-radius:99px;background:var(--bad);box-shadow:0 0 18px rgba(239,68,68,.55);}"
".dot.on{background:var(--good);box-shadow:0 0 18px rgba(34,197,94,.55);}"
".big{font-size:28px;font-weight:900;}"
".row{display:flex;align-items:center;justify-content:space-between;gap:10px;}"
".btns{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px;}"
"button{border:none;border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer;"
"background:rgba(96,165,250,.14);color:var(--text);border:1px solid rgba(96,165,250,.35);}"
"button:hover{background:rgba(96,165,250,.22)}"
".danger{background:rgba(239,68,68,.14);border:1px solid rgba(239,68,68,.35)}"
".danger:hover{background:rgba(239,68,68,.22)}"
".footer{margin-top:14px;color:var(--muted);font-size:12px;opacity:.9}"
"</style></head><body>"
"<div class='wrap'>"
" <div class='top'>"
"   <div class='title'>ESP32 PuzzleHouse Dashboard</div>"
"   <div class='pill'>Live updates: <span id='ts'>--</span></div>"
" </div>"

" <div class='grid'>"

"  <div class='card'>"
"    <div class='label'>PIR Motion</div>"
"    <div class='value'><span id='pirDot' class='dot'></span><span id='pirTxt'>-</span></div>"
"  </div>"

"  <div class='card'>"
"    <div class='label'>LED Status</div>"
"    <div class='value'><span id='ledDot' class='dot'></span><span id='ledTxt'>-</span></div>"
"  </div>"

"  <div class='card'>"
"    <div class='label'>Servo Angle</div>"
"    <div class='row'>"
"      <div class='big'><span id='servoTxt'>-</span>°</div>"
"      <div class='btns'>"
"        <button onclick=\"setServo(0)\">0°</button>"
"        <button onclick=\"setServo(90)\">90°</button>"
"        <button onclick=\"setServo(180)\">180°</button>"
"      </div>"
"    </div>"
"  </div>"

"  <div class='card'>"
"    <div class='label'>Last RFID UID</div>"
"    <div class='value'><span style='color:var(--accent);font-weight:900' id='uidTxt'>-</span></div>"
"    <div class='footer'>Tip: Tap the blue tag to update UID.</div>"
"  </div>"

" </div>"

" <div class='footer'>ESP32 + FreeRTOS + ESP-IDF Web Server (local IoT dashboard)</div>"
"</div>"

"<script>"
"function setDot(el,on){el.classList.toggle('on',!!on);}"
"function nowStr(){const d=new Date();return d.toLocaleTimeString();}"
"async function setServo(a){try{await fetch('/servo?angle='+a);}catch(e){}}"
"async function tick(){"
"  try{"
"    const r=await fetch('/status',{cache:'no-store'});"
"    const j=await r.json();"
"    document.getElementById('pirTxt').textContent = j.pir ? 'MOTION DETECTED' : 'NO MOTION';"
"    document.getElementById('ledTxt').textContent = j.led ? 'ON' : 'OFF';"
"    document.getElementById('servoTxt').textContent = j.servo;"
"    document.getElementById('uidTxt').textContent = j.uid;"
"    setDot(document.getElementById('pirDot'), j.pir);"
"    setDot(document.getElementById('ledDot'), j.led);"
"    document.getElementById('ts').textContent = nowStr();"
"  }catch(e){"
"    document.getElementById('ts').textContent = 'Disconnected';"
"  }"
"}"
"setInterval(tick,1000); tick();"
"</script>"
"</body></html>";



static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, DASH_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    char json[128];
    snprintf(json, sizeof(json),
             "{\"pir\":%d,\"led\":%d,\"servo\":%d,\"uid\":\"%s\"}",
             (int)g_pir_state, (int)g_led_state, (int)g_servo_angle, g_last_uid);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t servo_get_handler(httpd_req_t *req)
{
    char query[64] = {0};
    int angle = g_servo_angle;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[16] = {0};
        if (httpd_query_key_value(query, "angle", param, sizeof(param)) == ESP_OK) {
            angle = atoi(param);
        }
    }

    servo_set_angle(angle);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error starting server!");
        return NULL;
    }

    httpd_uri_t root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root);

    httpd_uri_t status = {
        .uri      = "/status",
        .method   = HTTP_GET,
        .handler  = status_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &status);

    httpd_uri_t servo = {
        .uri      = "/servo",
        .method   = HTTP_GET,
        .handler  = servo_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &servo);

    ESP_LOGI(TAG, "HTTP server started");
    return server;
}

/* ===================== MAIN ===================== */
void app_main(void)
{
    /* PIR input */
    gpio_config_t pir = {
        .pin_bit_mask = 1ULL << PIR_GPIO,
        .mode = GPIO_MODE_INPUT
    };
    ESP_ERROR_CHECK(gpio_config(&pir));

    /* LED output */
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&led));
    gpio_set_level(LED_GPIO, 0);
    g_led_state = 0;

    /* Servo PWM */
    servo_init();
    servo_set_angle(0);

    /* Motion semaphore */
    motionSemaphore = xSemaphoreCreateBinary();
    if (motionSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create motion semaphore!");
        return;
    }

    /* SPI init for RC522 (SPI2_HOST / VSPI) */
    spi_bus_config_t buscfg = {
        .mosi_io_num = RC522_MOSI,
        .miso_io_num = RC522_MISO,
        .sclk_io_num = RC522_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,   // 1MHz safe
        .mode = 0,
        .spics_io_num = RC522_SDA_SS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &rc522_dev));

    rc522_init();

    /* Start Wi-Fi + web dashboard */
    wifi_init_sta();

    /* Start web server after Wi-Fi connects (optional wait) */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    start_webserver();

    /* Tasks */
    xTaskCreate(led_task,  "LED+SERVO", 3072, NULL, 2, NULL);
    xTaskCreate(pir_task,  "PIR",       2048, NULL, 3, NULL);
    xTaskCreate(rfid_task, "RFID",      4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "System ready. PIR warm-up 30–60s.");
}
