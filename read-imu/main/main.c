/*
 * BNO085 IMU over UART-SHTP for ESP-IDF (ESP32-S3)
 * -------------------------------------------------
 * Debug build: 50 Hz reports + verbose stage-by-stage logging.
 *
 * Hardware (all 3.3 V logic):
 *   ESP32-S3 GPIO10 (TX) -> BNO085 pin 19 (H_RX)
 *   ESP32-S3 GPIO11 (RX) <- BNO085 pin 20 (H_TX)
 *   ESP32-S3 GPIO13      -> BNO085 pin 11 (NRST)   <-- required for this build
 *   BNO085 PS1 (pin 5)   -> VDDIO
 *   BNO085 PS0 (pin 6)   -> GND            (UART-SHTP protocol select)
 *   BNO085 BOOTN (pin 4) -> VDDIO via 10k
 *   32.768 kHz crystal across pins 26/27, CLKSEL0 (pin 10) low/unconnected
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

static const char *TAG = "BNO085";

/* ------------------------- User configuration --------------------------- */
#define IMU_UART_PORT       UART_NUM_1
#define IMU_TX_PIN          (GPIO_NUM_10)
#define IMU_RX_PIN          (GPIO_NUM_11)
#define IMU_BAUD_RATE       (3000000)      /* BNO085 UART-SHTP is fixed 3Mb/s */

/*
 * NRST (BNO085 pin 11) driven by the ESP32. This is the key to a reliable
 * startup: the BNO085 emits its one-time SHTP advertisement right after ITS
 * boot, so we reset it only AFTER our UART is listening. If NRST is truly not
 * wired, set this to 0 to use a framed soft-reset packet instead (less robust).
 */
#define USE_BNO_RST_PIN     (1)
#define BNO_RST_PIN         (GPIO_NUM_13)

#define IMU_RX_BUF_SIZE     (4096)
#define IMU_TX_BUF_SIZE     (1024)

#define REPORT_INTERVAL_US  (20000)        /* 50 Hz */
#define PRINT_PERIOD_MS     (20)
#define HAL_READ_TIMEOUT_MS (1000)

/* ------------------------- Application state ---------------------------- */
static TaskHandle_t      s_print_task  = NULL;
static SemaphoreHandle_t s_data_mutex  = NULL;
static sh2_ProductIds_t  s_prodIds;
static bool              s_uart_ready  = false;
static uint32_t          s_hal_read_timeouts = 0;

static struct {
    sh2_SensorValue_t rotation_vector;
    sh2_SensorValue_t linear_accel;
    bool     has_rotation;
    bool     has_accel;
    uint32_t rv_count;
    uint32_t la_count;
    uint32_t reset_count;
} s_data;

/* --------------------------- UART bring-up ------------------------------ */
static esp_err_t board_uart_init(void)
{
    if (s_uart_ready) return ESP_OK;

    const uart_config_t cfg = {
        .baud_rate  = IMU_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(IMU_UART_PORT, IMU_RX_BUF_SIZE,
                                        IMU_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IMU_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(IMU_UART_PORT, IMU_TX_PIN, IMU_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush(IMU_UART_PORT);
    s_uart_ready = true;
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Reset the hub so it re-sends its boot advertisement while we are listening.
 * The advertisement then sits in the 4 KB RX ring buffer until sh2_open()
 * reads it.
 * -------------------------------------------------------------------------*/
static void bno085_reset(void)
{
#if USE_BNO_RST_PIN
    ESP_LOGI(TAG, "Resetting BNO085 via NRST (GPIO%d) ...", BNO_RST_PIN);
    gpio_set_level(BNO_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));         /* tnrst spec is only ~10 ns min */
    gpio_set_level(BNO_RST_PIN, 1);
#else
    /* Framed executable-channel reset (channel 1, cargo 0x01).
     * UART-SHTP framing: 0x7E <header+cargo> 0x7E; no bytes here need escaping. */
    static const uint8_t rst_pkt[] = { 0x7E, 0x05, 0x00, 0x01, 0x00, 0x01, 0x7E };
    ESP_LOGI(TAG, "Resetting BNO085 via SHTP soft-reset packet ...");
    uart_write_bytes(IMU_UART_PORT, (const char *)rst_pkt, sizeof(rst_pkt));
    esp_rom_delay_us(200);
#endif
    /* Internal init ~90 ms typ + ~4 ms config, then advertisement is sent. */
    vTaskDelay(pdMS_TO_TICKS(400));
    ESP_LOGI(TAG, "Reset window done - advertisement should be buffered.");
}

/* --------------- Quaternion -> Euler (from your Python) ----------------- */
typedef struct { float roll, pitch, yaw; } euler_deg_t;

static euler_deg_t quaternion_to_euler(float i, float j, float k, float real)
{
    euler_deg_t e;
    float sinr_cosp = 2.0f * (real * i + j * k);
    float cosr_cosp = 1.0f - 2.0f * (i * i + j * j);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (real * j - k * i);
    float pitch = (fabsf(sinp) >= 1.0f)
                ? copysignf((float)M_PI / 2.0f, sinp) : asinf(sinp);

    float siny_cosp = 2.0f * (real * k + i * j);
    float cosy_cosp = 1.0f - 2.0f * (j * j + k * k);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    e.roll  = roll  * 180.0f / (float)M_PI;
    e.pitch = pitch * 180.0f / (float)M_PI;
    e.yaw   = yaw   * 180.0f / (float)M_PI;
    return e;
}

/* --------------------- ESP-IDF HAL for the sh2 lib ---------------------- */
static int sh2_hal_open(sh2_Hal_t *self)
{
    ESP_LOGI(TAG, "HAL open (UART already installed).");
    return (board_uart_init() == ESP_OK) ? 0 : -1;
}

static void sh2_hal_close(sh2_Hal_t *self)
{
    ESP_LOGI(TAG, "HAL close");
    uart_driver_delete(IMU_UART_PORT);
    s_uart_ready = false;
}

static int sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                        uint32_t *t_us)
{
    int total = 0;
    while (total < (int)len) {
        int r = uart_read_bytes(IMU_UART_PORT, pBuffer + total, len - total,
                                pdMS_TO_TICKS(HAL_READ_TIMEOUT_MS));
        if (r <= 0) break;
        total += r;
    }
    if (t_us != NULL) *t_us = (uint32_t)esp_timer_get_time();

    if (total < (int)len) {
        s_hal_read_timeouts++;
        if ((s_hal_read_timeouts % 10) == 1) {   /* one line every ~10 s */
            ESP_LOGW(TAG, "hal_read: TIMEOUT (got %d of %u B) [timeout #%lu] "
                          "- hub is not talking",
                     total, len, (unsigned long)s_hal_read_timeouts);
        }
    } else {
        ESP_LOGD(TAG, "hal_read: %d bytes", total);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, pBuffer, total, ESP_LOG_DEBUG);
    }
    return total;
}

static int sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    ESP_LOGI(TAG, "hal_write: %u bytes ->", len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, pBuffer, len, ESP_LOG_INFO);
    int written = uart_write_bytes(IMU_UART_PORT, (const char *)pBuffer, len);
    if (written != (int)len) {
        ESP_LOGE(TAG, "hal_write: only %d of %u bytes written", written, len);
    }
    /* BNO08X requires >=100 us separation between host->hub UART transfers. */
    esp_rom_delay_us(150);
    return written;
}

static uint32_t sh2_hal_getTimeUs(sh2_Hal_t *self)
{
    return (uint32_t)esp_timer_get_time();
}

static sh2_Hal_t s_sh2_hal = {
    .open      = sh2_hal_open,
    .close     = sh2_hal_close,
    .read      = sh2_hal_read,
    .write     = sh2_hal_write,
    .getTimeUs = sh2_hal_getTimeUs,
};

/* ------------------------- sh2 library callbacks ------------------------ */
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent)
{
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, pEvent) != SH2_OK) return;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    switch (value.sensorId) {
    case SH2_ROTATION_VECTOR:
        if (!s_data.has_rotation) {
            ESP_LOGI(TAG, ">>> First ROTATION_VECTOR report received!");
        }
        s_data.rotation_vector = value;
        s_data.has_rotation = true;
        s_data.rv_count++;
        break;
    case SH2_LINEAR_ACCELERATION:
        if (!s_data.has_accel) {
            ESP_LOGI(TAG, ">>> First LINEAR_ACCELERATION report received!");
        }
        s_data.linear_accel = value;
        s_data.has_accel = true;
        s_data.la_count++;
        break;
    default:
        ESP_LOGI(TAG, "Report from unexpected sensor id %u", value.sensorId);
        break;
    }
    xSemaphoreGive(s_data_mutex);
}

static void async_handler(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    ESP_LOGW(TAG, "Async event id=%d", pEvent->eventId);
    if (pEvent->eventId == SH2_RESET) {
        s_data.reset_count++;
        ESP_LOGW(TAG, "BNO085 RESET event (#%lu)",
                 (unsigned long)s_data.reset_count);
        if (s_print_task != NULL) xTaskNotifyGive(s_print_task);
    }
}

/* --------------------------- Session management ------------------------- */
static esp_err_t imu_start_session(void)
{
    ESP_LOGI(TAG, "Calling sh2_open() ... (waiting for hub advertisement)");
    int status = sh2_open(&s_sh2_hal, async_handler, NULL);
    ESP_LOGI(TAG, "sh2_open() returned %d", status);
    if (status != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open failed: %d", status);
        return ESP_FAIL;
    }

    memset(&s_prodIds, 0, sizeof(s_prodIds));
    status = sh2_getProdIds(&s_prodIds);
    ESP_LOGI(TAG, "sh2_getProdIds() -> %d, numEntries=%d",
             status, s_prodIds.numEntries);
    if (status == SH2_OK && s_prodIds.numEntries > 0) {
        ESP_LOGI(TAG, "Connected. SW part 0x%08lx v%u.%u",
                 (unsigned long)s_prodIds.entry[0].swPartNumber,
                 s_prodIds.entry[0].swVersionMajor,
                 s_prodIds.entry[0].swVersionMinor);
    } else {
        ESP_LOGW(TAG, "No product IDs yet - continuing anyway.");
    }

    sh2_setSensorCallback(sensor_handler, NULL);
    return ESP_OK;
}

static esp_err_t imu_enable_reports(void)
{
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled  = false,
        .wakeupEnabled             = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled           = false,
        .changeSensitivity         = 0,
        .batchInterval_us          = 0,
        .sensorSpecific            = 0,
        .reportInterval_us         = REPORT_INTERVAL_US,
    };

    int s1 = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    ESP_LOGI(TAG, "setSensorConfig(ROTATION_VECTOR) -> %d", s1);
    int s2 = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &cfg);
    ESP_LOGI(TAG, "setSensorConfig(LINEAR_ACCELERATION) -> %d", s2);

    if (s1 != SH2_OK || s2 != SH2_OK) return ESP_FAIL;
    ESP_LOGI(TAG, "Reports enabled @ %d us (%d Hz). Expect first samples "
                  "within ~1 s.",
             REPORT_INTERVAL_US, 1000000 / REPORT_INTERVAL_US);
    return ESP_OK;
}

/* ------------------------------ Tasks ----------------------------------- */
static void imu_service_task(void *arg)
{
    ESP_LOGI(TAG, "Service task running (sh2_service loop).");
    while (1) {
        sh2_service();
        taskYIELD();
    }
}

static void imu_print_task(void *arg)
{
    s_print_task = xTaskGetCurrentTaskHandle();
    uint32_t last_rv = 0, last_la = 0;
    int iter = 0;

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            ESP_LOGW(TAG, "Reset notification -> re-enabling reports");
            imu_enable_reports();
        }

        sh2_SensorValue_t rv, la;
        bool has_rv, has_la;
        uint32_t rv_count, la_count, reset_count;
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        rv = s_data.rotation_vector;  has_rv = s_data.has_rotation;
        la = s_data.linear_accel;     has_la = s_data.has_accel;
        rv_count = s_data.rv_count;   la_count = s_data.la_count;
        reset_count = s_data.reset_count;
        xSemaphoreGive(s_data_mutex);

        if (has_rv) {
            euler_deg_t e = quaternion_to_euler(rv.un.rotationVector.i,
                                                rv.un.rotationVector.j,
                                                rv.un.rotationVector.k,
                                                rv.un.rotationVector.real);
            float ax = has_la ? la.un.linearAcceleration.x : 0.0f;
            float ay = has_la ? la.un.linearAcceleration.y : 0.0f;
            float az = has_la ? la.un.linearAcceleration.z : 0.0f;
            printf("IMU: Roll: %5.1f\xc2\xb0 Pitch: %5.1f\xc2\xb0 Yaw: %5.1f\xc2\xb0 | "
                   "Accel: X: %6.2f Y: %6.2f Z: %6.2f\n",
                   e.roll, e.pitch, e.yaw, ax, ay, az);
        } else if ((iter % 50) == 0) {   /* once per second */
            ESP_LOGW(TAG, "Waiting for first IMU sample...");
        }

        if ((++iter % 250) == 0) {       /* stats every ~5 s */
            ESP_LOGI(TAG, "STATS: rv=%lu (+%lu), accel=%lu (+%lu), "
                          "resets=%lu, read_timeouts=%lu, heap=%lu",
                     (unsigned long)rv_count, (unsigned long)(rv_count - last_rv),
                     (unsigned long)la_count, (unsigned long)(la_count - last_la),
                     (unsigned long)reset_count,
                     (unsigned long)s_hal_read_timeouts,
                     (unsigned long)esp_get_free_heap_size());
            last_rv = rv_count; last_la = la_count;
        }

        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}

/* ------------------------------ app_main -------------------------------- */
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "=== BNO085 UART-SHTP debug build (crystal on hub) ===");
    ESP_LOGI(TAG, "Report interval %d us (%d Hz), free heap %lu",
             REPORT_INTERVAL_US, 1000000 / REPORT_INTERVAL_US,
             (unsigned long)esp_get_free_heap_size());

    s_data_mutex = xSemaphoreCreateMutex();
    configASSERT(s_data_mutex);

    /* 1. Install the UART FIRST, so nothing the hub says is ever missed. */
    board_uart_init();
    ESP_LOGI(TAG, "UART%d ready @ %d baud (TX=%d, RX=%d)",
             IMU_UART_PORT, IMU_BAUD_RATE, IMU_TX_PIN, IMU_RX_PIN);

#if USE_BNO_RST_PIN
    gpio_config_t rst = {
        .pin_bit_mask = (1ULL << BNO_RST_PIN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst);
    gpio_set_level(BNO_RST_PIN, 1);        /* idle: not in reset */
#endif

    /* 2. Reset the hub (fresh advertisement) -> open session. Retry forever. */
    int attempt = 0;
    while (1) {
        attempt++;
        ESP_LOGI(TAG, "--- Connection attempt %d ---", attempt);
        bno085_reset();
        if (imu_start_session() == ESP_OK) break;
        ESP_LOGE(TAG, "Connection failed (attempt %d). Retrying in 1 s...",
                 attempt);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Success! BNO085 connected cleanly over UART.");

    if (imu_enable_reports() != ESP_OK) {
        ESP_LOGE(TAG, "Could not enable sensor reports, halting.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xTaskCreate(imu_service_task, "imu_srv", 4096, NULL, 10, NULL);
    xTaskCreate(imu_print_task,  "imu_out", 4096, NULL,  5, NULL);
    ESP_LOGI(TAG, "Tasks started.");
}