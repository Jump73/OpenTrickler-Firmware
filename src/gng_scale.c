#include <FreeRTOS.h>
#include <queue.h>
#include <stdlib.h>
#include <semphr.h>
#include <time.h>
#include <math.h>
#include <task.h>
#include <string.h>

#include "hardware/uart.h"
#include "configuration.h"
#include "scale.h"
#include "app.h"

static const char CMD_REQUEST_DATA_TRANSFER[] = "!p\r\n";
static const char CMD_CALIBRATE_FUNC[] = "!q\r\n";
// const static char CMD_COUNTING_FUNC[] = "!r\r\n";
static const char CMD_CHANGE_WEIGHT_UNIT[] = "!s\r\n";
static const char CMD_TARE_FUNC[] = "!t\r\n";
static const char CMD_BACKLIGHT[] = "!u\r\n";

// G&G can beep on a remote TARE without actually zeroing. The listener owns
// both polling and taring so a new !p cannot interrupt the tare operation.
#define GNG_TARE_QUIET_BEFORE_MS        100
#define GNG_TARE_QUIET_AFTER_MS         500
#define GNG_TARE_VERIFY_TIMEOUT_MS    2500
#define GNG_TARE_POLL_GAP_MS            50
#define GNG_TARE_ZERO_TOLERANCE       0.05f
#define GNG_TARE_ZERO_SAMPLES             3
#define GNG_TARE_MAX_ATTEMPTS             2

static volatile scale_tare_status_t gng_tare_status = SCALE_TARE_IDLE;
static volatile bool gng_listener_ready = false;



typedef union {
    struct __attribute__((__packed__)){
        char header[2];
        char data[7];
        char unit[3];
        char terminator[2];
    };
    char bytes[14];
} gngscale_standard_data_format_t;


// Forward declaration
void _gng_scale_listener_task(void *p);
void scalegng_press_print_key();
void scalegng_press_tare_key();
int scalegng_get_tare_status(void);

extern scale_config_t scale_config;

// Instance of the scale handle for G&G JJB series
scale_handle_t gng_scale_handle = {
    .read_loop_task = _gng_scale_listener_task,
    .force_zero = scalegng_press_tare_key,
    .get_tare_status = scalegng_get_tare_status,
};

static float _decode_measurement_msg(gngscale_standard_data_format_t * msg) {
    // Decode header
    // Determine sign
    int sign = 1;

    // Check if the header contains a negative sign
    if (msg->header[0] == '-') {
        sign = -1;
    }

    // Decode weight information
    char *endptr;
    char numeric_data[sizeof(msg->data) + 1];
    memcpy(numeric_data, msg->data, sizeof(msg->data));
    numeric_data[sizeof(msg->data)] = '\0';
    float weight = strtof(numeric_data, &endptr);

    if (endptr == numeric_data) {
        // Conversion failed
        return NAN;
    }

    // Apply the sign
    weight *= sign;

    return weight;
}

// Called only by the G&G listener task. Each poll is a complete UART
// transaction: no other G&G command is started until it ends or times out.
static bool _gng_poll_once(float *measurement) {
    uint8_t string_buf_idx = 0;
    gngscale_standard_data_format_t frame;

    // Remove any late bytes from a previous, timed-out response.
    while (uart_is_readable(SCALE_UART)) {
        uart_getc(SCALE_UART);
    }

    scale_write(CMD_REQUEST_DATA_TRANSFER, strlen(CMD_REQUEST_DATA_TRANSFER));

    TickType_t start_tick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(200)) {
        while (uart_is_readable(SCALE_UART)) {
            char ch = uart_getc(SCALE_UART);
            frame.bytes[string_buf_idx++] = ch;

            if (string_buf_idx == sizeof(frame)) {
                string_buf_idx = 0;
                // Reject malformed/partial frames; they must not confirm tare.
                if (frame.terminator[0] == '\r' && frame.terminator[1] == '\n') {
                    float weight = _decode_measurement_msg(&frame);
                    if (isfinite(weight)) {
                        scale_config.current_scale_measurement = weight;
                        if (scale_config.scale_measurement_ready) {
                            xSemaphoreGive(scale_config.scale_measurement_ready);
                        }
                        if (measurement) {
                            *measurement = weight;
                        }
                        return true;
                    }
                }
            }

            if (ch == '\n') {
                string_buf_idx = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return false;
}

static void _gng_set_tare_status(scale_tare_status_t status) {
    taskENTER_CRITICAL();
    gng_tare_status = status;
    taskEXIT_CRITICAL();
}

int scalegng_get_tare_status(void) {
    taskENTER_CRITICAL();
    scale_tare_status_t status = gng_tare_status;
    taskEXIT_CRITICAL();
    return (int)status;
}

// Only the listener may change PENDING to RUNNING. Requests arriving while a
// tare is in progress are coalesced, rather than sending overlapping !t's.
static bool _gng_claim_tare_request(void) {
    taskENTER_CRITICAL();
    bool requested = (gng_tare_status == SCALE_TARE_PENDING);
    if (requested) {
        gng_tare_status = SCALE_TARE_RUNNING;
    }
    taskEXIT_CRITICAL();
    return requested;
}

static void _gng_perform_tare(void) {
    for (unsigned attempt = 0; attempt < GNG_TARE_MAX_ATTEMPTS; ++attempt) {
        // The preceding !p transaction is finished. Give the balance a quiet
        // gap before !t, and do not issue any !p while it processes the key.
        vTaskDelay(pdMS_TO_TICKS(GNG_TARE_QUIET_BEFORE_MS));
        while (uart_is_readable(SCALE_UART)) {
            uart_getc(SCALE_UART);
        }
        scale_write(CMD_TARE_FUNC, strlen(CMD_TARE_FUNC));
        uart_tx_wait_blocking(SCALE_UART);
        vTaskDelay(pdMS_TO_TICKS(GNG_TARE_QUIET_AFTER_MS));

        unsigned zero_readings = 0;
        TickType_t start_tick = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(GNG_TARE_VERIFY_TIMEOUT_MS)) {
            float weight = NAN;
            if (_gng_poll_once(&weight) && fabsf(weight) <= GNG_TARE_ZERO_TOLERANCE) {
                if (++zero_readings >= GNG_TARE_ZERO_SAMPLES) {
                    _gng_set_tare_status(SCALE_TARE_SUCCESS);
                    return;
                }
            } else {
                zero_readings = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(GNG_TARE_POLL_GAP_MS));
        }
    }

    _gng_set_tare_status(SCALE_TARE_FAILED);
}

// Read UART and handle queued tare requests in one task, avoiding the race
// between the WWW/encoder task sending !t and this task polling with !p.
void _gng_scale_listener_task(void *p) {
    (void)p;
    taskENTER_CRITICAL();
    gng_listener_ready = true;
    taskEXIT_CRITICAL();

    while (true) {
        if (_gng_claim_tare_request()) {
            _gng_perform_tare();
        } else {
            _gng_poll_once(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(GNG_TARE_POLL_GAP_MS));
    }
}

//G&G JJB key function
//C4 communication setting - data signal command control 
//standard ESC 0x1B to ! 0x22
//hint:
//For better syntax change standard setting 'C4' 27 'ESC' symbol at scale to 33 '!' sign 

//ESC p -> 0x1b 0x70 0x0D 0x0A standard setting
// ! p -> 0x21 0x70 0x0D 0x0A
void scalegng_press_print_key() {
    scale_write(CMD_REQUEST_DATA_TRANSFER, strlen(CMD_REQUEST_DATA_TRANSFER));
}

//ESC t -> 0x1B 0x74 0x0D 0x0A standard setting
// ! t -> 0x21 0x74 0x0D 0x0A
void scalegng_press_tare_key() {
    taskENTER_CRITICAL();
    if (!gng_listener_ready) {
        gng_tare_status = SCALE_TARE_FAILED;
    } else if (gng_tare_status != SCALE_TARE_PENDING &&
               gng_tare_status != SCALE_TARE_RUNNING) {
        gng_tare_status = SCALE_TARE_PENDING;
    }
    taskEXIT_CRITICAL();
}

//ESC s -> 0x1B 0x73 0x0D 0x0A standard setting
// ! s -> 0x21 0x73 0x0D 0x0A
void scalegng_press_weight_key() {
    scale_write(CMD_CHANGE_WEIGHT_UNIT, strlen(CMD_CHANGE_WEIGHT_UNIT));
}

//ESC q -> 0x1B 0x71 0x0D 0x0A standard setting
// ! q -> 0x21 0x71 0x0D 0x0A
void scalegng_press_cal_key() {
    scale_write(CMD_CALIBRATE_FUNC, strlen(CMD_CALIBRATE_FUNC));
}

// ESC u -> 0x1B 0x75 0x0D 0x0A standard setting
// ! u -> 0x21 0x75 0x0D 0x0A
void scalegng_display_light() {
    scale_write(CMD_BACKLIGHT, strlen(CMD_BACKLIGHT));
}

// AppState_t scale_enable_fast_report(AppState_t prev_state) {
//     // TODO: Finish this
// }
