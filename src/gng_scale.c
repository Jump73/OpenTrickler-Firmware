#include <FreeRTOS.h>
#include <queue.h>
#include <stdlib.h>
#include <semphr.h>
#include <time.h>
#include <math.h>
#include <task.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/uart.h"
#include "configuration.h"
#include "scale.h"
#include "app.h"

const static char CMD_REQUEST_DATA_TRANSFER[] = "!p\r\n";
const static char CMD_CALIBRATE_FUNC[] = "!q\r\n";
// const static char CMD_COUNTING_FUNC[] = "!r\r\n";
const static char CMD_CHANGE_WEIGHT_UNIT[] = "!s\r\n";
const static char CMD_TARE_FUNC[] = "!t\r\n";
const static char CMD_BACKLIGHT[] = "!u\r\n";

// The G&G scale can ignore !t when it arrives during a !p data request.
// Queue tare so the reader task owns the whole UART transaction.
static volatile bool gng_tare_requested = false;
static volatile bool gng_tare_in_progress = false;



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

extern scale_config_t scale_config;

// Instance of the scale handle for G&G JJB series
scale_handle_t gng_scale_handle = {
    .read_loop_task = _gng_scale_listener_task,
    .force_zero = scalegng_press_tare_key,
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
    float weight = strtof(msg->data, &endptr);

    if( endptr == msg->data ) {
        // Conversion failed
        return nanf(msg->data);
    }

    // Apply the sign
    weight *= sign;

    return weight;
}

//read UART
void _gng_scale_listener_task(void *p) {
    uint8_t string_buf_idx = 0;
    gngscale_standard_data_format_t frame;

    while (true) {
        bool perform_tare = false;
        taskENTER_CRITICAL();
        if (gng_tare_requested && !gng_tare_in_progress) {
            gng_tare_requested = false;
            gng_tare_in_progress = true;
            perform_tare = true;
        }
        taskEXIT_CRITICAL();

        if (perform_tare) {
            // The previous !p response has completed (or timed out). Leave
            // the balance idle before !t, then avoid !p while tare settles.
            vTaskDelay(pdMS_TO_TICKS(100));
            while (uart_is_readable(SCALE_UART)) {
                uart_getc(SCALE_UART);
            }
            scale_write(CMD_TARE_FUNC, strlen(CMD_TARE_FUNC));
            uart_tx_wait_blocking(SCALE_UART);
            vTaskDelay(pdMS_TO_TICKS(500));
            taskENTER_CRITICAL();
            gng_tare_in_progress = false;
            taskEXIT_CRITICAL();
        }

        // Reset frame buffer index at the start of each poll cycle (fixes partial-frame carryover on timeout)
        string_buf_idx = 0;

        // Clear any stale data in RX buffer before sending request
        while (uart_is_readable(SCALE_UART)) {
            uart_getc(SCALE_UART);
        }

        // Request for a data transfer using mutex-protected write
        scale_write(CMD_REQUEST_DATA_TRANSFER, strlen(CMD_REQUEST_DATA_TRANSFER));

        // Wait for scale to respond (up to 200ms)
        TickType_t start_tick = xTaskGetTickCount();
        bool got_response = false;

        while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(200)) {
            while (uart_is_readable(SCALE_UART)) {
                char ch = uart_getc(SCALE_UART);
                frame.bytes[string_buf_idx++] = ch;

                // If we have received 14 bytes then we can decode the message
                if (string_buf_idx == sizeof(gngscale_standard_data_format_t)) {
                    // Data is ready, send to decode
                    scale_config.current_scale_measurement = _decode_measurement_msg(&frame);
                    // Signal the data is ready
                    if (scale_config.scale_measurement_ready) {
                        xSemaphoreGive(scale_config.scale_measurement_ready);
                    }

                    // Reset
                    string_buf_idx = 0;
                    got_response = true;
                }

                // \n is the terminator. Reset on receiving it.
                if (ch == '\n') {
                    string_buf_idx = 0;
                    if (got_response) {
                        break;
                    }
                }
            }

            if (got_response) {
                break;
            }

            // Small delay before checking again
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Wait before next request
        vTaskDelay(pdMS_TO_TICKS(50));
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
    // Coalesce repeated requests, including multiple WWW button clicks.
    taskENTER_CRITICAL();
    if (!gng_tare_in_progress) {
        gng_tare_requested = true;
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
