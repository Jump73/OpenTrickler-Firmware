// Standalone host test for the real src/gng_scale.c UART/tare state machine.
// Build from the repository root:
// gcc -std=gnu11 -include tests/gng_tare_mocks/critical.h
//     -Itests/gng_tare_mocks -Isim/shims -Isrc
//     tests/gng_tare_test.c src/gng_scale.c -lm -o /tmp/gng_tare_test
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "scale.h"

extern scale_handle_t gng_scale_handle;

scale_config_t scale_config;
static jmp_buf test_end;
static TickType_t now_ms;
static TickType_t request_at_ms;
static TickType_t last_poll_ms;
static TickType_t last_tare_ms;
static float balance_weight;
static char rx_frame[14];
static unsigned rx_index;
static unsigned rx_length;
static unsigned tare_attempts;
static unsigned reject_first_n;
static unsigned polls_during_tare_quiet;
static bool requested;

TickType_t xTaskGetTickCount(void) { return now_ms; }

void vTaskDelay(TickType_t ticks) {
    now_ms += ticks;
    if (!requested && now_ms >= request_at_ms) {
        requested = true;
        gng_scale_handle.force_zero();
    }
    if (requested &&
        (gng_scale_handle.get_tare_status() == SCALE_TARE_SUCCESS ||
         gng_scale_handle.get_tare_status() == SCALE_TARE_FAILED)) {
        longjmp(test_end, 1);
    }
    if (now_ms > 7500) longjmp(test_end, 1);
}

bool uart_is_readable(void *uart) { (void)uart; return rx_index < rx_length; }
char uart_getc(void *uart) { (void)uart; return rx_frame[rx_index++]; }
void uart_tx_wait_blocking(void *uart) { (void)uart; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) { (void)sem; return pdTRUE; }

void scale_write(const char *command, size_t length) {
    if (length == 4 && memcmp(command, "!p\r\n", 4) == 0) {
        if (tare_attempts > 0 && now_ms - last_tare_ms < 500) {
            polls_during_tare_quiet++;
        }
        last_poll_ms = now_ms;
        memcpy(rx_frame, "  ", 2);
        char number[16];
        snprintf(number, sizeof(number), "%7.2f", balance_weight);
        memcpy(rx_frame + 2, number, 7);
        memcpy(rx_frame + 9, "GN ", 3);
        memcpy(rx_frame + 12, "\r\n", 2);
        rx_index = 0;
        rx_length = 14;
    } else if (length == 4 && memcmp(command, "!t\r\n", 4) == 0) {
        tare_attempts++;
        last_tare_ms = now_ms;
        // Model the observed JJ200 behavior: a beep alone is not success.
        // It ignores premature or deliberately rejected tare attempts.
        if (tare_attempts > reject_first_n && now_ms - last_poll_ms >= 100) {
            balance_weight = 0.0f;
        }
    } else {
        assert(!"Unexpected G&G UART command");
    }
}

static void test_tare(unsigned rejected_attempts, int expected_status, unsigned expected_tares) {
    now_ms = 0;
    request_at_ms = 150;
    last_poll_ms = 0;
    last_tare_ms = 0;
    balance_weight = 12.34f;
    rx_index = 0;
    rx_length = 0;
    tare_attempts = 0;
    reject_first_n = rejected_attempts;
    polls_during_tare_quiet = 0;
    requested = false;
    scale_config.scale_measurement_ready = (SemaphoreHandle_t)1;

    if (setjmp(test_end) == 0) {
        gng_scale_handle.read_loop_task(NULL);
    }
    assert(gng_scale_handle.get_tare_status() == expected_status);
    assert(tare_attempts == expected_tares);
    assert(polls_during_tare_quiet == 0);
    if (expected_status == SCALE_TARE_SUCCESS) {
        assert(fabsf(balance_weight) < 0.001f);
    }
}

int main(void) {
    test_tare(0, SCALE_TARE_SUCCESS, 1);
    test_tare(1, SCALE_TARE_SUCCESS, 2);
    test_tare(2, SCALE_TARE_FAILED, 2);
    puts("G&G tare UART serialization, retry and failure tests passed");
    return 0;
}
