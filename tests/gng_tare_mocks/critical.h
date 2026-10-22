// This driver harness runs in one host thread. Device builds use FreeRTOS.
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
