/* Simple example for I2C / BMP180 / Timer & Event Handling
 *
 * This sample code is in the public domain.
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

// BMP180 driver
#include "am2320.h"

#define I2C_BUS 0

#define MY_EVT_TIMER 0x01
#define MY_EVT_AM2320 0x02

typedef struct
{
    uint8_t event_type;
    am2320_result_t am2320_data;
} my_event_t;

//device descriptor
i2c_dev_t dev = {
    .addr = AM2320_DEVICE_ADDRESS,
    .bus = I2C_BUS,
};

// Communication Queue
static QueueHandle_t mainqueue;
static TimerHandle_t timerHandle;

// Own BMP180 User Inform Implementation
bool am2320_i2c_informUser(const QueueHandle_t *resultQueue, uint8_t cmd, am2320_temp_t temperature, am2320_humid_t humidity)
{
    my_event_t ev;

    ev.event_type = MY_EVT_AM2320;
    ev.am2320_data.cmd = cmd;
    ev.am2320_data.temperature = temperature;
    ev.am2320_data.humidity = humidity;

    return (xQueueSend(*resultQueue, &ev, 0) == pdTRUE);
}

// Timer call back
static void am2320_i2c_timer_cb(TimerHandle_t xTimer)
{
    my_event_t ev;
    ev.event_type = MY_EVT_TIMER;

    xQueueSend(mainqueue, &ev, 0);
}

// Check for communiction events
void am2320_task(void *pvParameters)
{
    // Received pvParameters is communication queue
    QueueHandle_t *com_queue = (QueueHandle_t *)pvParameters;

    printf("%s: Started user interface task\n", __FUNCTION__);

    while (1)
    {
        my_event_t ev;

        xQueueReceive(*com_queue, &ev, portMAX_DELAY);

        switch (ev.event_type)
        {
        case MY_EVT_TIMER:
            printf("%s: Received Timer Event\n", __FUNCTION__);
            am2320_trigger_measurement(&dev, com_queue);
            break;
        case MY_EVT_AM2320:
            printf("%s: Received AM2320 Event\n", __FUNCTION__);
            printf("Temperature: %f, Humidity: %f\n", (float)ev.am2320_data.temperature, (float)ev.am2320_data.humidity);
            break;
        default:
            break;
        }
    }
}

// Setup HW
void user_setup(void)
{
    // Set UART Parameter
    uart_set_baud(0, 115200);

    // Give the UART some time to settle
    sdk_os_delay_us(500);

    // Init I2C bus Interface
    i2c_init(I2C_BUS, 5, 4, I2C_FREQ_100K);
}

void user_init(void)
{
    // Setup HW
    user_setup();

    // Just some infomations
    printf("\n");
    printf("SDK version : %s\n", sdk_system_get_sdk_version());

    // Use our user inform implementation
    am2320_informUser = am2320_i2c_informUser;

    // Init BMP180 Interface
    am2320_init(&dev);

    // Create Main Communication Queue
    mainqueue = xQueueCreate(10, sizeof(my_event_t));

    // Create user interface task
    xTaskCreate(am2320_task, "am2320_task", 256, &mainqueue, 2, NULL);

    // Create Timer (Trigger a measurement every 2 seconds)
    timerHandle = xTimerCreate("AM2320 Trigger", 3000 / portTICK_PERIOD_MS, pdTRUE, NULL, am2320_i2c_timer_cb);

    if (timerHandle != NULL)
    {
        if (xTimerStart(timerHandle, 0) != pdPASS)
        {
            printf("%s: Unable to start Timer ...\n", __FUNCTION__);
        }
    }
    else
    {
        printf("%s: Unable to create Timer ...\n", __FUNCTION__);
    }
}