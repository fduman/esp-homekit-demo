#include "am2320.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "espressif/esp_common.h"
#include "espressif/sdk_private.h"

#define AM2320_RX_QUEUE_SIZE 10
#define AM2320_TASK_PRIORITY 9
#define AM2320_MEASURE 0

// AM2320_Event_Command
typedef struct
{
    const QueueHandle_t *resultQueue;
} am2320_command_t;

// Just works due to the fact that QueueHandle_t is a "void *"
static QueueHandle_t am2320_rx_queue[I2C_MAX_BUS] = {NULL};
static TaskHandle_t am2320_task_handle[I2C_MAX_BUS] = {NULL};

static bool am2320_informUser_Impl(const QueueHandle_t *resultQueue, am2320_temp_t temperature, am2320_humid_t humidity);

bool (*am2320_informUser)(const QueueHandle_t *resultQueue, am2320_temp_t temperature, am2320_humid_t humidity) = am2320_informUser_Impl;

static uint16_t crc16(uint8_t *ptr, unsigned int len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;

    while (len--)
    {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void am2320_wakeup(i2c_dev_t *dev)
{
#ifdef AM2320_DEBUG
    printf("Wake up command sent to sensor");
#endif
    i2c_start(dev->bus);
    i2c_write(dev->bus, dev->addr << 1);
    sdk_os_delay_us(800);
    i2c_stop(dev->bus);
}

static bool am2320_read(i2c_dev_t *dev, uint8_t start_address, uint8_t *buffer, uint8_t length)
{
    am2320_wakeup(dev);

    uint8_t function_code = 0x03;

    // Send command
    i2c_start(dev->bus);
    i2c_write(dev->bus, dev->addr << 1);
    i2c_write(dev->bus, function_code);
    i2c_write(dev->bus, start_address);
    i2c_write(dev->bus, length);
    i2c_stop(dev->bus);

    // Read data
    sdk_os_delay_us(1500);
    i2c_start(dev->bus);
    i2c_write(dev->bus, dev->addr << 1 | 1);
    sdk_os_delay_us(30);

    for (int i = 0; i < length + 2; i++)
    {
        buffer[i] = i2c_read(dev->bus, false);
    }

    uint16_t crc = i2c_read(dev->bus, false);
    crc |= (uint16_t)i2c_read(dev->bus, true) << 8;

    i2c_stop(dev->bus);

#ifdef AM2320_DEBUG
    printf("Data read succeeded: ") for (int i = 0; i < length + 2; i++)
    {
        printf("-%x", buffer[i]);
    }

    printf("\n");
#endif

    uint16_t calculated_crc = crc16(buffer, length + 2);

    if (crc != calculated_crc)
    {
#ifdef AM2320_DEBUG
        printf("Crc check failed\n");
#endif
        return false;
    }

    if (buffer[0] != function_code || buffer[1] != length)
    {
#ifdef AM2320_DEBUG
        printf("Result check failed\n");
#endif
        return false;
    }

    return true;
}

bool am2320_measure(i2c_dev_t *dev, am2320_temp_t *temperature, am2320_humid_t *humidity)
{
#ifdef AM2320_DEBUG
    printf("Reading sensor data..\n");
#endif
    uint8_t data[6];

    if (!am2320_read(dev, 0x00, data, sizeof(data) - 2))
    {
        return false;
    }

    int16_t temp = (((uint16_t)data[sizeof(data) - 2] << 8) | data[sizeof(data) - 1]);
    uint16_t humid = (((uint16_t)data[sizeof(data) - 4] << 8) | data[sizeof(data) - 3]);

    if (temp & 0x8000)
        temp = -(temp & 0x7fff);

#ifdef AM2320_DEBUG
    printf("Temperature: %d, Humidity: %u\n", temp, humid);
#endif

    *temperature = temp / 10.0;
    *humidity = humid / 10.0;

    return true;
}

bool am2320_is_available(i2c_dev_t *dev)
{
    uint8_t data[9];

    if (!am2320_read(dev, 0x08, data, sizeof(data) - 2))
    {
        return false;
    }

#ifdef AM2320_DEBUG
    printf("AM2320 sensor is ready to use\n");
#endif
    return true;
}

// I2C Driver Task
static void am2320_driver_task(void *pvParameters)
{
    // Data to be received from user
    am2320_command_t current_command;
    i2c_dev_t *dev = (i2c_dev_t *)pvParameters;

#ifdef AM2320_DEBUG
    printf("%s: Started Task\n", __FUNCTION__);
#endif

    while (1)
    {
        // Wait for user to insert commands
        if (xQueueReceive(am2320_rx_queue[dev->bus], &current_command, portMAX_DELAY) == pdTRUE)
        {
#ifdef AM2320_DEBUG
            printf("%s: Received user command 0x%p\n", __FUNCTION__, current_command.resultQueue);
#endif
            // use user provided queue
            if (current_command.resultQueue != NULL)
            {
                // Work on it ...
                am2320_temp_t temperature = 0.0;
                am2320_humid_t humidity = 0.0;

                if (am2320_measure(dev, &temperature, &humidity))
                {
                    if (!am2320_informUser(current_command.resultQueue, temperature, humidity))
                    {
                        // Failed to send info to user
                        printf("%s: Unable to inform user am2320_informUser returned \"false\"\n", __FUNCTION__);
                    }
                }
            }
        }
    }
}

static bool am2320_create_communication_queues(i2c_dev_t *dev)
{
    // Just create them once by bus
    if (am2320_rx_queue[dev->bus] == NULL)
        am2320_rx_queue[dev->bus] = xQueueCreate(AM2320_RX_QUEUE_SIZE, sizeof(am2320_result_t));

    return am2320_rx_queue[dev->bus] != NULL;
}

static bool am2320_createTask(i2c_dev_t *dev)
{
    // We already have a task
    portBASE_TYPE x = pdPASS;

    if (am2320_task_handle[dev->bus] == NULL)
    {
        x = xTaskCreate(am2320_driver_task, "am2320_driver_task", 256, (void *)dev, AM2320_TASK_PRIORITY, &am2320_task_handle[dev->bus]); //TODO: name task with i2c bus
    }
    return x == pdPASS;
}

// Default user inform implementation
static bool am2320_informUser_Impl(const QueueHandle_t *resultQueue, am2320_temp_t temperature, am2320_humid_t humidity)
{
    am2320_result_t result;

    result.temperature = temperature;
    result.humidity = humidity;

    return (xQueueSend(*resultQueue, &result, 0) == pdTRUE);
}

// Just init all needed queues
bool am2320_init(i2c_dev_t *dev)
{
    // 1. Create required queues
    bool result = false;

    if (am2320_create_communication_queues(dev))
    {
        if (am2320_is_available(dev))
        {
            // 3. Start driver task
            if (am2320_createTask(dev))
            {
                // We are finished
                result = true;
            }
        }
    }
    return result;
}

void am2320_trigger_measurement(i2c_dev_t *dev, const QueueHandle_t *resultQueue)
{
    am2320_command_t c;

    c.resultQueue = resultQueue;

    xQueueSend(am2320_rx_queue[dev->bus], &c, 0);
}
