/*
 * am2320.h
 *
 *  Created on: 23/03/2020
 *      Author: fduman
 */

#ifndef DRIVER_AM2320_H_
#define DRIVER_AM2320_H_

#include "stdint.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "i2c/i2c.h"

// Uncomment to enable debug output
//#define AM2320_DEBUG

#define AM2320_DEVICE_ADDRESS 0xB8 >> 1

#ifdef __cplusplus
extern "C"
{
#endif

    // temperature in °C
    typedef float am2320_temp_t;
    // humidity in %
    typedef float am2320_humid_t;

    typedef struct
    {
        am2320_temp_t temperature;
        am2320_humid_t humidity;
    } am2320_result_t;

    // Init am2320 driver ...
    bool am2320_init(i2c_dev_t *dev);

    // Trigger a "complete" measurement (temperature and pressure will be valid when given to "am2320_informUser)
    void am2320_trigger_measurement(i2c_dev_t *dev, const QueueHandle_t *resultQueue);

    // Give the user the chance to create it's own handler
    extern bool (*am2320_informUser)(const QueueHandle_t *resultQueue, am2320_temp_t temperature, am2320_humid_t humidity);

    // Returns true if the am2320 is detected.
    bool am2320_is_available(i2c_dev_t *dev);

    bool am2320_measure(i2c_dev_t *dev, am2320_temp_t *temperature, am2320_humid_t *humidity);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_AM2320_H_ */